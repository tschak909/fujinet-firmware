/* fuji_cart.c -- core0's half of the cartridge (Rev1): the ring drain, the
 * install sequence, console power and the watchdog.
 *
 * Before a game, and whenever the console comes back on, the engine serves
 * the BOOT BANK in all four PRG slots: an 8K-aligned bank whose second half
 * is the mailbox arena, so its last page -- $FF00-$FFFF -- is the loader
 * ROM's last page, which carries the cart's vectors (RESET $5800, NMI and IRQ
 * on the RTI at $5803). The console starts in the loader, asks, and core0
 * installs the image under it: CHR to RP-B over the link, the engine's banks
 * pointed at the store. The loader never copies anything; it sees LOAD_DONE,
 * acknowledges, and jumps through $FFFC into the game.
 */

#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/io_bank0.h"

#include "fuji_cart.h"
#include "fuji_mailbox.h"
#include "fujimail.h"
#include "nes_engine.h"
#include "nes_irq.h"
#include "nes_link.h"
#include "nes_link_tx.h"

extern nes_engine_t nes_eng;                   /* nes_cart.c */

fuji_ring_t fuji_ring;
/* One 8K bank, as Rev0: the engine serves an 8K window (banked boards later). */
uint8_t fuji_wram[FN_WRAM_SIZE];
volatile nes_serve_t fuji_serve;
nesmap_t fuji_map;
volatile bool fuji_boot_armed;
volatile bool fuji_have_staged;

/* The boot bank: [scratch 4K][arena 4K], 8K-aligned (see the top). */
static struct {
    uint8_t scratch[8192 - FN_ARENA_SIZE];
    uint8_t arena[FN_ARENA_SIZE];
} boot_bank __attribute__((aligned(8192)));
_Static_assert(FN_ARENA_SIZE == 4096, "the boot bank is half arena");

uint8_t *const fuji_arena = boot_bank.arena;
uint8_t *const fuji_scratch4k = boot_bank.scratch;

/* The engine's bank table (one entry per 8K PRG bank) and MMC1 rows. */
static const uint8_t *bank_ptr[NESMAP_PRG_MAX / 8192];
static const uint8_t *mmc1_rows[FASTPRG_MMC1_ROWS][4];

static fuji_image_t resident_img;
static nesmap_plan_t resident_plan;
static fuji_image_t staged_img;
static nesmap_plan_t staged_plan;
static const uint8_t *live_body;               /* the store the engine serves */
static uint8_t diag_rmw;

/* The load in progress. */
static enum { LS_IDLE, LS_REQ, LS_DONE } ls_state;
static fuji_image_t ls_img;
static nesmap_plan_t ls_plan;
static bool ls_autoload;                       /* boot the resident image when the console runs */

/* Console power: M2 stops, CONFIG must come back. */
static uint32_t wd_last_cycle;
static absolute_time_t wd_last_change;
static bool console_alive;
static bool pwrok_seen;

/* ---------------- the IRQ line, the LED, the bus gate ---------------- */

void fuji_cart_set_irq(bool asserted)
{
    if (asserted) {
        gpio_put(IRQ_PIN, 0);
        gpio_set_dir(IRQ_PIN, true);
    } else {
        gpio_set_dir(IRQ_PIN, false);          /* the console's pull-up   */
    }
}

/* D0-D7 and /IRQ may drive only while the console is powered: with only USB
 * up, driving them would feed the console through its own pins. OEOVER
 * holds the output enables off whatever PIO or SIO asks. */
static void bus_gate(bool on)
{
    unsigned p;
    uint32_t v = on ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_LOW;

    for (p = CD0_PIN; p < CD0_PIN + 8; p++)
        gpio_set_oeover(p, v);
    gpio_set_oeover(IRQ_PIN, v);
}

/* ---------------- the ring ---------------- */

uint32_t fuji_cart_bus_cycle(void)
{
    return nes_eng.cycle;
}

/* Only an event at least two bus cycles old is handed out, so that the write
 * of the cycle after it -- an RMW's real write -- is already in the ring and
 * the dummy write can be recognised: same offset, adjacent cycle. */
bool fuji_cart_next_event(nes_event_t *ev)
{
    uint16_t tail = fuji_ring.tail;
    uint16_t head = fuji_ring.head;
    nes_event_t e;

    if (tail == head)
        return false;
    e.offset = fuji_ring.buf[tail].offset;
    e.data = fuji_ring.buf[tail].data;
    e.kind = fuji_ring.buf[tail].kind;
    e.cycle = fuji_ring.buf[tail].cycle;
    if ((int32_t)(fuji_cart_bus_cycle() - e.cycle) < 2 && console_alive)
        return false;
    fuji_ring.tail = (uint16_t)((tail + 1u) % FUJI_RING_LEN);

    if (e.kind == NES_EV_MAILBOX && fuji_ring.tail != head) {
        uint16_t n = fuji_ring.tail;
        if (fuji_ring.buf[n].kind == NES_EV_MAILBOX
            && fuji_ring.buf[n].offset == e.offset
            && fuji_ring.buf[n].cycle == e.cycle + 1) {
            /* the dummy write of an RMW: drop it, count it, hand out the real one */
            diag_rmw++;
            fuji_cart_poke(FN_R_DIAG_RMW, diag_rmw);
            return fuji_cart_next_event(ev);
        }
    }
    *ev = e;
    return true;
}

/* Publish one mailbox byte into the arena. The loader ROM half is never
 * written this way. */
void fuji_cart_poke(unsigned offset, uint8_t value)
{
    if (offset < FN_R_PAINT_END)
        fuji_arena[offset] = value;
}

/* ---------------- the engine ---------------- */

/* core1 to the parked loop (its own pages and mailbox writes only), or
 * back. core1 notices between bus cycles, so this needs the console
 * running; with it off, core1 is reset instead (see console_off). */
static void park(bool on)
{
    nes_eng.park = on;
    while (nes_eng.parked != on)
        tight_loop_contents();
}

/* All four slots on the boot bank, no mapper: the power-on state. core1
 * must not be running (not launched yet, or reset). */
static void engine_boot(void)
{
    fastprg_t *fp = &nes_eng.fp;
    unsigned i;

    nes_engine_stop(&nes_eng);
    memset(fp, 0, sizeof *fp);
    fp->slot = fp->slot_mem;
    for (i = 0; i < 4; i++)
        fp->slot_mem[i] = (const uint8_t *)&boot_bank;
    nes_eng.atrans = false;
    nes_eng.mailbox = true;
    nes_engine_pages(&nes_eng);
    nes_engine_start(&nes_eng);
    live_body = NULL;
    fuji_serve.prg_live = false;
    fuji_serve.mailbox = true;
    nes_eng.park = 0;
    nes_eng.parked = 0;
}

/* The engine for an image (core1 parked): its banks, the tier's path. */
static bool engine_image(const fuji_image_t *img, const nesmap_plan_t *plan)
{
    fastprg_t *fp = &nes_eng.fp;
    uint32_t b, n = plan->prg_size / 8192;

    nes_engine_stop(&nes_eng);
    for (b = 0; b < n; b++)
        bank_ptr[b] = img->flash_off ? nes_engine_atrans_bank(img->flash_off + b * 8192)
                                     : img->body + b * 8192;
    if (nesmap_init(&fuji_map, plan) != NESMAP_OK)
        return false;
    if (!fastprg_init_tab(fp, &fuji_map, bank_ptr, mmc1_rows))
        return false;
    nes_eng.atrans = img->flash_off != 0;
    nes_engine_pages(&nes_eng);                /* WRAM in or out per the gate */
    nes_engine_start(&nes_eng);
    live_body = img->body;
    return true;
}

/* ---------------- staging ---------------- */

void fuji_cart_set_resident(const fuji_image_t *image, const nesmap_plan_t *plan)
{
    resident_img = *image;
    resident_plan = *plan;
}

void fuji_cart_stage(const fuji_image_t *image, const nesmap_plan_t *plan)
{
    staged_img = *image;
    staged_plan = *plan;
    fuji_have_staged = true;
}

bool fuji_cart_store_busy(const uint8_t *body)
{
    if (fuji_have_staged && body == staged_img.body)
        return true;
    if (ls_state != LS_IDLE && body == ls_img.body)
        return true;
    return body == live_body;
}

/* ---------------- the install ---------------- */

static void load_fail(void)
{
    fuji_cart_poke(FN_R_LOAD_STATE, FN_LOAD_FAILED);
    ls_state = LS_IDLE;
}

/* The console is in the loader, polling LOAD_STATE out of the arena. */
static void install(const fuji_image_t *img, const nesmap_plan_t *plan)
{
    const uint8_t *chr = !plan->chr_size ? NULL : img->chr ? img->chr : img->body + plan->prg_size;

    ls_img = *img;
    ls_plan = *plan;
    ls_state = LS_REQ;
    fuji_have_staged = false;
    fuji_boot_armed = false;
    fuji_serve.loading = true;
    fuji_cart_set_irq(false);
    nes_irq_enable(false);
    fuji_cart_poke(FN_R_BOOT_STATE, FN_BOOT_IDLE);
    fuji_cart_poke(FN_R_LOAD_PCT, 0);
    fuji_cart_poke(FN_R_LOAD_STATE, FN_LOAD_IDLE);

    park(true);
    /* CHR first: RP-B reads from the store while the engine still serves
     * the boot bank (identity flash windows). */
    if (!nes_link_send_image(img->hdr, chr, plan->chr_size)) {
        park(false);
        load_fail();
        return;
    }
    if (!engine_image(img, plan)) {
        /* Not an image fastprg can serve: back to the boot bank's slots,
         * the loader still running from the arena. */
        nes_eng.fp.mapper = 0;
        nes_eng.fp.slot = nes_eng.fp.slot_mem;
        for (unsigned i = 0; i < 4; i++)
            nes_eng.fp.slot_mem[i] = (const uint8_t *)&boot_bank;
        nes_eng.atrans = false;
        nes_engine_start(&nes_eng);
        park(false);
        load_fail();
        return;
    }
    nes_link_send(NES_LINK_MODE(NES_LINK_MODE_SERVE));
    fuji_serve.prg_live = true;
    fuji_serve.loading = false;
    park(false);
    nes_irq_enable(fuji_map.desc->irq == NESMAP_IRQ_A12);
    gpio_put(LED_PIN, 1);

    /* The mailbox stays live only for an image that claims it. It is not
     * switched off here: the loader still has to read FN_LOAD_DONE, and it
     * says goodbye with one more SLICE_ACK. */
    if (plan->fuji_claim && !plan->claims_5000)
        fujimail_paint();                      /* ACKSEQ restarts for the new client */
    fuji_cart_poke(FN_R_MAPPER, (uint8_t)plan->mapper);
    fuji_cart_poke(FN_R_LOAD_PCT, 100);
    fuji_cart_poke(FN_R_LOAD_STATE, FN_LOAD_DONE);
    ls_state = LS_DONE;
}

/* The loader's request. A client armed a staged image with BOOTLOCK, or --
 * at power-on and after a console power-cycle -- the loader was entered
 * through the boot bank's vectors and the resident CONFIG is what it gets. */
void fuji_cart_request_load(void)
{
    if (ls_state != LS_IDLE)
        return;
    if (fuji_boot_armed && fuji_have_staged)
        install(&staged_img, &staged_plan);
    else if (ls_autoload && resident_img.body) {
        ls_autoload = false;
        install(&resident_img, &resident_plan);
    }
}

void fuji_cart_slice_acked(void)
{
    if (ls_state != LS_DONE)
        return;
    /* The loader is about to JMP ($FFFC). Its last read of the arena is
     * done, so the mailbox pages can go now. */
    fuji_serve.mailbox = ls_plan.fuji_claim && !ls_plan.claims_5000;
    if (!fuji_serve.mailbox) {
        nes_eng.mailbox = false;
        nes_engine_pages(&nes_eng);            /* word stores: safe live */
    }
    fuji_cart_poke(FN_R_LOAD_STATE, FN_LOAD_IDLE);
    ls_state = LS_IDLE;
}

/* ---------------- console power ---------------- */

static void console_off(void)
{
    console_alive = false;
    ls_state = LS_IDLE;
    fuji_serve.loading = false;
    fuji_cart_set_irq(false);
    nes_irq_enable(false);
    if (nes_link_rpb_ready())
        nes_link_send(NES_LINK_MODE(NES_LINK_MODE_OFF));
    /* core1 is waiting for an M2 edge that will not come: reset it, put the
     * vectors back for next time, and start it again. */
    multicore_reset_core1();
    engine_boot();
    multicore_launch_core1(nes_core1_main);
    gpio_put(LED_PIN, 0);
    ls_autoload = true;                        /* CONFIG again when it comes back */
}

static void console_on(void)
{
    console_alive = true;                      /* the loader will ask */
}

/* ---------------- init and service ---------------- */

void fuji_cart_init(const uint8_t *loader_rom, unsigned loader_len)
{
    memset((void *)&fuji_ring, 0, sizeof fuji_ring);
    memset(&boot_bank, 0, sizeof boot_bank);
    memset(fuji_wram, 0, sizeof fuji_wram);
    if (loader_len > FN_LOADER_SIZE)
        loader_len = FN_LOADER_SIZE;
    memcpy(fuji_arena + FN_LOADER, loader_rom, loader_len);

    /* The vectors, at the loader's last six bytes ($5FFA-$5FFF), which the
     * boot bank serves at $FFFA-$FFFF: RESET into the loader, NMI and IRQ
     * onto the RTI at its third byte (loader.s keeps them there). */
    fuji_arena[FN_ARENA_SIZE - 6] = 0x03; fuji_arena[FN_ARENA_SIZE - 5] = 0x58;
    fuji_arena[FN_ARENA_SIZE - 4] = 0x00; fuji_arena[FN_ARENA_SIZE - 3] = 0x58;
    fuji_arena[FN_ARENA_SIZE - 2] = 0x03; fuji_arena[FN_ARENA_SIZE - 1] = 0x58;

    fuji_serve.arena = fuji_arena;
    fuji_serve.wram = fuji_wram;
    fuji_serve.wram_size = FN_WRAM_SIZE;
    fuji_serve.vectors = fuji_arena + FN_ARENA_SIZE - 256;
    fuji_serve.prg_live = false;
    fuji_serve.mailbox = true;
    fuji_serve.wram_en = false;
    fuji_serve.wram_wp = false;
    fuji_serve.loading = false;
    fuji_boot_armed = false;
    fuji_have_staged = false;
    ls_state = LS_IDLE;
    ls_autoload = true;
    console_alive = false;
    wd_last_cycle = 0;
    wd_last_change = get_absolute_time();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, true);
    gpio_put(LED_PIN, 0);
    gpio_init(PWROK_PIN);
    gpio_set_dir(PWROK_PIN, false);
    gpio_disable_pulls(PWROK_PIN);
    pwrok_seen = false;
    bus_gate(false);

    nes_eng.wram = fuji_wram;
    nes_eng.arena = fuji_arena;
    nes_eng.romsel_settle = (uint32_t)(40ull * clock_get_hz(clk_sys) / 1000000000ull) + 1;
    nes_eng.adjacent = (uint32_t)(838ull * clock_get_hz(clk_sys) / 1000000000ull);
    nes_eng.link_txf = nes_link_tx_fifo();
    nes_eng.pio = pio0;
    nes_engine_init(&nes_eng);
    engine_boot();                             /* core1 launches after this */
}

void fuji_cart_service(void)
{
    uint32_t cyc = fuji_cart_bus_cycle();
    absolute_time_t now = get_absolute_time();
    bool pwrok = gpio_get(PWROK_PIN);

    /* The bus gate follows the console's 5 V. */
    if (pwrok != pwrok_seen) {
        pwrok_seen = pwrok;
        bus_gate(pwrok);
    }

    /* The watchdog: M2 has stopped for 250 ms means the console is off. */
    if (cyc != wd_last_cycle) {
        wd_last_cycle = cyc;
        wd_last_change = now;
        if (!console_alive)
            console_on();
    } else if (console_alive
               && absolute_time_diff_us(wd_last_change, now) > 250000) {
        console_off();
    }
}
