/* nes_engine.c -- RP-A's CPU bus (Rev1). See nes_engine.h.
 *
 * core1, one M2 cycle at a time:
 *
 * Reads: the engine (PIO + DMA) answers $8000-$FFFF on its own; core1 only
 * looks up the cart's own pages ($5000-$5FFF, WRAM) and offers the byte to
 * nes_page_out, which drives it only if /ROMSEL stays high (not the ROM alias
 * at $C000-$FFFF). Spare time (after M2 falls) prepares the next MMC1 write
 * and follows WRAM gate changes in the page table.
 *
 * Writes: the engine is held (IRQ 4) at once. While M2 is high, MMC1 and
 * MMC3 work out what the write can do from the address; once M2 falls the
 * last sample taken while it was high is the data, the slots get their new
 * values (four stores at most, usually one), one is read back so they have
 * landed (in flash mode they are QMI registers, over APB), and the engine is
 * let go. Only then: the mapper's bookkeeping, RP-B's link frame, core0's
 * ring, WRAM. All of that before the next M2 rise is the aim; a write that
 * runs late makes only that next cycle late, which the engine serves anyway.
 *
 * Everything here runs from SRAM (the firmware is copy_to_ram).
 */

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/sio.h"

#include "nes_engine.h"
#include "nes_link.h"
#include "fuji_mailbox.h"
#include "nes_engine.pio.h"

/* Event kinds for nes_engine_note (nes_cart.h's NES_EV_*). */
#define EV_MAILBOX 0
#define EV_MAPPER  1

/* Where a write's data is. On the cart, D0-D7. The one-chip bench cannot let
 * its console drive D0-D7 (SIO owns them, for the reads), so its writes carry
 * the data in A1-A8. */
#ifdef NES_ENGINE_BENCH_DATA
#define WRITE_DATA(pins) ((uint8_t)((pins) >> 1))
#else
#define WRITE_DATA(pins) ((uint8_t)((pins) >> CD0_PIN))
#endif

static int ch_a = -1, ch_b, ch_c, ch_d;
static uint off_cap = ~0u, off_rom, off_page;
static const pio_program_t *cap_prog;

static inline uint32_t cyc(void)
{
    return sio_hw->mtime;            /* clk_sys, see nes_engine_init */
}

#pragma GCC push_options
#pragma GCC optimize("O3")

/* ---------------- the cart's own pages ---------------- */

static inline const uint8_t *wram_page(const nes_engine_t *e, unsigned p)
{
    return e->wram + (((p - 0x60) << 8) & (FN_WRAM_SIZE - 1));
}

void nes_engine_pages(nes_engine_t *e)
{
    unsigned p, w;

    for (w = 0; w < 2; w++) {
        for (p = 0; p < 0x50; p++)
            e->page[w][p] = NULL;
        for (p = 0x50; p < 0x60; p++) {
            unsigned off = (p - 0x50) << 8;

            if (off >= FN_LOADER || (off < FN_H_REGSEL && e->mailbox))
                e->page[w][p] = e->arena + off;    /* loader; reply + status */
            else
                e->page[w][p] = NULL;              /* hotspots: inert        */
        }
        for (p = 0x60; p < 0x80; p++)
            e->page[w][p] = w ? wram_page(e, p) : NULL;
    }
}

/* ---------------- the loop ---------------- */

#define PARK 0xFFFFu                 /* MAP for the parked loop */

/* A mapper write's bookkeeping, put off until the next read's spare time (or
 * the next write, whichever comes first): right after a write the next cycle
 * is already close, and the read in it may be one of core1's own pages. */
typedef struct {
    bool due;
    bool consec;
    uint16_t a;
    uint8_t v;
    uint32_t release;                /* cycles, M2 fall -> let go */
    fastprg_mmc3_plan_t p3;
} deferred_t;

static inline __attribute__((always_inline)) void
flush(nes_engine_t *e, deferred_t *w, const unsigned MAP, uint32_t cycle)
{
    fastprg_t *fp = &e->fp;

    w->due = false;
    if (MAP == 1)
        fastprg_mmc1_late(fp, w->a, w->v, w->consec);
    else if (MAP == 4)
        fastprg_mmc3_late(fp, w->v, &w->p3);
    e->writes++;
    if (MAP == 4 && w->a >= 0xC000)                /* the IRQ registers: core0 */
        nes_engine_note(w->a, w->v, EV_MAPPER, cycle);
    if (w->release > e->max_release) {
        e->max_release = w->release;
        e->worst_write = (uint32_t)w->a << 8 | w->v;
    }
}

static inline __attribute__((always_inline)) void eng_loop(nes_engine_t *e, const unsigned MAP)
{
    fastprg_t *fp = &e->fp;
    io_wo_32 *const page_tx = &e->pio->txf[NES_ENGINE_SM_PAGE];
    const uint32_t adjacent = e->adjacent;
    const uint32_t settle = e->romsel_settle;
    uint32_t last_w = 0, cycle = e->cycle;
    const uint8_t *const *page = e->page[fp->wram_en];
    bool pending = false;
    deferred_t w;

    w.due = false;

    for (;;) {
        uint32_t pins, t0;
#ifdef NES_ENGINE_DIAG
        unsigned path = 2;
#endif

        pins = sio_hw->gpio_in;
        if (pins & M2_MASK) {
            e->late++;
#ifdef NES_ENGINE_DIAG
            path = 3;
#endif
        } else {
            do
                pins = sio_hw->gpio_in;
            while (!(pins & M2_MASK));
        }
        t0 = cyc();

        if (pins & RW_MASK) {
            const uint8_t *q;

            /* A read: the engine has $8000+; the cart's own pages are core1's.
             * Offer the byte at once; never into the next cycle. */
            if ((pins & CA14_MASK) &&
                (q = page[(pins & CA_LO_MASK) >> 8 | ((pins >> CA13_PIN) & 3) << 5]) != NULL) {
                uint8_t v = q[pins & 0xFF];
                uint32_t dt;

                if (sio_hw->gpio_in & M2_MASK)
                    *page_tx = v;
                dt = cyc() - t0;
                if (dt > e->max_page)
                    e->max_page = dt;
#ifdef NES_ENGINE_DIAG
                e->hist[path][dt >= 128 ? 15 : dt >> 3]++;
#endif
            }
            /* Spare time, once M2 has fallen. The cycle count (core0's
             * watchdog, the mailbox's RMW check) is kept here, off the
             * offer's path. */
            while (sio_hw->gpio_in & M2_MASK)
                ;
            t0 = cyc();
            e->cycle = ++cycle;
            if (w.due) {
                flush(e, &w, MAP, cycle);
                pending = true;                    /* the WRAM gate may move */
            } else if (MAP == 1 && fp->m1_stale) {
                fastprg_mmc1_prepare(fp);          /* the next write's rows */
            } else if (pending) {
                page = e->page[fp->wram_en];       /* the WRAM gate */
                pending = false;
            } else if (MAP == PARK ? !e->park : e->park) {
                return;                            /* to nes_engine_core1 */
            }
            t0 = cyc() - t0;
            if (t0 > e->max_idle)
                e->max_idle = t0;
            continue;
        }

        /* A write. Hold the engine's next fetch until the slots are what
         * this write makes them. */
        e->pio->irq_force = 1u << NES_ENGINE_HOLD_IRQ;
        if (w.due)
            flush(e, &w, MAP, cycle);              /* an RMW's second write */
        {
            uint32_t prev, t1, t2;
            uint16_t a;
            uint8_t v;
            /* consec: MMC1's RMW rule, and the link frame's flag. From the M2
             * rises, the same spacing as the falls. */
            const bool consec = (t0 - last_w) < adjacent;
            fastprg_mmc1_plan_t p1;
            const uint8_t *const *cur = NULL;

            if (MAP == 4)
                fastprg_mmc3_early(fp, (uint16_t)(0x8000u | (pins & CA_LO_MASK) |
                                                  ((pins >> CA13_PIN) & 3u) << 13), &w.p3);
            if (MAP == 1) {
                fastprg_mmc1_early(fp, (uint16_t)(((pins >> CA13_PIN) & 3u) << 13), consec, &p1);
                cur = p1.now;
                /* Once /ROMSEL has settled: a ROM write puts in its likeliest
                 * row now, while the engine is held anyway. */
                while (cyc() - t0 < settle)
                    ;
                pins = sio_hw->gpio_in;
                if ((pins & M2_MASK) && !(pins & ROMSEL_MASK))
                    fastprg_mmc1_guess(fp, &p1);
            }
            if (!(sio_hw->gpio_in & M2_MASK))
                e->past_fall++;                    /* the data sample is late */
            do {
                prev = pins;
                pins = sio_hw->gpio_in;
            } while (pins & M2_MASK);
            t1 = cyc();
            a = (uint16_t)((prev & CA_LO_MASK) | (((prev >> CA13_PIN) & 3u) << 13));
            v = WRITE_DATA(prev);

            if (!(prev & ROMSEL_MASK)) {
                /* $8000-$FFFF: the mapper. Slots, read one back, let go. */
                a |= 0x8000u;
                if (MAP == 1)
                    fastprg_mmc1_commit(fp, v, &p1);
                else if (MAP == 4)
                    fastprg_mmc3_commit(fp, v, &w.p3);
                else if (MAP != PARK)
                    fastprg_write_m(fp, MAP == 0xFFFEu ? fp->mapper : MAP, a, v, consec);
                (void)fp->slot[3];
                e->pio->irq = 1u << NES_ENGINE_HOLD_IRQ;
                t2 = cyc();
                /* RP-B's frame now (CHR follows within the microsecond);
                 * the rest when there is time. */
                if (MAP != PARK) {
                    if (e->link_txf)
                        *e->link_txf = NES_LINK_WRITE(consec, a, v);
                    w.due = true;
                    w.a = a;
                    w.v = v;
                    w.consec = consec;
                    w.release = t2 - t1;
                }
                e->cycle = ++cycle;
                last_w = t0;
            } else if ((prev & (CA14_MASK | CA13_MASK)) == (CA14_MASK | CA13_MASK)) {
                /* $6000-$7FFF: WRAM, and NINA-001's registers at $7FFD-$7FFF. */
                e->cycle = ++cycle;
                if (MAP == 1 && p1.now != cur)
                    fastprg_mmc1_put(fp, cur);         /* guessed, yet not ROM */
                if ((MAP == 34 || MAP == 0xFFFEu) && fp->mapper == 34 && !fp->bnrom && a >= 0x7FFD) {
                    fastprg_write_m(fp, 34, a, v, consec);
                    (void)fp->slot[3];
                    e->pio->irq = 1u << NES_ENGINE_HOLD_IRQ;
                    last_w = t0;
                    if (e->link_txf)
                        *e->link_txf = NES_LINK_WRITE(consec, a, v);
                } else {
                    e->pio->irq = 1u << NES_ENGINE_HOLD_IRQ;
                }
                if (fp->wram_en && !fp->wram_wp && MAP != PARK)
                    e->wram[a & (FN_WRAM_SIZE - 1)] = v;
            } else {
                if (MAP == 1 && p1.now != cur)
                    fastprg_mmc1_put(fp, cur);
                e->pio->irq = 1u << NES_ENGINE_HOLD_IRQ;
                e->cycle = ++cycle;
                /* $5000-$5FFF: the mailbox's hotspots, for core0. */
                if ((a & 0x7000u) == 0x5000u && e->mailbox) {
                    unsigned off = a & (FN_ARENA_SIZE - 1);

                    if (off >= FN_H_REGSEL && off < FN_LOADER)
                        nes_engine_note((uint16_t)off, v, EV_MAILBOX, cycle);
                }
            }
            t2 = cyc() - t1;
            if (t2 > e->max_wapply)
                e->max_wapply = t2;
        }
    }
}

/* MMC1 and MMC3 get their own loops (their early halves); 0xFFFE is every
 * other mapper, dispatched per write; PARK serves pages and mailbox writes
 * only, while core0 sets the engine up anew. */
#define ELOOP(n, m) static void __not_in_flash_func(eloop_##n)(nes_engine_t *e) { eng_loop(e, m); }
ELOOP(1, 1)
ELOOP(4, 4)
ELOOP(any, 0xFFFEu)
ELOOP(park, PARK)

void __not_in_flash_func(nes_engine_core1)(nes_engine_t *e)
{
    for (;;) {
        if (e->park) {
            e->parked = 1;
            eloop_park(e);
            e->parked = 0;
            continue;
        }
        switch (e->fp.mapper) {
        case 1: eloop_1(e); break;
        case 4: eloop_4(e); break;
        default: eloop_any(e); break;
        }
    }
}

#pragma GCC pop_options

/* ---------------- setup ---------------- */

void nes_engine_init(nes_engine_t *e)
{
    PIO pio = e->pio;
    pio_sm_config c;
    unsigned p;

    sio_hw->mtime_ctrl = SIO_MTIME_CTRL_EN_BITS | SIO_MTIME_CTRL_FULLSPEED_BITS;
    off_rom = pio_add_program(pio, &nes_rom_out_program);
    off_page = pio_add_program(pio, &nes_page_out_program);
    if (ch_a < 0) {
        ch_a = dma_claim_unused_channel(true);
        ch_b = dma_claim_unused_channel(true);
        ch_c = dma_claim_unused_channel(true);
        ch_d = dma_claim_unused_channel(true);
    }
    for (p = CD0_PIN; p < CD0_PIN + 8; p++) {
        pio_gpio_init(pio, p);
        gpio_disable_pulls(p);
    }
    pio_sm_set_consecutive_pindirs(pio, NES_ENGINE_SM_PAGE, CD0_PIN, 8, false);

    /* The cart's own pages are served from here on, across every engine
     * start and stop: the loader runs from them while an image goes in. */
    c = nes_page_out_program_get_default_config(off_page);
    sm_config_set_out_pins(&c, CD0_PIN, 8);
    sm_config_set_jmp_pin(&c, ROMSEL_PIN);
    sm_config_set_out_shift(&c, true, false, 32);
    pio_sm_init(pio, NES_ENGINE_SM_PAGE, off_page, &c);
    pio_sm_set_enabled(pio, NES_ENGINE_SM_PAGE, true);
}

void nes_engine_start(nes_engine_t *e)
{
    PIO pio = e->pio;
    pio_sm_config c;
    dma_channel_config k;

    /* The capture program for this image: only one fits beside the others. */
    cap_prog = e->atrans ? &nes_capture_x_program : &nes_capture_program;
    off_cap = pio_add_program(pio, cap_prog);
    c = e->atrans ? nes_capture_x_program_get_default_config(off_cap)
                  : nes_capture_program_get_default_config(off_cap);
    sm_config_set_in_pins(&c, CA0_PIN);
    sm_config_set_in_pin_count(&c, 26);
    sm_config_set_jmp_pin(&c, RW_PIN);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_shift(&c, true, false, 32);
    pio_sm_init(pio, NES_ENGINE_SM_CAP, off_cap, &c);
    if (e->atrans) {
        pio_sm_put(pio, NES_ENGINE_SM_CAP, XIP_NOCACHE_NOALLOC_BASE >> 24);
        fastprg_mirror(&e->fp, (const uint8_t *volatile *)&qmi_hw->atrans[0]);
    } else {
        pio_sm_put(pio, NES_ENGINE_SM_CAP, (uint32_t)e->fp.slot >> 4);
    }

    c = nes_rom_out_program_get_default_config(off_rom);
    sm_config_set_out_pins(&c, CD0_PIN, 8);
    sm_config_set_jmp_pin(&c, ROMSEL_PIN);
    sm_config_set_out_shift(&c, true, false, 32);
    pio_sm_init(pio, NES_ENGINE_SM_ROM, off_rom, &c);

    pio->irq = 1u << NES_ENGINE_HOLD_IRQ;

    /* D: the byte -> nes_rom_out; then re-arm A. */
    k = dma_channel_get_default_config(ch_d);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_8);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, false);
    channel_config_set_chain_to(&k, ch_a);
    dma_channel_configure(ch_d, &k, &pio->txf[NES_ENGINE_SM_ROM], NULL, 1, false);
    /* C: the offset, ORed into D's READ_ADDR (banks are 8K-aligned); then D. */
    k = dma_channel_get_default_config(ch_c);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, false);
    channel_config_set_dreq(&k, pio_get_dreq(pio, NES_ENGINE_SM_CAP, false));
    channel_config_set_chain_to(&k, ch_d);
    dma_channel_configure(ch_c, &k, hw_set_alias(&dma_hw->ch[ch_d].read_addr),
                          &pio->rxf[NES_ENGINE_SM_CAP], 1, false);
    /* B: the slot pointer -> D's READ_ADDR; then C. */
    k = dma_channel_get_default_config(ch_b);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, false);
    channel_config_set_chain_to(&k, ch_c);
    dma_channel_configure(ch_b, &k, &dma_hw->ch[ch_d].read_addr, NULL, 1, false);
    /* A: RAM, the slot pointer's address -> B's READ_ADDR_TRIG; flash, the
     * byte's address -> D's READ_ADDR_TRIG. */
    k = dma_channel_get_default_config(ch_a);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, false);
    channel_config_set_dreq(&k, pio_get_dreq(pio, NES_ENGINE_SM_CAP, false));
    dma_channel_configure(ch_a, &k, e->atrans ? &dma_hw->ch[ch_d].al3_read_addr_trig
                                              : &dma_hw->ch[ch_b].al3_read_addr_trig,
                          &pio->rxf[NES_ENGINE_SM_CAP], 1, true);

    pio_enable_sm_mask_in_sync(pio, (1u << NES_ENGINE_SM_CAP) | (1u << NES_ENGINE_SM_ROM));
}

void nes_engine_stop(nes_engine_t *e)
{
    PIO pio = e->pio;
    unsigned i;

    /* No new fetches; then let nes_rom_out finish the byte it may be
     * driving (it lets go when M2 falls) before it stops. The page machine
     * keeps running. */
    pio_sm_set_enabled(pio, NES_ENGINE_SM_CAP, false);
    dma_channel_abort(ch_a);
    dma_channel_abort(ch_b);
    dma_channel_abort(ch_c);
    dma_channel_abort(ch_d);
    busy_wait_us(2);
    pio_sm_set_enabled(pio, NES_ENGINE_SM_ROM, false);
    pio_sm_clear_fifos(pio, NES_ENGINE_SM_CAP);
    pio_sm_clear_fifos(pio, NES_ENGINE_SM_ROM);
    pio->irq = 1u << NES_ENGINE_HOLD_IRQ;
    if (off_cap != ~0u) {
        pio_remove_program(pio, cap_prog, off_cap);
        off_cap = ~0u;
    }
    if (e->atrans) {
        /* The slots back in RAM (as they stand), flash back to identity. */
        for (i = 0; i < 4; i++)
            e->fp.slot_mem[i] = e->fp.slot[i];
        e->fp.slot = e->fp.slot_mem;
        for (i = 0; i < 4; i++)
            qmi_hw->atrans[i] = QMI_ATRANS0_RESET + i * 0x400u;
    }
}
