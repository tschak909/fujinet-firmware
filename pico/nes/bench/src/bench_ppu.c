/* bench_ppu.c -- the PPU-bus test: a 2C02 on PIO against RP-B's Rev1 loop
 * and its CIRAM table.
 *
 * Reads: the console state machine plays the PPU's fetch pattern (nametable,
 * attribute, pattern low, pattern high) at 186 ns a dot; one capture machine
 * samples PD0-PD7 Y cycles after /RD falls, another samples CIRAM A10 and
 * CIRAM /CE Y cycles after the address changes. Sweeping each Y finds when
 * the cart's answer is always there. The PPU latches PD when /RD rises, one
 * dot (186 ns) after it fell.
 *
 * Writes (CHR-RAM): PIO drives PD too, then core0 compares the DUT's RAM with
 * what the console wrote. Release: the DUT raises a marker right after it
 * lets go of PD, and a PIO stopwatch times /RD rising -> marker.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/structs/busctrl.h"
#include "hardware/structs/sio.h"

#include "bench.h"
#include "bench.pio.h"
#include "dut_ppu.h"
#include "nesmap.h"

static uint8_t nt_ram[4096];       /* four-screen nametables                 */
static uint8_t chr_ram[8192];      /* CHR-RAM for the write test             */
static uint8_t chr_oracle[8192];

enum { MODE_V = NESMAP_MIR_V, MODE_H = NESMAP_MIR_H, MODE_1LO = NESMAP_MIR_1LO,
       MODE_1HI = NESMAP_MIR_1HI, MODE_4 = NESMAP_MIR_4 };
static const char *const mode_name[] = { "v", "h", "1lo", "1hi", "four" };

static PIO pio = pio0, pio_c = pio1, pio_w = pio2;
static uint sm_stim = 0, sm_capd = 1, sm_capc = 2, sm_ciram = 0, sm_watch = 0;
static uint off_stim, off_capd, off_capc, off_watch;
static int dma_stim = -1, dma_capd = -1, dma_capc = -1;

static uint32_t phase_cnt;          /* ppu_console loop count per dot        */
static uint8_t chr_bank[8];
static int mode;

/* ---------------- the CIRAM table ---------------- */

static uint8_t ciram_side(int m, unsigned slot)
{
    unsigned pa13 = slot >> 3, pa11 = (slot >> 1) & 1, pa10 = slot & 1;
    unsigned ce_n = 1, a10 = 0;

    if (pa13 && m != MODE_4) {
        ce_n = 0;
        a10 = (m == MODE_V) ? pa10 : (m == MODE_H) ? pa11 : (m == MODE_1HI) ? 1 : 0;
    }
    return (uint8_t)((ce_n << 1) | a10);
}

static void ciram_patch(int m)
{
    unsigned i;

    for (i = 0; i < 16; i++)
        pio_c->instr_mem[i] = pio_encode_mov(pio_pc, pio_pins) | pio_encode_sideset(2, ciram_side(m, i));
}

/* ---------------- generator ---------------- */

typedef enum { KIND_READ, KIND_WRITE_HOLD, KIND_WRITE_NOHOLD, KIND_RELEASE } kind_t;

typedef struct {
    uint32_t rng;
    unsigned step;                  /* position in the 4-fetch pattern       */
    uint16_t tile_addr;
    uint8_t last_pd;
    kind_t kind;
} pgen_t;

static pgen_t pg;

/* bench_exp: 7-0 data, 8 data valid, 9-10 CIRAM value, 11-12 CIRAM mask, 29-16 address */
static uint32_t expect(uint16_t a)
{
    uint32_t e = (uint32_t)a << 16;
    unsigned slot = (a >> 10) & 15;
    uint8_t side = ciram_side(mode, slot);
    uint32_t cmask = 2u | ((a & 0x2000u) && mode != MODE_4 ? 1u : 0u);

    e |= ((uint32_t)side << 9) | (cmask << 11);
    if (!(a & 0x2000u))
        e |= 0x100u | chr_byte(chr_bank[slot] * 1024u + (a & 0x3FFu));
    else if (mode == MODE_4)
        e |= 0x100u | nt_ram[a & 0x0FFFu];
    return e;
}

static void emit(uint16_t a, bool read, uint8_t pd_addr_phase, uint8_t pd_data, uint32_t *w)
{
    uint32_t base = (phase_cnt << 24) | (a & PA_MASK);

    w[0] = base | RD_MASK | WR_MASK | ((uint32_t)pd_addr_phase << PD0_PIN);
    w[1] = base | (read ? WR_MASK : RD_MASK) | ((uint32_t)pd_data << PD0_PIN);
}

static void gen_access(uint32_t *w, uint32_t *e)
{
    uint32_t r = rnd(&pg.rng);
    uint16_t a;

    if (pg.kind == KIND_WRITE_HOLD || pg.kind == KIND_WRITE_NOHOLD) {
        uint8_t d = (uint8_t)(r >> 24);
        a = (uint16_t)((r >> 8) & 0x1FFFu);
        emit(a, false, pg.kind == KIND_WRITE_HOLD ? pg.last_pd : (uint8_t)(r >> 16), d, w);
        chr_oracle[a] = d;
        pg.last_pd = d;
        *e = 0;
        return;
    }
    if (pg.kind == KIND_RELEASE) {
        a = (uint16_t)((r >> 8) & 0x1FFFu);
    } else {
        switch (pg.step++ & 3) {
        case 0:                        /* nametable byte                     */
            a = (uint16_t)(0x2000u | ((r >> 8) & 0x0FFFu));
            pg.tile_addr = (uint16_t)(((r >> 20) & 0x1000u) | ((r >> 4) & 0x0FF0u) | ((r >> 28) & 7));
            break;
        case 1:                        /* attribute byte                     */
            a = (uint16_t)(0x23C0u | ((r >> 8) & 0x0C3Fu));
            break;
        case 2:                        /* pattern, low plane                 */
            a = pg.tile_addr;
            break;
        default:                       /* pattern, high plane                */
            a = (uint16_t)(pg.tile_addr | 8u);
            break;
        }
        if ((r & 0xFF) < 8)            /* now and then: anywhere at all      */
            a = (uint16_t)((r >> 8) & 0x3EFFu);
    }
    emit(a, true, 0, 0, w);
    *e = expect(a);
}

/* ---------------- setup ---------------- */

static void pins_setup(bool pd_to_pio)
{
    unsigned p;

    for (p = 0; p < 24; p++) {
        if (p >= PD0_PIN && p < PD0_PIN + 8 && !pd_to_pio) {
            gpio_init(p);
            gpio_set_dir(p, false);
            gpio_disable_pulls(p);
            gpio_set_drive_strength(p, GPIO_DRIVE_STRENGTH_4MA);
        } else {
            pio_gpio_init(pio, p);
            gpio_disable_pulls(p);
        }
    }
    pio_gpio_init(pio_c, CIRAM_A10_PIN);
    pio_gpio_init(pio_c, CIRAM_A10_PIN + 1);
    gpio_init(PPU_MARK_PIN);
    gpio_set_dir(PPU_MARK_PIN, true);
    gpio_put(PPU_MARK_PIN, 0);
}

static void stim_init(bool pd_to_pio)
{
    pio_sm_config c = ppu_console_program_get_default_config(off_stim);
    uint32_t mask = PA_MASK | RD_MASK | WR_MASK | (pd_to_pio ? PD_MASK : 0);

    sm_config_set_out_pins(&c, 0, 24);
    sm_config_set_out_shift(&c, false, false, 32);
    pio_sm_init(pio, sm_stim, off_stim, &c);
    pio_sm_set_pindirs_with_mask(pio, sm_stim, mask, mask);
    pio_sm_set_pins_with_mask(pio, sm_stim, RD_MASK | WR_MASK, mask);
}

static void capd_init(uint32_t y)
{
    pio_sm_config c = capture_irq0_program_get_default_config(off_capd);

    sm_config_set_in_pins(&c, PD0_PIN);
    sm_config_set_in_pin_count(&c, 8);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_out_shift(&c, false, false, 32);
    pio_sm_init(pio, sm_capd, off_capd, &c);
    pio_sm_put(pio, sm_capd, y);
}

static void capc_init(uint32_t y)
{
    pio_sm_config c = capture_irq1_program_get_default_config(off_capc);

    sm_config_set_in_pins(&c, CIRAM_A10_PIN);
    sm_config_set_in_pin_count(&c, 2);
    sm_config_set_in_shift(&c, false, true, 2);
    sm_config_set_out_shift(&c, false, false, 32);
    pio_sm_init(pio, sm_capc, off_capc, &c);
    pio_sm_put(pio, sm_capc, y);
}

static void ciram_init(void)
{
    pio_sm_config c = ciram_table_program_get_default_config(0);

    sm_config_set_in_pins(&c, PA0_PIN + 10);
    sm_config_set_in_pin_count(&c, 4);
    sm_config_set_sideset_pins(&c, CIRAM_A10_PIN);
    pio_sm_init(pio_c, sm_ciram, 0, &c);
    pio_sm_set_consecutive_pindirs(pio_c, sm_ciram, CIRAM_A10_PIN, 2, true);
    pio_sm_set_enabled(pio_c, sm_ciram, true);
}

/* The stopwatch, at offset 0 of PIO2: /RD rises -> count 2-cycle steps until
 * the marker is high -> push the count. */
static void watch_load(void)
{
    static uint16_t prog[8];
    pio_program_t q = { .instructions = prog, .length = 8, .origin = 0 };

    prog[0] = pio_encode_wait_gpio(false, RD_PIN);
    prog[1] = pio_encode_wait_gpio(true, RD_PIN);
    prog[2] = pio_encode_mov_not(pio_x, pio_null);
    prog[3] = pio_encode_jmp_pin(5);
    prog[4] = pio_encode_jmp_x_dec(3);
    prog[5] = pio_encode_mov_not(pio_isr, pio_x);
    prog[6] = pio_encode_push(false, false);
    prog[7] = pio_encode_jmp(0);
    off_watch = pio_add_program(pio_w, &q);
}

static void dma_ring_in(int ch, PIO p, uint sm, uint8_t *ring)
{
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 12);
    channel_config_set_dreq(&c, pio_get_dreq(p, sm, false));
    dma_channel_configure(ch, &c, ring, &p->rxf[sm], dma_encode_endless_transfer_count(), true);
}

static void dma_stim_start(bool endless)
{
    dma_channel_config c = dma_channel_get_default_config(dma_stim);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    if (endless)
        channel_config_set_ring(&c, false, 15);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_stim, true));
    dma_channel_configure(dma_stim, &c, &pio->txf[sm_stim], bench_stim,
                          endless ? dma_encode_endless_transfer_count() : 2 * BLOCK, true);
}

static void stop_all(void)
{
    pio_sm_set_enabled(pio, sm_stim, false);
    pio_sm_set_enabled(pio, sm_capd, false);
    pio_sm_set_enabled(pio, sm_capc, false);
    pio_sm_set_enabled(pio_w, sm_watch, false);
    dma_channel_abort(dma_stim);
    dma_channel_abort(dma_capd);
    dma_channel_abort(dma_capc);
    multicore_reset_core1();
    pio_sm_clear_fifos(pio, sm_stim);
    pio_sm_clear_fifos(pio, sm_capd);
    pio_sm_clear_fifos(pio, sm_capc);
    pio_sm_clear_fifos(pio_w, sm_watch);
    pio_interrupt_clear(pio, 0);
    pio_interrupt_clear(pio, 1);
    sio_hw->gpio_oe_clr = PD_MASK;
    busctrl_hw->priority = 0;
}

/* ---------------- one run ---------------- */

typedef struct {
    uint32_t checked, derr, cerr, werr, wmax, wsum, wn;
    uint32_t bad[2][2];
    uint32_t nbad;
} prun_t;

static void slots_for(kind_t kind)
{
    bool wr = (kind == KIND_WRITE_HOLD || kind == KIND_WRITE_NOHOLD);
    unsigned i;

    for (i = 0; i < 16; i++) {
        dut_ppu.rslot[i] = NULL;
        dut_ppu.wslot[i] = NULL;
    }
    for (i = 0; i < 8; i++) {
        if (wr) {
            dut_ppu.rslot[i] = chr_ram + i * 1024u;
            dut_ppu.wslot[i] = chr_ram + i * 1024u;
        } else {
            dut_ppu.rslot[i] = bench_big + chr_bank[i] * 1024u;
        }
    }
    if (!wr && mode == MODE_4)
        for (i = 8; i < 16; i++)
            dut_ppu.rslot[i] = nt_ram + (i & 3) * 1024u;
}

static void start_dut(kind_t kind)
{
    busctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
    dut_ppu.running = 0;
    multicore_launch_core1(kind == KIND_RELEASE ? dut_ppu_main_marker : dut_ppu_main);
    while (!dut_ppu.running)
        tight_loop_contents();
}

/* CHR-RAM writes: blocks of BLOCK - 1 writes and one idle access (so /WR
 * ends high), each played once; then the DUT's RAM against the oracle. */
static prun_t run_writes(kind_t kind, uint32_t n, uint32_t seed)
{
    prun_t r;
    uint32_t pass, i;

    memset(&r, 0, sizeof r);
    memset(&pg, 0, sizeof pg);
    pg.rng = seed | 1;
    pg.kind = kind;
    memset(chr_ram, 0, sizeof chr_ram);
    memset(chr_oracle, 0, sizeof chr_oracle);
    slots_for(kind);
    ciram_patch(mode);
    pins_setup(true);
    start_dut(kind);

    for (pass = 0; pass * (BLOCK - 1) < n; pass++) {
        for (i = 0; i < BLOCK - 1; i++)
            gen_access(&bench_stim[2 * i], &bench_exp[i]);
        emit(0, true, pg.last_pd, pg.last_pd, &bench_stim[2 * i]);
        bench_stim[2 * i + 1] |= RD_MASK;      /* no strobe at all */
        stim_init(true);
        pio_sm_set_enabled(pio, sm_stim, true);
        dma_stim_start(false);
        dma_channel_wait_for_finish_blocking(dma_stim);
        while (!pio_sm_is_tx_fifo_empty(pio, sm_stim))
            tight_loop_contents();
        busy_wait_us(5);
        pio_sm_set_enabled(pio, sm_stim, false);
        r.checked += BLOCK - 1;
    }
    stop_all();
    for (i = 0; i < sizeof chr_ram; i++)
        if (chr_ram[i] != chr_oracle[i])
            r.werr++;
    pins_setup(false);
    return r;
}

/* Reads (and the release stopwatch): one block, replayed until n checked. */
static prun_t run(kind_t kind, uint32_t yd, uint32_t yc, uint32_t n, uint32_t seed)
{
    prun_t r;
    uint32_t cap = 0, last_wd, capc = 0, last_wc;
    unsigned i;

    if (kind == KIND_WRITE_HOLD || kind == KIND_WRITE_NOHOLD)
        return run_writes(kind, n, seed);

    memset(&r, 0, sizeof r);
    memset(&pg, 0, sizeof pg);
    pg.rng = seed | 1;
    pg.kind = kind;
    for (i = 0; i < 8; i++)
        chr_bank[i] = (uint8_t)(rnd(&pg.rng) & 0xFF);
    slots_for(kind);
    ciram_patch(mode);
    pins_setup(false);
    for (i = 0; i < BLOCK; i++)
        gen_access(&bench_stim[2 * i], &bench_exp[i]);

    stim_init(false);
    if (kind == KIND_RELEASE) {
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, off_watch, off_watch + 7);
        sm_config_set_jmp_pin(&c, PPU_MARK_PIN);
        sm_config_set_in_shift(&c, false, false, 32);
        pio_sm_init(pio_w, sm_watch, off_watch, &c);
        pio_sm_set_enabled(pio_w, sm_watch, true);
        dma_ring_in(dma_capd, pio_w, sm_watch, bench_cap0);
    } else {
        capd_init(yd);
        capc_init(yc);
        pio_sm_set_enabled(pio, sm_capd, true);
        pio_sm_set_enabled(pio, sm_capc, true);
        dma_ring_in(dma_capd, pio, sm_capd, bench_cap0);
        dma_ring_in(dma_capc, pio, sm_capc, bench_cap1);
    }
    last_wd = (uint32_t)dma_hw->ch[dma_capd].write_addr;
    last_wc = (uint32_t)dma_hw->ch[dma_capc].write_addr;

    start_dut(kind);
    pio_sm_set_enabled(pio, sm_stim, true);
    dma_stim_start(true);

    while (r.checked < n) {
        uint32_t wd = dma_hw->ch[dma_capd].write_addr;

        cap += (wd - last_wd) % BLOCK;
        last_wd = wd;
        if (kind == KIND_RELEASE) {
            while (r.checked + 2 < cap && r.checked < n) {
                uint8_t v = bench_cap0[r.checked % BLOCK];
                if (v > r.wmax)
                    r.wmax = v;
                r.wsum += v;
                r.wn++;
                r.checked++;
            }
            continue;
        }
        {
            uint32_t wc = dma_hw->ch[dma_capc].write_addr;
            capc += (wc - last_wc) % BLOCK;
            last_wc = wc;
        }
        while (r.checked + 2 < cap && r.checked + 2 < capc && r.checked < n) {
            uint32_t e = bench_exp[r.checked % BLOCK];
            uint8_t gd = bench_cap0[r.checked % BLOCK];
            uint8_t gc = bench_cap1[r.checked % BLOCK];
            uint32_t cmask = (e >> 11) & 3, cval = (e >> 9) & 3;

            if ((e & 0x100u) && gd != (uint8_t)e) {
                if (r.nbad < 2) {
                    r.bad[r.nbad][0] = e;
                    r.bad[r.nbad][1] = gd;
                    r.nbad++;
                }
                r.derr++;
            }
            if ((gc & cmask) != (cval & cmask))
                r.cerr++;
            r.checked++;
        }
    }
    stop_all();
    return r;
}

static prun_t run_long(kind_t kind, uint32_t yd, uint32_t yc, uint32_t n, uint32_t seed)
{
    prun_t t, r;
    unsigned k;

    memset(&t, 0, sizeof t);
    for (k = 0; k < 4; k++) {
        r = run(kind, yd, yc, n / 4, seed + k * 977u);
        t.checked += r.checked;
        t.derr += r.derr;
        t.cerr += r.cerr;
        for (unsigned j = 0; j < r.nbad && t.nbad < 2; j++) {
            t.bad[t.nbad][0] = r.bad[j][0];
            t.bad[t.nbad][1] = r.bad[j][1];
            t.nbad++;
        }
    }
    return t;
}

/* ---------------- the sweeps ---------------- */

static uint32_t sweep(kind_t kind, bool data, uint32_t ymax, uint32_t seed, prun_t *last)
{
    uint32_t lo = 0, hi = ymax, y;
    prun_t r;

    r = data ? run(kind, hi, ymax, 200000, seed) : run(kind, ymax, hi, 200000, seed);
    if (data ? r.derr : r.cerr) {
        *last = r;
        return 0xFFFFFFFFu;
    }
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2;
        r = data ? run(kind, mid, ymax, 200000, seed + mid) : run(kind, ymax, mid, 200000, seed + mid);
        if ((data ? r.derr : r.cerr) == 0)
            hi = mid;
        else
            lo = mid + 1;
    }
    for (y = lo; y <= ymax; y++) {
        r = data ? run_long(kind, y, ymax, bench_confirm_cycles, seed ^ 0x55u)
                 : run_long(kind, ymax, y, bench_confirm_cycles, seed ^ 0x55u);
        if ((data ? r.derr : r.cerr) == 0)
            break;
    }
    *last = r;
    return y;
}

void bench_ppu(void)
{
    static bool loaded;
    const int32_t offset = 2;        /* the capture machines' fixed latency; bench_cpu calibrates it */
    uint32_t phase, ymax;
    int m;
    prun_t r;

    bench_pio_reset();
    off_stim = pio_add_program(pio, &ppu_console_program);
    off_capd = pio_add_program(pio, &capture_irq0_program);
    off_capc = pio_add_program(pio, &capture_irq1_program);
    pio_add_program_at_offset(pio_c, &ciram_table_program, 0);
    watch_load();
    if (!loaded) {
        dma_stim = dma_claim_unused_channel(true);
        dma_capd = dma_claim_unused_channel(true);
        dma_capc = dma_claim_unused_channel(true);
        for (unsigned i = 0; i < sizeof nt_ram; i++)
            nt_ram[i] = hbyte(i ^ NT_SEED);
        loaded = true;
    }
    for (uint32_t i = 0; i < PRG_RAM_SIZE; i++)
        bench_big[i] = chr_byte(i);

    phase_cnt = (ns_to_cycles(186) - 6) / 2;
    phase = 2 * phase_cnt + 6;
    ymax = phase - 4;
    mode = MODE_V;
    pins_setup(false);
    ciram_patch(mode);
    ciram_init();

    REPORT("R ppu_cal mhz=%lu dot_ns=%lu offset_cyc=%ld\n", (unsigned long)bench_mhz,
           (unsigned long)cycles_to_ns(phase), (long)offset);

    for (m = MODE_V; m <= MODE_4; m++) {
        uint32_t yd, yc;
        prun_t rd, rc;

        mode = m;
        yd = sweep(KIND_READ, true, ymax, 0x1000u + (uint32_t)m, &rd);
        yc = sweep(KIND_READ, false, ymax, 0x2000u + (uint32_t)m, &rc);
        {
            char d_ns[12], c_ns[12];

            if (yd == 0xFFFFFFFFu)
                strcpy(d_ns, "none");
            else
                snprintf(d_ns, sizeof d_ns, "%lu", (unsigned long)cycles_to_ns(yd + offset));
            if (yc == 0xFFFFFFFFu)
                strcpy(c_ns, "none");
            else
                snprintf(c_ns, sizeof c_ns, "%lu", (unsigned long)cycles_to_ns(yc + offset));
            REPORT("R ppu read mode=%s mhz=%lu rd_to_data_ns=%s ciram_ns=%s derr=%lu cerr=%lu checked=%lu\n",
                   mode_name[m], (unsigned long)bench_mhz, d_ns, c_ns,
                   (unsigned long)rd.derr, (unsigned long)rc.cerr, (unsigned long)rd.checked);
        }
        if (rd.nbad)
            REPORT("R ppu_bad mode=%s addr=%04lx exp=%02lx got=%02lx\n", mode_name[m],
                   (unsigned long)(rd.bad[0][0] >> 16), (unsigned long)(rd.bad[0][0] & 0xFF),
                   (unsigned long)rd.bad[0][1]);
    }

    mode = MODE_V;
    r = run(KIND_WRITE_HOLD, ymax, ymax, bench_confirm_cycles / 4, 0x3000u);
    REPORT("R ppu write hold mhz=%lu writes=%lu werr=%lu\n", (unsigned long)bench_mhz,
           (unsigned long)r.checked, (unsigned long)r.werr);
    r = run(KIND_WRITE_NOHOLD, ymax, ymax, bench_confirm_cycles / 4, 0x3100u);
    REPORT("R ppu write nohold mhz=%lu writes=%lu werr=%lu\n", (unsigned long)bench_mhz,
           (unsigned long)r.checked, (unsigned long)r.werr);

    r = run(KIND_RELEASE, ymax, ymax, 1000000, 0x4000u);
    REPORT("R ppu release mhz=%lu max_ns=%lu avg_ns=%lu samples=%lu\n",
           (unsigned long)bench_mhz, (unsigned long)cycles_to_ns(2 * r.wmax + 1),
           (unsigned long)(r.wn ? cycles_to_ns((2 * r.wsum + r.wn) / r.wn) : 0),
           (unsigned long)r.wn);
    pins_setup(false);
}
