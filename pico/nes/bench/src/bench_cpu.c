/* bench_cpu.c -- the CPU-bus test: a 2A03 on PIO against RP-A's Rev1 loop.
 *
 * core0 generates one block of bus cycles (BLOCK of them) and, with its own
 * mapper engine and its own WRAM copy, works out what the cart must answer
 * on every read. An endless DMA ring replays the block into the console state
 * machine, and the capture state machine samples D0-D7 Y cycles after every
 * M2 rising edge into a ring of the same length, so capture i is cycle
 * i mod BLOCK and core0 only compares. A run is good when every byte the
 * cart had to drive was there in time.
 *
 * Blocks are state-closed: each starts by writing every mapper register to a
 * known value, WRAM reads only touch bytes this block wrote first (or the
 * $7000-$7FFF half, which no block writes), and each ends on reads. So every
 * pass of the ring expects exactly what the first did.
 *
 * Sweeping Y finds the earliest moment the data is always valid: that is the
 * number the 2A03 cares about (it samples at M2 falling, 350 ns after the
 * rise, and wants ~50 ns of setup).
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/structs/busctrl.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/sio.h"
#include "hardware/regs/addressmap.h"

#include "bench.h"
#include "bench.pio.h"
#include "dut_cpu.h"
#include "fuji_mailbox.h"
#include "nesmap.h"
#include "nes_fast.h"

static uint8_t wram_dut[FN_WRAM_SIZE], wram_oracle[FN_WRAM_SIZE];
static uint8_t arena[FN_ARENA_SIZE];
static uint32_t stress_src[1024] __attribute__((aligned(4096)));
static uint32_t stress_dst;

#define EXP_VALID (1u << 24)

/* With the ATRANS engine the slots are ATRANS values, not pointers: each 8K
 * bank's flash sector and an 8K aperture. The engine's own MMC1 table, as
 * replay_check rebuilds fastprg_init's with pointers. */
static const uint8_t *atr_bank[64];
static const uint8_t *atr_mmc1[FASTPRG_MMC1_ROWS][4];

typedef enum { BACK_RAM, BACK_XIP, BACK_HYBRID } backing_t;

typedef struct {
    const char *name;
    uint16_t mapper;
    uint32_t prg_size;
    backing_t backing;
    uint8_t write_pct, wram_pct, mcu_pct, other_pct, seq_pct;
    uint8_t lag_ns;                   /* /ROMSEL after M2                  */
    uint8_t cooldown;                 /* QMI COOLDOWN while it runs        */
} cpu_scn_t;

static const cpu_scn_t scenarios[] = {
    { "ram_romonly", 4, 256u * 1024, BACK_RAM,    0,  0, 0,  0,  0, 20, 0 },
    { "ram_rand",    4, 256u * 1024, BACK_RAM,    0, 10, 5, 15,  0, 20, 0 },
    { "ram_mmc3",    4, 256u * 1024, BACK_RAM,    3, 10, 5, 15, 70, 20, 0 },
    { "ram_mmc3_l40",4, 256u * 1024, BACK_RAM,    3, 10, 5, 15, 70, 40, 0 },
    { "ram_mmc1",    1, 256u * 1024, BACK_RAM,    6, 10, 5, 15, 70, 20, 0 },
    { "ram_unrom",   2, 256u * 1024, BACK_RAM,    5, 10, 5, 15, 70, 20, 0 },
    { "xip_mmc3",    4, 512u * 1024, BACK_XIP,    3, 10, 5, 15, 70, 20, 0 },
    { "xip_mmc3_l40",4, 512u * 1024, BACK_XIP,    3, 10, 5, 15, 70, 40, 0 },
    { "xip_mmc3_cd1",4, 512u * 1024, BACK_XIP,    3, 10, 5, 15, 70, 20, 1 },
    { "xip_rand",    4, 512u * 1024, BACK_XIP,    0,  0, 0,  0,  0, 20, 0 },
    { "hyb_mmc3",    4, 512u * 1024, BACK_HYBRID, 3, 10, 5, 15, 70, 20, 0 },
    { "hyb_mmc1",    1, 512u * 1024, BACK_HYBRID, 6, 10, 5, 15, 70, 20, 0 },
    { "xip_mmc1",    1, 512u * 1024, BACK_XIP,    6, 10, 5, 15, 70, 20, 0 },
};

/* ---------------- timing ---------------- */

typedef struct {
    uint32_t low, lag, high;          /* the console program's loop counts */
    uint32_t low_ns, high_ns, lag_ns; /* what they come to                 */
} cpu_timing_t;

static cpu_timing_t timing_for(uint32_t lag_ns)
{
    cpu_timing_t t;
    int32_t lo = ((int32_t)ns_to_cycles(209) - 5) / 2;
    int32_t hi = (int32_t)ns_to_cycles(350);
    int32_t lg = ((int32_t)ns_to_cycles(lag_ns) - 4) / 2;

    if (lg < 0)
        lg = 0;
    t.low = (uint32_t)lo;
    t.lag = (uint32_t)lg;
    t.high = (uint32_t)((hi - 10) / 2 - lg);
    t.low_ns = cycles_to_ns(2 * t.low + 5);
    t.high_ns = cycles_to_ns(2 * (t.lag + t.high) + 10);
    t.lag_ns = cycles_to_ns(2 * t.lag + 4);
    return t;
}

/* ---------------- the generator and its oracle ---------------- */

typedef struct {
    const cpu_scn_t *s;
    cpu_timing_t t;
    uint32_t rng;
    uint16_t pc;
    uint32_t i;                       /* cycles in the block so far        */
    nesmap_t map;
    uint16_t written[64];             /* WRAM bytes this block wrote       */
    unsigned nwritten;
    bool in_prologue;                 /* banks not yet known: read RAM     */
    unsigned reads_due;               /* reads still owed since the last write */
} gen_t;

static gen_t gen;

static inline uint32_t img(uint16_t a, bool rw)
{
    return (a & CA_LO_MASK) | ((uint32_t)((a >> 13) & 3u) << CA13_PIN) | (rw ? RW_MASK : 0);
}

static void put(uint16_t a, bool rw, uint32_t e)
{
    uint32_t i = img(a, rw);

    bench_stim[2 * gen.i] = (gen.t.low << 26) | i | ROMSEL_MASK;
    bench_stim[2 * gen.i + 1] = (gen.t.lag << 26) | i | M2_MASK | ((a & 0x8000u) ? 0 : ROMSEL_MASK);
    bench_exp[gen.i] = ((uint32_t)a << 8) | e;
    gen.i++;
}

static uint32_t expect_read(uint16_t a)
{
    if (a & 0x8000u) {
        uint32_t bank = gen.map.out.prg[(a >> 13) & 3];
        return EXP_VALID | prg_byte(bank * 8192u + (a & 0x1FFFu));
    }
    if ((a & 0xE000u) == 0x6000u)
        return gen.map.out.wram_en ? EXP_VALID | wram_oracle[a & (FN_WRAM_SIZE - 1)] : 0;
    if ((a & 0xF000u) == 0x5000u) {
        unsigned off = a & (FN_ARENA_SIZE - 1);
        if (off >= FN_LOADER || off < FN_R_PAINT_END)
            return EXP_VALID | arena[off];
    }
    return 0;
}

static void put_read(uint16_t a)
{
    put(a, true, expect_read(a));
}

static void put_rom_read(void)
{
    uint32_t r = rnd(&gen.rng);
    uint16_t a;

    if (((r >> 8) % 100) < gen.s->seq_pct)
        a = gen.pc++;
    else
        a = gen.pc = (uint16_t)(r >> 12);
    a |= 0x8000u;
    gen.pc |= 0x8000u;
    put_read(a);
}

/* A write (its data rides in A1-A8), then the fetch that always follows.
 * A 6502 puts at least three reads between two separate writes to
 * $8000-$FFFF (the next STA's opcode and address); the generator honours
 * that, and spends the first of them here. */
static void put_write(uint16_t a, uint8_t d)
{
    a = (uint16_t)((a & ~0x01FEu) | ((uint16_t)d << 1));
    put(a, false, 0);
    if ((a & 0xE000u) == 0x6000u && gen.map.out.wram_en && !gen.map.out.wram_wp) {
        wram_oracle[a & (FN_WRAM_SIZE - 1)] = d;
        if (gen.nwritten < 64)
            gen.written[gen.nwritten++] = a;
        else
            gen.written[rnd(&gen.rng) & 63] = a;
    }
    if (a >= gen.map.desc->write_lo) {
        nesmap_write(&gen.map, a, d, gen.i);    /* gen.i is already the next cycle: */
        gen.map.dirty = 0;                       /* the DUT counts from 1 the same  */
    }
    if (gen.in_prologue) {
        put((uint16_t)(rnd(&gen.rng) & 0x07FFu), true, 0);   /* console RAM: unchecked */
        put((uint16_t)(rnd(&gen.rng) & 0x07FFu), true, 0);
        put((uint16_t)(rnd(&gen.rng) & 0x07FFu), true, 0);
    } else {
        put_rom_read();
        gen.reads_due = 2;
    }
}

/* Every register to a known value, so a pass starts where the last did. */
static void prologue(void)
{
    unsigned r, k;

    /* Until the registers are set, the banks are whatever the last pass
     * left: the reads in here go to console RAM, which nothing checks. And
     * never a write on a pass's first cycle: MMC1's consecutive-write rule
     * would see the last pass. */
    gen.in_prologue = true;
    put(0x0000, true, 0);
    switch (gen.s->mapper) {
    case 4:
        for (r = 0; r < 8; r++) {
            static const uint8_t init[8] = { 0, 2, 4, 5, 6, 7, 0, 1 };
            put_write(0x8000, (uint8_t)r);
            put_write(0x8001, init[r]);
        }
        put_write(0x8000, 0);
        put_write(0xA000, 0);
        put_write(0xA001, 0x80);
        break;
    case 1: {
        static const uint8_t val[4] = { 0x0E, 0, 0, 0 };   /* ctrl, chr0, chr1, prg */
        put_write(0x8000, 0x80);
        for (r = 0; r < 4; r++)
            for (k = 0; k < 5; k++)
                put_write((uint16_t)(0x8000u | (r << 13)), (uint8_t)((val[r] >> k) & 1));
        break;
    }
    default:
        put_write(0x8000, 0);
        break;
    }
    gen.in_prologue = false;
}

static void mapper_write_random(void)
{
    uint32_t r = rnd(&gen.rng);
    uint8_t d = (uint8_t)(r >> 8);
    uint16_t a;

    switch (gen.s->mapper) {
    case 4: {
        unsigned k = (r >> 16) % 100;
        if (k < 30)       a = 0x8000;
        else if (k < 80)  a = 0x8001;
        else if (k < 85)  a = 0xA000;
        else if (k < 90) { a = 0xA001; d = (k == 86) ? 0xC0 : 0x80; }
        else              a = (uint16_t)(0xC000u | ((r >> 24) & 0x2001u));
        break;
    }
    case 1:
        a = (uint16_t)(0x8000u | ((r >> 16) & 0x6000u));
        d = (((r >> 20) & 63) == 0) ? 0x80 : (d & 0x7F);
        break;
    default:
        a = (uint16_t)(0x8000u | ((r >> 16) & 0x6000u));
        break;
    }
    put_write(a, d);
}

static void gen_block(const cpu_scn_t *s, uint32_t seed)
{
    nesmap_plan_t p;
    unsigned i;

    memset(&gen, 0, sizeof gen);
    gen.s = s;
    gen.t = timing_for(s->lag_ns);
    gen.rng = seed | 1;
    gen.pc = 0x8000;

    memset(&p, 0, sizeof p);
    p.mapper = s->mapper;
    p.prg_size = s->prg_size;
    p.chr_ram_size = 8192;
    p.prg_ram_size = 8192;
    p.mirror = NESMAP_MIR_V;
    p.image_size = 16 + s->prg_size;
    nesmap_init(&gen.map, &p);
    gen.map.dirty = 0;
    nesmap_init(&dut_cpu.map, &p);
    dut_cpu.map.dirty = 0;
    if (dut_cpu.eng.atrans)                 /* RP-A's half */
        fastprg_init_tab(&dut_cpu.eng.fp, &dut_cpu.map, atr_bank, atr_mmc1);
    else
        fastprg_init(&dut_cpu.eng.fp, &dut_cpu.map, dut_cpu.bank_ptr);
    for (i = 0; i < FN_WRAM_SIZE; i++)
        wram_oracle[i] = wram_dut[i] = hbyte(i ^ 0xAAAA0000u);

    prologue();
    while (gen.i < BLOCK - 4) {
        uint32_t r = rnd(&gen.rng);
        unsigned k = r % 100;

        if (gen.reads_due) {
            gen.reads_due--;
            put_rom_read();
            continue;
        }
        if (k < s->write_pct) {
            mapper_write_random();
            continue;
        }
        k -= s->write_pct;
        if (k < s->wram_pct) {
            if ((r >> 31) && gen.nwritten) {
                put_read(gen.written[(r >> 8) % gen.nwritten]);       /* written this block */
            } else if (r & 0x40000000u) {
                put_read((uint16_t)(0x7000u | ((r >> 8) & 0x0FFFu)));  /* never written     */
            } else {
                put_write((uint16_t)(0x6000u | ((r >> 8) & 0x0E01u)), (uint8_t)(r >> 16));
            }
        } else if ((k -= s->wram_pct) < s->mcu_pct) {
            uint16_t a = (uint16_t)(0x5000u | ((r >> 8) & 0x0FFFu));
            if ((a & 0x0F00u) >= 0x0500u && (a & 0x0F00u) < 0x0800u)
                a = (uint16_t)(a | 0x0800u);           /* reads only: skip the hotspots */
            put_read(a);
        } else if ((k -= s->mcu_pct) < s->other_pct) {
            put((uint16_t)((r >> 8) & 0x27FFu), true, 0);   /* console RAM or PPU regs */
        } else {
            put_rom_read();
        }
    }
    while (gen.i < BLOCK)
        put_rom_read();
}

/* Offline: run the DUT's fast engine over the block as core1 would (one
 * idle call per read, the writes in order) and count ROM reads whose byte
 * it would get wrong. Zero means any error on the bus is timing. */
static uint32_t block_rom_writes;

static uint32_t replay_check(const cpu_scn_t *s)
{
    static fastprg_t f;
    nesmap_plan_t p;
    nesmap_t m0;
    uint32_t i, bad = 0;

    memset(&p, 0, sizeof p);
    p.mapper = s->mapper;
    p.prg_size = s->prg_size;
    p.chr_ram_size = 8192;
    p.prg_ram_size = 8192;
    p.mirror = NESMAP_MIR_V;
    p.image_size = 16 + s->prg_size;
    nesmap_init(&m0, &p);
    fastprg_init(&f, &m0, dut_cpu.bank_ptr);
    block_rom_writes = 0;
    for (i = 0; i < BLOCK; i++) {
        uint32_t w1 = bench_stim[2 * i + 1], e = bench_exp[i];
        uint16_t a = (uint16_t)((w1 & CA_LO_MASK) | (((w1 >> CA13_PIN) & 3u) << 13));
        bool rom = !(w1 & ROMSEL_MASK);

        if (!(w1 & RW_MASK)) {
            if (rom) {
                fastprg_write(&f, (uint16_t)(a | 0x8000u), (uint8_t)(a >> 1), false);
                block_rom_writes++;
            }
            continue;
        }
        if (rom && (e & EXP_VALID) && f.slot[a >> 13][a & 0x1FFF] != (uint8_t)e)
            bad++;
    }
    return bad;
}

/* ---------------- setup ---------------- */

static PIO pio = pio0, pio_link = pio1;
static uint sm_stim = 0, sm_cap = 1, sm_sink = 0;
static uint off_stim, off_cap, off_sink;
static int dma_stim = -1, dma_cap = -1, dma_stress = -1;

static void pins_setup(void)
{
    unsigned p;

    for (p = 0; p < 26; p++) {
        if (p >= CD0_PIN && p < CD0_PIN + 8) {
            gpio_init(p);                         /* SIO: the DUT's D0-D7 */
            gpio_set_dir(p, false);
            gpio_disable_pulls(p);
            gpio_set_drive_strength(p, GPIO_DRIVE_STRENGTH_4MA);
        } else {
            pio_gpio_init(pio, p);
            gpio_disable_pulls(p);
        }
    }
}

static void stim_init(void)
{
    pio_sm_config c = cpu_console_program_get_default_config(off_stim);

    sm_config_set_out_pins(&c, 0, 26);
    sm_config_set_set_pins(&c, M2_PIN, 1);
    sm_config_set_out_shift(&c, false, false, 32);
    pio_sm_init(pio, sm_stim, off_stim, &c);
    pio_sm_set_pindirs_with_mask(pio, sm_stim, 0x03E01FFFu, 0x03E01FFFu);
    pio_sm_set_pins_with_mask(pio, sm_stim, ROMSEL_MASK, 0x03E01FFFu);
}

static void cap_init(uint in_base, uint32_t y)
{
    pio_sm_config c = capture_irq0_program_get_default_config(off_cap);

    sm_config_set_in_pins(&c, in_base);
    sm_config_set_in_pin_count(&c, 8);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_out_shift(&c, false, false, 32);
    pio_sm_init(pio, sm_cap, off_cap, &c);
    pio_sm_put(pio, sm_cap, y);
}

static void dma_start(void)
{
    dma_channel_config c;

    c = dma_channel_get_default_config(dma_cap);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 12);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_cap, false));
    dma_channel_configure(dma_cap, &c, bench_cap0, &pio->rxf[sm_cap],
                          dma_encode_endless_transfer_count(), true);

    c = dma_channel_get_default_config(dma_stim);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 15);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_stim, true));
    dma_channel_configure(dma_stim, &c, &pio->txf[sm_stim], bench_stim,
                          dma_encode_endless_transfer_count(), false);
}

static void stress_start(void)
{
    dma_channel_config c;

    if (!bench_stress)
        return;
    c = dma_channel_get_default_config(dma_stress);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 12);
    channel_config_set_dreq(&c, DREQ_FORCE);
    dma_channel_configure(dma_stress, &c, &stress_dst, stress_src,
                          dma_encode_endless_transfer_count(), true);
}

static void stop_all(void)
{
    dut_engine_stop();
    pio_sm_set_enabled(pio, sm_stim, false);
    pio_sm_set_enabled(pio, sm_cap, false);
    dma_channel_abort(dma_stim);
    dma_channel_abort(dma_cap);
    if (dma_stress >= 0)
        dma_channel_abort(dma_stress);
    multicore_reset_core1();
    pio_sm_clear_fifos(pio, sm_stim);
    pio_sm_clear_fifos(pio, sm_cap);
    pio_interrupt_clear(pio, 0);
    sio_hw->gpio_oe_clr = DATA_MASK;
    busctrl_hw->priority = 0;
}

static void set_cooldown(uint32_t cd)
{
    uint32_t t = qmi_hw->m[0].timing;

    t = (t & ~QMI_M0_TIMING_COOLDOWN_BITS) | (cd << QMI_M0_TIMING_COOLDOWN_LSB);
    qmi_hw->m[0].timing = t;
}

static void images_for(const cpu_scn_t *s)
{
    uint32_t i, b;
    uint32_t nbank = s->prg_size / 8192u;
    uint32_t ram_first = (s->backing == BACK_HYBRID) ? nbank - 32 : 0;

    if (s->backing == BACK_RAM)
        for (i = 0; i < s->prg_size; i++)
            bench_big[i] = prg_byte(i);
    else if (s->backing == BACK_HYBRID)
        for (i = 0; i < PRG_RAM_SIZE; i++)
            bench_big[i] = prg_byte(ram_first * 8192u + i);

    for (b = 0; b < 64; b++) {
        uint32_t bb = b % nbank;
        if (s->backing == BACK_RAM)
            dut_cpu.bank_ptr[b] = bench_big + bb * 8192u;
        else if (s->backing == BACK_XIP || bb < ram_first)
            dut_cpu.bank_ptr[b] = (const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + FLASH_IMG_OFF + bb * 8192u);
        else
            dut_cpu.bank_ptr[b] = bench_big + (bb - ram_first) * 8192u;
        atr_bank[b] = (const uint8_t *)(uintptr_t)(((FLASH_IMG_OFF + bb * 8192u) >> 12) |
                                                   (2u << QMI_ATRANS0_SIZE_LSB));
    }
}

/* ---------------- one run ---------------- */

typedef struct {
    uint32_t errors, checked, late, max_drive, max_wapply, cycles_seen, max_load;
    uint32_t hist[4][16];
    uint32_t first_bad[4][3];
    uint32_t nbad;
} run_result_t;

static const cpu_scn_t *images_of;

static run_result_t run(const cpu_scn_t *s, uint32_t y, uint32_t ncycles, uint32_t seed)
{
    run_result_t r;
    uint32_t captured = 0, last_wr;

    memset(&r, 0, sizeof r);
    dut_cpu.eng.atrans = bench_atrans;
    if (images_of != s) {
        images_for(s);
        images_of = s;
    }
    gen_block(s, seed);
    {
        static const cpu_scn_t *checked;
        if (checked != s) {
            checked = s;
            REPORT("R cpu_replay %s bad=%lu\n", s->name, (unsigned long)replay_check(s));
        }
    }

    dut_cpu.eng.wram = wram_dut;
    dut_cpu.eng.arena = arena;
    dut_cpu.eng.mailbox = true;
    nes_engine_pages(&dut_cpu.eng);
    dut_cpu.eng.romsel_settle = ns_to_cycles(40);
    dut_cpu.eng.adjacent = ns_to_cycles(838);       /* 1.5 bus cycles */
    dut_cpu.eng.link_txf = &pio_link->txf[sm_sink];
    dut_cpu.eng.park = 0;
    dut_cpu.eng.late = dut_cpu.eng.max_page = dut_cpu.eng.max_wapply = dut_cpu.eng.writes = 0;
    memset((void *)dut_cpu.eng.hist, 0, sizeof dut_cpu.eng.hist);
    dut_cpu.running = 0;
    set_cooldown(s->cooldown);

    stim_init();
    cap_init(CD0_PIN, y);
    pio_sm_set_enabled(pio, sm_cap, true);
    dma_start();
    last_wr = (uint32_t)dma_hw->ch[dma_cap].write_addr;

    busctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS |
                           BUSCTRL_BUS_PRIORITY_DMA_W_BITS;
    dut_engine_start();
    multicore_launch_core1(dut_engine_main);
    while (!dut_cpu.running)
        tight_loop_contents();
    stress_start();

    pio_sm_put(pio, sm_stim, gen.t.high);
    pio_sm_set_enabled(pio, sm_stim, true);
    dma_channel_start(dma_stim);

    while (r.checked < ncycles) {
        uint32_t wr = dma_hw->ch[dma_cap].write_addr;

        captured += (wr - last_wr) % BLOCK;
        last_wr = wr;
        while (r.checked + 2 < captured && r.checked < ncycles) {
            uint32_t e = bench_exp[r.checked % BLOCK];
            uint8_t got = bench_cap0[r.checked % BLOCK];

            if ((e & EXP_VALID) && got != (uint8_t)e) {
                if (r.nbad < 4) {
                    r.first_bad[r.nbad][0] = r.checked;
                    r.first_bad[r.nbad][1] = e;
                    r.first_bad[r.nbad][2] = got;
                    r.nbad++;
                }
                r.errors++;
            }
            r.checked++;
        }
    }

    r.late = dut_cpu.eng.late;
    r.max_drive = dut_cpu.eng.max_page;
    r.max_wapply = dut_cpu.eng.max_wapply;
    r.cycles_seen = 0;
    r.max_load = 0;
    memcpy(r.hist, (const void *)dut_cpu.eng.hist, sizeof r.hist);
    {
        uint32_t seen = dut_cpu.eng.writes;

        /* After the stop: with ATRANS the image's own window is gone until
         * dut_engine_stop puts the identity mapping back. */
        stop_all();
        {
            static const cpu_scn_t *said;
            if (said != s) {
                said = s;
                replay_check(s);
                REPORT("R cpu_writes %s seen=%lu expected=%lu\n", s->name, (unsigned long)seen,
                       (unsigned long)((uint64_t)block_rom_writes * r.checked / BLOCK));
            }
        }
    }
    return r;
}

/* Several blocks, each from its own seed: the long confirmation run. */
static run_result_t run_long(const cpu_scn_t *s, uint32_t y, uint32_t ncycles, uint32_t seed)
{
    run_result_t t, r;
    unsigned k;

    memset(&t, 0, sizeof t);
    for (k = 0; k < 4; k++) {
        r = run(s, y, ncycles / 4, seed + k * 977u);
        t.errors += r.errors;
        t.checked += r.checked;
        t.late += r.late;
        for (unsigned p = 0; p < 4; p++)
            for (unsigned j = 0; j < 16; j++)
                t.hist[p][j] += r.hist[p][j];
        if (r.max_drive > t.max_drive)
            t.max_drive = r.max_drive;
        if (r.max_wapply > t.max_wapply)
            t.max_wapply = r.max_wapply;
        for (unsigned j = 0; j < r.nbad && t.nbad < 4; j++) {
            memcpy(t.first_bad[t.nbad], r.first_bad[j], sizeof t.first_bad[0]);
            t.nbad++;
        }
    }
    return t;
}

/* ---------------- calibration ---------------- */

/* Where does a capture of delay Y sample, in cycles after M2 rises? Drive a
 * stream where CA0 changes at a known instruction after M2 rises (the lag
 * `out`), and find the smallest Y that sees the new value. */
static int32_t calibrate(void)
{
    const uint32_t lag = 10;
    uint32_t edge = 2 * lag + 4;
    uint32_t y, k;
    cpu_timing_t t = timing_for(20);
    uint32_t tot = t.lag + t.high;

    for (y = 0; y < edge + 16; y++) {
        uint32_t seen = 0;

        stim_init();
        cap_init(CA0_PIN, y);
        pio_sm_set_enabled(pio, sm_cap, true);
        pio_sm_put(pio, sm_stim, tot > lag ? tot - lag : 0);
        pio_sm_set_enabled(pio, sm_stim, true);
        for (k = 0; k < 64; k++) {
            pio_sm_put_blocking(pio, sm_stim, (t.low << 26) | ROMSEL_MASK);
            pio_sm_put_blocking(pio, sm_stim, (lag << 26) | M2_MASK | ROMSEL_MASK | 1u);
            if (k >= 4 && (pio_sm_get_blocking(pio, sm_cap) & 1))
                seen++;
            else if (k < 4)
                (void)pio_sm_get_blocking(pio, sm_cap);
        }
        pio_sm_set_enabled(pio, sm_stim, false);
        pio_sm_set_enabled(pio, sm_cap, false);
        pio_sm_clear_fifos(pio, sm_stim);
        pio_sm_clear_fifos(pio, sm_cap);
        pio_interrupt_clear(pio, 0);
        if (seen >= 58)
            return (int32_t)edge - (int32_t)y;
    }
    return 0;
}

/* ---------------- the sweep ---------------- */

static const uint32_t probe_cycles = 200000;

static void report_hist(const char *name, const run_result_t *r)
{
    static const char *const pn[4] = { "rom", "spec", "page", "late" };

    REPORT("R cpu_dbg %s fall_to_release_ns=%lu worst_aw=%05lx guess_past_fall=%lu idle_ns=%lu\n", name,
           (unsigned long)cycles_to_ns(dut_cpu.eng.max_release), (unsigned long)dut_cpu.eng.worst_write,
           (unsigned long)dut_cpu.eng.past_fall, (unsigned long)cycles_to_ns(dut_cpu.eng.max_idle));
    dut_cpu.eng.max_release = dut_cpu.eng.worst_write = dut_cpu.eng.past_fall = dut_cpu.eng.max_idle = 0;

    for (unsigned p = 0; p < 4; p++) {
        char buf[200];
        int n = 0;

        for (unsigned k = 0; k < 16; k++)
            if (r->hist[p][k])
                n += snprintf(buf + n, sizeof buf - n, " %lu:%lu", (unsigned long)cycles_to_ns(k * 8),
                              (unsigned long)r->hist[p][k]);
        buf[n] = 0;
        if (n)
            REPORT("R cpu_hist %s mhz=%lu path=%s drive_ns_from:%s\n", name, (unsigned long)bench_mhz,
                   pn[p], buf);
    }
}

static void report_bad(const char *name, const run_result_t *r)
{
    for (unsigned k = 0; k < r->nbad; k++)
        REPORT("R cpu_bad %s cycle=%lu addr=%04lx exp=%02lx got=%02x prev_exp=%02lx\n", name,
               (unsigned long)r->first_bad[k][0], (unsigned long)((r->first_bad[k][1] >> 8) & 0xFFFF),
               (unsigned long)(r->first_bad[k][1] & 0xFF), (unsigned)r->first_bad[k][2],
               (unsigned long)(bench_exp[(r->first_bad[k][0] + BLOCK - 1) % BLOCK] & 0xFF));
}

void bench_cpu(void)
{
    static bool loaded;
    int32_t offset;
    unsigned si;

    bench_pio_reset();
    dut_engine_load();
    off_stim = pio_add_program(pio, &cpu_console_program);
    off_cap = pio_add_program(pio, &capture_irq0_program);
    off_sink = pio_add_program(pio_link, &sink_program);
    {
        pio_sm_config c = sink_program_get_default_config(off_sink);
        pio_sm_init(pio_link, sm_sink, off_sink, &c);
        pio_sm_set_enabled(pio_link, sm_sink, true);
    }
    if (!loaded) {
        dma_stim = dma_claim_unused_channel(true);
        dma_cap = dma_claim_unused_channel(true);
        dma_stress = dma_claim_unused_channel(true);
        for (unsigned i = 0; i < FN_ARENA_SIZE; i++)
            arena[i] = hbyte(i ^ 0x77770000u);
        for (unsigned i = 0; i < 1024; i++)
            stress_src[i] = i * 0x9E3779B9u;
        loaded = true;
    }
    images_of = NULL;
    pins_setup();

    offset = calibrate();
    {
        cpu_timing_t t = timing_for(20);
        REPORT("R cpu_cal mhz=%lu offset_cyc=%ld m2_low_ns=%lu m2_high_ns=%lu engine=%s\n",
               (unsigned long)bench_mhz, (long)offset, (unsigned long)t.low_ns,
               (unsigned long)t.high_ns, bench_atrans ? "atrans" : "dma");
    }

    for (si = 0; si < sizeof scenarios / sizeof scenarios[0]; si++) {
        const cpu_scn_t *s = &scenarios[si];
        cpu_timing_t t = timing_for(s->lag_ns);
        uint32_t ymax = 2 * (t.lag + t.high) + 10 - 6;   /* still inside M2 high */
        uint32_t lo = 0, hi = ymax, y;
        run_result_t r;

        if (bench_atrans && s->backing != BACK_XIP)
            continue;                      /* ATRANS windows are flash only */
        /* Smallest Y with no errors: errors only fall as Y grows. */
        r = run(s, hi, probe_cycles, 0xC0FFEEu + si);
        if (r.errors) {
            REPORT("R cpu %s mhz=%lu stress=%d tmin_ns=none errors=%lu cycles=%lu late=%lu "
                   "max_drive_ns=%lu max_wapply_ns=%lu\n",
                   s->name, (unsigned long)bench_mhz, bench_stress, (unsigned long)r.errors,
                   (unsigned long)r.checked, (unsigned long)r.late,
                   (unsigned long)cycles_to_ns(r.max_drive), (unsigned long)cycles_to_ns(r.max_wapply));
            report_bad(s->name, &r);
            report_hist(s->name, &r);
            continue;
        }
        while (lo < hi) {
            uint32_t mid = (lo + hi) / 2;
            r = run(s, mid, probe_cycles, 0xBEEF00u + si * 7 + mid);
            if (r.errors == 0)
                hi = mid;
            else
                lo = mid + 1;
        }
        /* Confirm, walking up if the long run finds a straggler. */
        for (y = lo; y <= ymax; y++) {
            r = run_long(s, y, bench_confirm_cycles, 0x5EED00u + si);
            if (r.errors == 0)
                break;
        }
        REPORT("R cpu %s mhz=%lu stress=%d tmin_ns=%lu cycles=%lu errors=%lu late=%lu "
               "max_drive_ns=%lu max_wapply_ns=%lu lag_ns=%lu\n",
               s->name, (unsigned long)bench_mhz, bench_stress,
               (unsigned long)cycles_to_ns((uint32_t)((int32_t)y + offset)),
               (unsigned long)r.checked, (unsigned long)r.errors,
               (unsigned long)r.late, (unsigned long)cycles_to_ns(r.max_drive),
               (unsigned long)cycles_to_ns(r.max_wapply), (unsigned long)t.lag_ns);
        if (r.errors)
            report_bad(s->name, &r);
        report_hist(s->name, &r);
    }
    set_cooldown(0);
}
