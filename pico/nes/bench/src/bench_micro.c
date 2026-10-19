/* bench_micro.c -- pin-free microbenchmarks: XIP read latency and the mapper
 * engine's worst case, both timed with core0's DWT cycle counter.
 *
 * The XIP numbers decide whether a 512K PRG image can be served from the
 * in-package flash. The RP2350 datasheet: "Cache misses are always issued as
 * 64-bit QSPI transfers", while "an uncached one-byte read ... will fetch
 * exactly one byte", so RP-A must read PRG through the uncached window
 * (0x14000000).
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/xip_cache.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/sio.h"
#include "hardware/regs/addressmap.h"

#include "bench.h"
#include "nesmap.h"
#include "nes_fast.h"

static bool qmi_in_continuous;   /* see qmi_continuous() */

#define UNCACHED(off) ((volatile const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + (off)))
#define CACHED(off)   ((volatile const uint8_t *)(XIP_BASE + (off)))

/* ---------------- the flash image ---------------- */

void bench_flash_image(void)
{
    uint8_t *page = bench_big;            /* free at start-up: a sector of scratch */
    uint32_t off, i, bad = 0;

    for (off = 0; off < FLASH_IMG_SIZE; off++)
        if (*CACHED(FLASH_IMG_OFF + off) != prg_byte(off))
            bad++;
    if (bad == 0) {
        REPORT("R flash image=present\n");
        return;
    }
    REPORT("R flash image=writing size=%lu bad_bytes=%lu\n", (unsigned long)FLASH_IMG_SIZE,
           (unsigned long)bad);
    for (off = 0; off < FLASH_IMG_SIZE; off += FLASH_SECTOR_SIZE) {
        uint32_t ints;

        for (i = 0; i < FLASH_SECTOR_SIZE; i++)
            page[i] = prg_byte(off + i);
        ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_IMG_OFF + off, FLASH_SECTOR_SIZE);
        flash_range_program(FLASH_IMG_OFF + off, page, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
    }
    xip_cache_invalidate_all();
    qmi_in_continuous = false;     /* the bootrom took the flash out of it */
    REPORT("R flash image=written\n");
}

/* ---------------- QMI ---------------- */

void bench_qmi_report(const char *tag)
{
    uint32_t t = qmi_hw->m[0].timing;

    REPORT("R qmi %s mhz=%lu clkdiv=%lu rxdelay=%lu cooldown=%lu sck_mhz=%lu rfmt=%08lx rcmd=%08lx\n",
           tag, (unsigned long)bench_mhz,
           (unsigned long)((t & QMI_M0_TIMING_CLKDIV_BITS) >> QMI_M0_TIMING_CLKDIV_LSB),
           (unsigned long)((t & QMI_M0_TIMING_RXDELAY_BITS) >> QMI_M0_TIMING_RXDELAY_LSB),
           (unsigned long)((t & QMI_M0_TIMING_COOLDOWN_BITS) >> QMI_M0_TIMING_COOLDOWN_LSB),
           (unsigned long)(bench_mhz / ((t & QMI_M0_TIMING_CLKDIV_BITS) >> QMI_M0_TIMING_CLKDIV_LSB)),
           (unsigned long)qmi_hw->m[0].rfmt, (unsigned long)qmi_hw->m[0].rcmd);
}

static uint32_t image_errors(void)
{
    uint32_t off, bad = 0;

    for (off = 0; off < FLASH_IMG_SIZE; off++)
        if (*UNCACHED(FLASH_IMG_OFF + off) != prg_byte(off))
            bad++;
    return bad;
}

static void set_timing(uint32_t clkdiv, uint32_t rxdelay)
{
    uint32_t t = qmi_hw->m[0].timing;

    t &= ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS | QMI_M0_TIMING_COOLDOWN_BITS);
    t |= clkdiv << QMI_M0_TIMING_CLKDIV_LSB;
    t |= rxdelay << QMI_M0_TIMING_RXDELAY_LSB;
    qmi_hw->m[0].timing = t;
}

/* The flash's continuous-read mode: once a read has sent the A0 mode bits
 * after its address, the flash expects the next read to start with the
 * address, no EBh command. That is 14 SCK a byte instead of 22. The first
 * read must carry the command (and must, before the prefix goes): drop it
 * on a flash that is not in the mode and the address goes in as a command.
 * The bootrom's flash functions leave the mode (so after an erase or
 * program it is entered again); entering it while already in it is the one
 * thing never to do: the EBh command would be clocked in as an address, the
 * mode bits would land wrong, the flash would leave the mode, and every
 * prefix-less read after that would be an address taken as a command. */
static void qmi_continuous(void)
{
    if (qmi_in_continuous)
        return;              /* entering twice would send EBh as an address */
    qmi_in_continuous = true;
    qmi_hw->m[0].rcmd = (0xEBu << QMI_M0_RCMD_PREFIX_LSB) | (0xA0u << QMI_M0_RCMD_SUFFIX_LSB);
    qmi_hw->m[0].rfmt =
        (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_S << QMI_M0_RFMT_PREFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB) |
        (QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB) |
        (QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB) |
        (QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB) |
        (QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB) |
        (QMI_M0_RFMT_SUFFIX_LEN_VALUE_8 << QMI_M0_RFMT_SUFFIX_LEN_LSB) |
        (4u << QMI_M0_RFMT_DUMMY_LEN_LSB);
    (void)*UNCACHED(0);                       /* the command, and A0: mode on */
    qmi_hw->m[0].rfmt &= ~QMI_M0_RFMT_PREFIX_LEN_BITS;
}

/* SCK = sysclk / 2 (100-133 MHz, the flash's limit is 133) and the RX delay
 * in the middle of the window that reads the whole image back clean. The
 * firmware is copy_to_ram, so nothing runs from flash while this changes. */
void bench_qmi_tune(void)
{
    uint32_t div = bench_mhz > 266 ? 3 : 2;
    uint32_t rx, first = 0xFF, last = 0xFF;

    qmi_continuous();
    for (rx = 0; rx <= 6; rx++) {
        set_timing(div, rx);
        if (image_errors() == 0) {
            if (first == 0xFF)
                first = rx;
            last = rx;
        }
    }
    if (first == 0xFF) {
        set_timing(div + 1, 2);
        REPORT("R qmi_tune mhz=%lu result=fail fallback_clkdiv=%lu\n",
               (unsigned long)bench_mhz, (unsigned long)(div + 1));
    } else {
        set_timing(div, (first + last) / 2);
        REPORT("R qmi_tune mhz=%lu clkdiv=%lu rx_ok=%lu-%lu chosen=%lu\n", (unsigned long)bench_mhz,
               (unsigned long)div, (unsigned long)first, (unsigned long)last,
               (unsigned long)((first + last) / 2));
    }
    bench_qmi_report("tuned");
}

/* ---------------- XIP latency ---------------- */

typedef struct {
    uint32_t min, max, sum, n, over[3];
} lat_t;

static const uint32_t over_ns[3] = { 150, 200, 230 };

static void lat_add(lat_t *l, uint32_t c)
{
    uint32_t ns = cycles_to_ns(c);
    unsigned i;

    if (c < l->min)
        l->min = c;
    if (c > l->max)
        l->max = c;
    l->sum += c;
    l->n++;
    for (i = 0; i < 3; i++)
        if (ns > over_ns[i])
            l->over[i]++;
}

static void lat_report(const char *name, const lat_t *l, uint32_t errors)
{
    REPORT("R xip %s mhz=%lu n=%lu min_ns=%lu avg_ns=%lu max_ns=%lu over150=%lu over200=%lu over230=%lu errors=%lu\n",
           name, (unsigned long)bench_mhz, (unsigned long)l->n, (unsigned long)cycles_to_ns(l->min),
           (unsigned long)(l->n ? cycles_to_ns(l->sum / l->n) : 0), (unsigned long)cycles_to_ns(l->max),
           (unsigned long)l->over[0], (unsigned long)l->over[1], (unsigned long)l->over[2],
           (unsigned long)errors);
}

/* One timed load. The DWT reads bracket it; baseline() is what they cost
 * around a RAM load, and is subtracted. */
static uint32_t __not_in_flash_func(timed_load)(volatile const uint8_t *p, uint8_t *v)
{
    uint32_t t0, t1;

    __asm volatile ("" ::: "memory");
    t0 = bench_cyc();
    *v = *p;
    t1 = bench_cyc();
    __asm volatile ("" ::: "memory");
    return t1 - t0;
}

static uint32_t baseline(void)
{
    static volatile uint8_t ram = 0x5A;
    uint32_t best = 0xFFFFFFFFu, i;
    uint8_t v;

    for (i = 0; i < 1000; i++) {
        uint32_t c = timed_load(&ram, &v);
        if (c < best)
            best = c;
    }
    return best;
}

static volatile uint32_t hammer_stop;
static void core1_hammer(void)
{
    uint32_t s = 0x9E3779B9u, sink = 0;

    while (!hammer_stop) {
        sink += *CACHED(rnd(&s) % (FLASH_IMG_OFF + FLASH_IMG_SIZE));   /* mostly misses */
    }
    (void)sink;
    for (;;)
        tight_loop_contents();
}

static void xip_case(const char *name, bool cached, bool seq, uint32_t cooldown, uint32_t n)
{
    uint32_t base = baseline(), s = 0x2545F491u, off = 0, errors = 0, i;
    uint32_t t = qmi_hw->m[0].timing;
    lat_t l = { 0xFFFFFFFFu, 0, 0, 0, { 0, 0, 0 } };
    uint8_t v;

    qmi_hw->m[0].timing = (t & ~QMI_M0_TIMING_COOLDOWN_BITS) | (cooldown << QMI_M0_TIMING_COOLDOWN_LSB);
    for (i = 0; i < n; i++) {
        uint32_t c;

        off = seq ? (off + 1) % FLASH_IMG_SIZE : rnd(&s) % FLASH_IMG_SIZE;
        if (cached)
            xip_cache_invalidate_all();
        c = timed_load(cached ? CACHED(FLASH_IMG_OFF + off) : UNCACHED(FLASH_IMG_OFF + off), &v);
        if (v != prg_byte(off))
            errors++;
        lat_add(&l, c > base ? c - base : 0);
    }
    qmi_hw->m[0].timing = t;
    lat_report(name, &l, errors);
}

void bench_micro_xip(void)
{
    bench_dwt_enable();
    xip_case("uncached_random_cd0", false, false, 0, 200000);
    xip_case("uncached_random_cd1", false, false, 1, 200000);
    xip_case("uncached_seq_cd0", false, true, 0, 200000);
    xip_case("uncached_seq_cd1", false, true, 1, 200000);
    xip_case("cached_miss_random", true, false, 0, 5000);

    /* What happens when another master touches XIP at the same time: the
     * reason RP-A must keep everything else off the QMI while PRG is live. */
    hammer_stop = 0;
    multicore_launch_core1(core1_hammer);
    xip_case("uncached_random_contended", false, false, 0, 100000);
    hammer_stop = 1;
    multicore_reset_core1();
}

/* ---------------- the mapper engine ---------------- */

static const uint16_t tier1[] = { 0, 1, 2, 3, 4, 7, 11, 30, 34, 66, 71, 206 };

/* One write's worth of RP-A (or RP-B) work, worst case over a fuzz, with
 * interrupts off so a USB interrupt is not counted against the engine. */
static uint32_t map_worst(uint16_t mapper, uint8_t outputs, uint32_t *avg)
{
    nesmap_t m;
    nesmap_plan_t p;
    uint32_t s = 0xDEADBEEFu + mapper, i, max = 0, sum = 0;
    static const uint8_t *bank_ptr[64];
    const uint8_t *slot[4] = { bench_big, bench_big, bench_big, bench_big };

    memset(&p, 0, sizeof p);
    p.mapper = mapper;
    p.prg_size = 512u * 1024;
    p.prg_ram_size = 8192;
    p.mirror = NESMAP_MIR_V;
    if (mapper == 3 || mapper == 4 || mapper == 11 || mapper == 34 || mapper == 66 || mapper == 206)
        p.chr_size = 256u * 1024;
    else
        p.chr_ram_size = mapper == 30 ? 32768 : 8192;
    if (nesmap_init(&m, &p) != NESMAP_OK)
        return 0xFFFFFFFFu;
    nesmap_set_outputs(&m, outputs);
    for (i = 0; i < 64; i++)
        bank_ptr[i] = bench_big + (i & 31) * 8192u;

    for (i = 0; i < 100000; i++) {
        uint32_t r = rnd(&s), t0, c, ints;
        uint16_t a = (uint16_t)(0x8000u | (r & 0x7FFFu));
        uint8_t d = (uint8_t)(r >> 16);

        if (mapper == 34 && (r >> 31))
            a = (uint16_t)(0x7FFDu + ((r >> 24) % 3));
        ints = save_and_disable_interrupts();
        t0 = bench_cyc();
        nesmap_write(&m, a, d, i * 2);
        if (m.dirty & NESMAP_DIRTY_PRG) {
            slot[0] = bank_ptr[m.out.prg[0]];
            slot[1] = bank_ptr[m.out.prg[1]];
            slot[2] = bank_ptr[m.out.prg[2]];
            slot[3] = bank_ptr[m.out.prg[3]];
        }
        m.dirty = 0;
        c = bench_cyc() - t0;
        restore_interrupts(ints);
        if (c > max)
            max = c;
        sum += c;
    }
    __asm volatile ("" :: "r"(slot[0]), "r"(slot[3]));
    *avg = sum / 100000;
    return max;
}

/* The fast halves (nes_fast.h): RP-A's PRG write path, RP-B's CHR one. */
static uint32_t fast_worst(uint16_t mapper, bool chr, uint32_t *avg)
{
    nesmap_t m;
    nesmap_plan_t p;
    static fastprg_t fp;
    static fastchr_t fc;
    static const uint8_t *bank_ptr[64];
    uint32_t s = 0xC0DEF00Du + mapper, i, max = 0, sum = 0, cycle = 10, last = 0;

    memset(&p, 0, sizeof p);
    p.mapper = mapper;
    p.prg_size = 512u * 1024;
    p.prg_ram_size = 8192;
    p.mirror = NESMAP_MIR_V;
    if (mapper == 3 || mapper == 4 || mapper == 11 || mapper == 34 || mapper == 66 || mapper == 206)
        p.chr_size = 256u * 1024;
    else
        p.chr_ram_size = mapper == 30 ? 32768 : 8192;
    if (nesmap_init(&m, &p) != NESMAP_OK)
        return 0xFFFFFFFFu;
    for (i = 0; i < 64; i++)
        bank_ptr[i] = bench_big + (i & 31) * 8192u;
    fastprg_init(&fp, &m, bank_ptr);
    fastchr_init(&fc, &m, bench_big);

    for (i = 0; i < 100000; i++) {
        uint32_t r = rnd(&s), t0, c, ints;
        uint16_t a = (uint16_t)(0x8000u | (r & 0x7FFFu));
        uint8_t d = (uint8_t)(r >> 16);
        bool consec;

        if (mapper == 34 && (r >> 31))
            a = (uint16_t)(0x7FFDu + ((r >> 24) % 3));
        cycle += (r >> 29) % 3 + 1;
        consec = cycle == last + 1;
        last = cycle;
        ints = save_and_disable_interrupts();
        t0 = bench_cyc();
        if (chr)
            fastchr_write(&fc, a, d, consec);
        else
            fastprg_write(&fp, a, d, consec);
        c = bench_cyc() - t0;
        restore_interrupts(ints);
        if (c > max)
            max = c;
        sum += c;
    }
    *avg = sum / 100000;
    return max;
}

void bench_micro_nesmap(void)
{
    unsigned k;

    bench_dwt_enable();
    for (k = 0; k < sizeof tier1 / sizeof tier1[0]; k++) {
        uint32_t af = 0, ap = 0, ac = 0;
        uint32_t full = map_worst(tier1[k], NESMAP_OUT_ALL, &af);
        uint32_t prg = map_worst(tier1[k], NESMAP_OUT_PRG, &ap);
        uint32_t chr = map_worst(tier1[k], NESMAP_OUT_CHR, &ac);

        uint32_t fpa = 0, fca = 0;
        uint32_t fprg = fast_worst(tier1[k], false, &fpa);
        uint32_t fchr = fast_worst(tier1[k], true, &fca);

        REPORT("R map mapper=%u mhz=%lu nesmap_max_ns=%lu fastprg_max_ns=%lu fastchr_max_ns=%lu "
               "nesmap_avg_ns=%lu fastprg_avg_ns=%lu fastchr_avg_ns=%lu\n", tier1[k],
               (unsigned long)bench_mhz, (unsigned long)cycles_to_ns(full),
               (unsigned long)cycles_to_ns(fprg), (unsigned long)cycles_to_ns(fchr),
               (unsigned long)cycles_to_ns(af), (unsigned long)cycles_to_ns(fpa),
               (unsigned long)cycles_to_ns(fca));
        (void)prg; (void)chr; (void)ap; (void)ac;
    }
}

/* ---------------- what an instruction costs here ---------------- */

static volatile uint32_t ipc_out[8];
static uint32_t chain[64];

static void __not_in_flash_func(ipc_body)(volatile uint32_t *out)
{
    uint32_t t0, t1, i, v = 0, ints = save_and_disable_interrupts();
    const uint32_t *p;

    t0 = bench_cyc();
    for (i = 1000; i; i--)
        __asm volatile ("");
    t1 = bench_cyc();
    out[0] = t1 - t0;                        /* empty loop               */

    t0 = bench_cyc();
    for (i = 1000; i; i--)
        v += sio_hw->gpio_in & 1;
    t1 = bench_cyc();
    out[1] = t1 - t0;                        /* SIO read per iteration   */

    p = chain;
    t0 = bench_cyc();
    for (i = 1000; i; i--)
        p = (const uint32_t *)*p;            /* dependent RAM loads      */
    t1 = bench_cyc();
    out[2] = t1 - t0;

    t0 = bench_cyc();
    for (i = 1000; i; i--)
        __asm volatile ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
    t1 = bench_cyc();
    out[3] = t1 - t0;                        /* 8 nops per iteration     */
    out[4] = v + (uint32_t)p;
    restore_interrupts(ints);
}

static volatile bool ipc_done;
static void core1_ipc(void)
{
    ipc_body(ipc_out);
    ipc_done = true;
    for (;;)
        tight_loop_contents();
}

void bench_micro_ipc(void)
{
    static uint32_t o0[8];

    for (unsigned i = 0; i < 64; i++)
        chain[i] = (uint32_t)&chain[(i * 17 + 5) & 63];
    ipc_body(o0);
    ipc_done = false;
    multicore_reset_core1();
    multicore_launch_core1(core1_ipc);
    while (!ipc_done)
        tight_loop_contents();
    multicore_reset_core1();
    REPORT("R ipc mhz=%lu core0_loop=%lu core0_sio=%lu core0_load=%lu core0_8nop=%lu "
           "core1_loop=%lu core1_sio=%lu core1_load=%lu core1_8nop=%lu (cycles per 1000)\n",
           (unsigned long)bench_mhz, (unsigned long)o0[0], (unsigned long)o0[1], (unsigned long)o0[2],
           (unsigned long)o0[3], (unsigned long)ipc_out[0], (unsigned long)ipc_out[1],
           (unsigned long)ipc_out[2], (unsigned long)ipc_out[3]);
}

/* What QMI register traffic costs core0, for the ATRANS engine: eight of
 * each, timed with interrupts off; and whether a read through a window just
 * after its ATRANS store (no read back) can still see the old mapping. */
void bench_micro_apb(void)
{
    static volatile uint32_t sram[8];

    (void)sram;
    io_rw_32 *at = &qmi_hw->atrans[1];
    const uint32_t a0 = ((FLASH_IMG_OFF) >> 12) | (2u << QMI_ATRANS0_SIZE_LSB);
    const uint32_t a1 = ((FLASH_IMG_OFF + 8192u) >> 12) | (2u << QMI_ATRANS0_SIZE_LSB);
    volatile const uint8_t *w1 = (volatile const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + (4u << 20));
    uint32_t c[6], t, ints, i, stale = 0, n = 0;
    unsigned k;

    bench_dwt_enable();
    ints = save_and_disable_interrupts();
#define TIME8(slot, stmt) do { t = bench_cyc(); for (k = 0; k < 8; k++) { stmt; } c[slot] = bench_cyc() - t; } while (0)
    TIME8(0, *at = (k & 1) ? a1 : a0);
    TIME8(1, (*at = (k & 1) ? a1 : a0, (void)*at));
    TIME8(2, (void)*at);
    TIME8(3, pio2->irq = 1u << 7);
    TIME8(4, sram[k] = k);
    TIME8(5, (void)w1[k]);
#undef TIME8
    /* Store, then read straight through the window: old bank = stale. */
    for (i = 0; i < 20000; i++) {
        uint32_t b = i & 1, off = (i * 37u) & 0x1FFFu;

        *at = b ? a1 : a0;
        if (w1[off] != prg_byte(b * 8192u + off))
            stale++;
        n++;
    }
    /* The same, but the read after the switch is the next byte after the
     * read before it: a QMI that carries on a burst from the last address
     * would hand back the old bank's byte. */
    {
        uint32_t seq_bad = 0, cross_bad = 0;
        volatile const uint8_t *w2 = (volatile const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + (8u << 20));

        qmi_hw->atrans[2] = a0;
        for (i = 0; i < 20000; i++) {
            uint32_t b = i & 1, off = (i * 37u) & 0x1FFEu;

            *at = b ? a0 : a1;
            (void)w1[off];
            *at = b ? a1 : a0;
            if (w1[off + 1] != prg_byte(b * 8192u + off + 1))
                seq_bad++;
            /* another window's read in between, as the engine does */
            (void)w2[off];
            if (w1[off] != prg_byte(b * 8192u + off))
                cross_bad++;
        }
        qmi_hw->atrans[2] = QMI_ATRANS0_RESET + 0x800u;
        REPORT("R apb_seq seq_bad=%lu cross_bad=%lu of 20000\n", (unsigned long)seq_bad,
               (unsigned long)cross_bad);
    }
    /* Again with a DMA channel reading flash (uncached) back to back, as
     * the engine's ch D does: does QMI register access wait for it? */
    {
        int ch = dma_claim_unused_channel(true);
        dma_channel_config k = dma_channel_get_default_config(ch);
        static uint8_t sink;
        uint32_t c2[3];

        channel_config_set_transfer_data_size(&k, DMA_SIZE_8);
        channel_config_set_read_increment(&k, true);
        channel_config_set_write_increment(&k, false);
        dma_channel_configure(ch, &k, &sink, (const void *)(XIP_NOCACHE_NOALLOC_BASE + FLASH_IMG_OFF),
                              1u << 20, true);
#define TIME8(slot, stmt) do { t = bench_cyc(); for (k2 = 0; k2 < 8; k2++) { stmt; } c2[slot] = bench_cyc() - t; } while (0)
        {
            unsigned k2;
            TIME8(0, *at = (k2 & 1) ? a1 : a0);
            TIME8(1, (*at = (k2 & 1) ? a1 : a0, (void)*at));
            TIME8(2, pio2->irq = 1u << 7);
        }
#undef TIME8
        dma_channel_abort(ch);
        /* One single-byte flash read in flight, as the engine's ch D does,
         * then an ATRANS store and a PIO store: when does the PIO store
         * get out? Then the same with no read in flight. */
        {
            uint32_t busy = 0, idle = 0, j;

            for (j = 0; j < 256; j++) {
                uint32_t t0, t1;

                dma_channel_configure(ch, &k, &sink,
                                      (const void *)(XIP_NOCACHE_NOALLOC_BASE + FLASH_IMG_OFF + j * 97u), 1, true);
                t0 = bench_cyc();
                *at = (j & 1) ? a1 : a0;
                pio2->irq = 1u << 7;
                t1 = bench_cyc();
                if (t1 - t0 > busy)
                    busy = t1 - t0;
                dma_channel_wait_for_finish_blocking(ch);
                t0 = bench_cyc();
                *at = (j & 1) ? a1 : a0;
                pio2->irq = 1u << 7;
                t1 = bench_cyc();
                if (t1 - t0 > idle)
                    idle = t1 - t0;
            }
            REPORT("R apb_inflight st_then_pio_max busy=%lu idle=%lu (cycles)\n", (unsigned long)busy,
                   (unsigned long)idle);
        }
        dma_channel_unclaim(ch);
        REPORT("R apb_busy atrans_st=%lu atrans_st_rb=%lu pio_st=%lu (cycles per 8, flash DMA running)\n",
               (unsigned long)c2[0], (unsigned long)c2[1], (unsigned long)c2[2]);
    }
    qmi_hw->atrans[1] = QMI_ATRANS0_RESET + 0x400u;
    restore_interrupts(ints);
    REPORT("R apb mhz=%lu atrans_st=%lu atrans_st_rb=%lu atrans_rd=%lu pio_st=%lu sram_st=%lu xip_rd=%lu "
           "(cycles per 8) stale=%lu/%lu\n",
           (unsigned long)bench_mhz, (unsigned long)c[0], (unsigned long)c[1], (unsigned long)c[2],
           (unsigned long)c[3], (unsigned long)c[4], (unsigned long)c[5], (unsigned long)stale,
           (unsigned long)n);
}
