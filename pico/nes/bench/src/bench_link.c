/* bench_link.c -- the RP-A -> RP-B link, transmitter and receiver on one chip.
 *
 * Integrity: core1 streams an LFSR sequence through the transmitter as fast
 * as it will go, core0 checks every frame the receiver assembles. The
 * transmitter also runs at fractional clock dividers, so its edges wander
 * against the receiver's sampling the way two crystals' would, only worse.
 *
 * Latency: core1 plays RP-A (random MMC3 writes, a few microseconds apart,
 * each forwarded as one frame), core0 plays RP-B (runs the same mapper
 * engine on the frame, rebuilds its CHR slot table, raises a marker). A PIO
 * stopwatch times LFRM rising -> marker. At the end the two mapper engines
 * must agree.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/pio_instructions.h"
#include "hardware/structs/busctrl.h"
#include "hardware/structs/sio.h"

#include "bench.h"
#include "bench.pio.h"
#include "nesmap.h"
#include "nes_fast.h"

static PIO pio = pio0, pio_w = pio1;
static uint sm_tx = 0, sm_rx = 1, sm_watch = 0;
static uint off_tx, off_rx, off_watch;

static volatile uint32_t tx_count;
static nesmap_t map_a, map_b;
static fastchr_t chr_b;

static void pins_setup(void)
{
    unsigned p;

    for (p = LD0_PIN; p <= LFRM_PIN; p++) {
        pio_gpio_init(pio, p);
        gpio_disable_pulls(p);
    }
    gpio_init(LINK_MARK_PIN);
    gpio_set_dir(LINK_MARK_PIN, true);
    gpio_put(LINK_MARK_PIN, 0);
}

static void link_init(float tx_div)
{
    pio_sm_config c;

    c = link_tx_program_get_default_config(off_tx);
    sm_config_set_out_pins(&c, LD0_PIN, 8);
    sm_config_set_sideset_pins(&c, LCLK_PIN);
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_clkdiv(&c, tx_div);
    pio_sm_init(pio, sm_tx, off_tx, &c);
    pio_sm_set_consecutive_pindirs(pio, sm_tx, LD0_PIN, 10, true);

    c = link_rx_program_get_default_config(off_rx);
    sm_config_set_in_pins(&c, LD0_PIN);
    sm_config_set_in_pin_count(&c, 8);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm_rx, off_rx, &c);

    pio_sm_set_enabled(pio, sm_rx, true);
    pio_sm_set_enabled(pio, sm_tx, true);
}

static void link_stop(void)
{
    pio_sm_set_enabled(pio, sm_tx, false);
    pio_sm_set_enabled(pio, sm_rx, false);
    pio_sm_set_enabled(pio_w, sm_watch, false);
    pio_sm_clear_fifos(pio, sm_tx);
    pio_sm_clear_fifos(pio, sm_rx);
    pio_sm_clear_fifos(pio_w, sm_watch);
}

/* ---------------- integrity ---------------- */

/* Every frame checks itself: a 16-bit sequence number and a hash of it, so a
 * dropped frame shows as a gap, not as every later frame being wrong. */
static inline uint32_t frame_for(uint32_t seq)
{
    return (seq & 0xFFFFu) << 16 | (hbyte(seq ^ 0x11110000u) << 8) | hbyte(seq ^ 0x22220000u);
}

/* frame_for's low half for every sequence number, so the checker keeps up
 * with a div-1.0 link (two hashes a frame did not, and lapped the ring). In
 * bench_big, which the link tests do not otherwise read. */
static uint16_t *const frame_lo = (uint16_t *)bench_big;

static uint32_t stream_n;
static int dma_rx = -1;
static uint32_t rx_ring[BLOCK] __attribute__((aligned(BLOCK * 4)));

static void core1_stream(void)
{
    uint32_t i;

    for (i = 0; i < stream_n; i++)
        pio_sm_put_blocking(pio, sm_tx, frame_for(i & 0xFFFFu));
    tx_count = i;
    for (;;)
        tight_loop_contents();
}

static void integrity(float div)
{
    uint32_t got = 0, errors = 0, lost = 0, seq = 0, last_wr, idle = 0;
    dma_channel_config c;

    for (uint32_t i = 0; i < 0x10000u; i++)
        frame_lo[i] = (uint16_t)frame_for(i);
    link_init(div);
    c = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 14);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_rx, false));
    dma_channel_configure(dma_rx, &c, rx_ring, &pio->rxf[sm_rx], dma_encode_endless_transfer_count(), true);
    last_wr = (uint32_t)rx_ring;

    tx_count = 0;
    multicore_launch_core1(core1_stream);
    while (seq < stream_n) {
        uint32_t wr = dma_hw->ch[dma_rx].write_addr;
        uint32_t n = ((wr - last_wr) / 4u) % BLOCK;

        if (!n) {
            if (++idle > 20000000)
                break;                               /* the link stalled */
            continue;
        }
        idle = 0;
        while (n--) {
            uint32_t f = rx_ring[((last_wr - (uint32_t)rx_ring) / 4u) % BLOCK];
            uint32_t s16 = f >> 16;

            last_wr += 4;
            if (last_wr >= (uint32_t)rx_ring + sizeof rx_ring)
                last_wr = (uint32_t)rx_ring;
            got++;
            if ((uint16_t)f != frame_lo[s16]) {
                errors++;
                seq++;
                continue;
            }
            lost += (s16 - seq) & 0xFFFFu;           /* frames skipped before this one */
            seq += ((s16 - seq) & 0xFFFFu) + 1;
        }
    }
    multicore_reset_core1();
    dma_channel_abort(dma_rx);
    link_stop();
    REPORT("R link integrity mhz=%lu tx_div=%d.%03d frames=%lu errors=%lu lost=%lu\n",
           (unsigned long)bench_mhz, (int)div, (int)((div - (int)div) * 1000 + 0.5f),
           (unsigned long)got, (unsigned long)errors, (unsigned long)(lost + (stream_n > seq ? stream_n - seq : 0)));
}

/* ---------------- latency ---------------- */

static uint32_t lat_n;
static volatile uint32_t rx_got, rpb_stop;

static void plan_mmc3(nesmap_t *m, uint8_t outputs)
{
    nesmap_plan_t p;

    memset(&p, 0, sizeof p);
    p.mapper = 4;
    p.prg_size = 512u * 1024;
    p.chr_size = 256u * 1024;
    p.prg_ram_size = 8192;
    p.mirror = NESMAP_MIR_V;
    nesmap_init(m, &p);
    nesmap_set_outputs(m, outputs);
    m->dirty = 0;
}

/* RP-B, on core1 as on the cart: no interrupts, nothing but the link. */
static void __not_in_flash_func(core1_rpb)(void)
{
    uint32_t last_cycle = 0;

    while (!rpb_stop) {
        uint32_t f, cyc;

        if (pio_sm_is_rx_fifo_empty(pio, sm_rx))
            continue;
        f = pio->rxf[sm_rx];
        cyc = (f & (1u << 27)) ? last_cycle + 1 : last_cycle + 2;
        last_cycle = cyc;
        fastchr_write(&chr_b, (uint16_t)(f >> 8), (uint8_t)f, (f >> 27) & 1);
        (void)cyc;
        sio_hw->gpio_set = 1u << LINK_MARK_PIN;
        __asm volatile ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
        sio_hw->gpio_clr = 1u << LINK_MARK_PIN;
        rx_got++;
    }
    for (;;)
        tight_loop_contents();
}

static void latency(void)
{
    pio_sm_config c;
    uint32_t rng = 0x1234567u, i, cycle = 10, last_w = 0, wmax = 0, wsum = 0, wn = 0;
    unsigned mismatch = 0;

    plan_mmc3(&map_a, NESMAP_OUT_PRG);
    plan_mmc3(&map_b, NESMAP_OUT_CHR);
    fastchr_init(&chr_b, &map_b, bench_big);
    link_init(1.0f);

    c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, off_watch, off_watch + 8);
    sm_config_set_jmp_pin(&c, LINK_MARK_PIN);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio_w, sm_watch, off_watch, &c);
    pio_sm_set_enabled(pio_w, sm_watch, true);

    rx_got = 0;
    rpb_stop = 0;
    busctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
    multicore_launch_core1(core1_rpb);
    bench_dwt_enable();

    /* RP-A: random MMC3 writes a microsecond or two apart. */
    for (i = 0; i < lat_n; i++) {
        uint32_t r = rnd(&rng), t0 = bench_cyc();
        uint32_t gap = ns_to_cycles(1500 + (r & 2047));
        uint16_t a = (uint16_t)(((r >> 12) & 1) ? 0x8001u : 0x8000u);
        uint8_t d = (uint8_t)(r >> 16);
        bool consec;

        if (((r >> 24) & 7) == 0)
            a = (uint16_t)(0xA000u | ((r >> 27) & 1));    /* mirroring / WRAM */
        while (bench_cyc() - t0 < gap)
            while (!pio_sm_is_rx_fifo_empty(pio_w, sm_watch)) {
                uint32_t k = pio_sm_get(pio_w, sm_watch);
                if (k > wmax)
                    wmax = k;
                wsum += k;
                wn++;
            }
        cycle += (r & 0x100u) ? 1 : 3;                    /* sometimes adjacent */
        consec = cycle == last_w + 1;
        last_w = cycle;
        pio_sm_put_blocking(pio, sm_tx, NES_LINK_WRITE(consec, a, d));
        nesmap_write(&map_a, a, d, cycle);
        map_a.dirty = 0;
    }
    {
        uint32_t t0 = time_us_32();
        while (rx_got != lat_n && time_us_32() - t0 < 100000)
            tight_loop_contents();
    }
    rpb_stop = 1;
    busy_wait_us(10);
    multicore_reset_core1();
    busctrl_hw->priority = 0;
    while (!pio_sm_is_rx_fifo_empty(pio_w, sm_watch)) {
        uint32_t k = pio_sm_get(pio_w, sm_watch);
        if (k > wmax)
            wmax = k;
        wsum += k;
        wn++;
    }
    link_stop();

    /* RP-A keeps the PRG half, RP-B the CHR half; both keep the gates. */

    {
        /* a full engine fed the same writes must agree with each half */
        nesmap_t full;
        uint32_t rr = 0x1234567u, cy = 10;

        plan_mmc3(&full, NESMAP_OUT_ALL);
        for (i = 0; i < lat_n; i++) {
            uint32_t r = rnd(&rr);
            uint16_t a = (uint16_t)(((r >> 12) & 1) ? 0x8001u : 0x8000u);
            if (((r >> 24) & 7) == 0)
                a = (uint16_t)(0xA000u | ((r >> 27) & 1));
            cy += (r & 0x100u) ? 1 : 3;
            nesmap_write(&full, a, (uint8_t)(r >> 16), cy);
        }
        for (i = 0; i < 4; i++)
            if (full.out.prg[i] != map_a.out.prg[i])
                mismatch++;
        for (i = 0; i < 8; i++)
            if (chr_b.slot[i] != bench_big + ((uint32_t)full.out.chr[i] << 10))
                mismatch++;
        if (chr_b.mirror != full.out.mirror)
            mismatch++;
    }

    /* The stopwatch counts 2-cycle steps from LFRM rising. */
    REPORT("R link latency mhz=%lu frames=%lu lost=%lu max_ns=%lu avg_ns=%lu samples=%lu lockstep_mismatch=%u\n",
           (unsigned long)bench_mhz, (unsigned long)rx_got, (unsigned long)(lat_n - rx_got),
           (unsigned long)cycles_to_ns(2 * wmax),
           (unsigned long)(wn ? cycles_to_ns((2 * wsum + wn / 2) / wn) : 0),
           (unsigned long)wn, mismatch);
}

static void watch_load(void)
{
    static uint16_t prog[9];
    pio_program_t q = { .instructions = prog, .length = 9, .origin = 0 };

    prog[0] = pio_encode_wait_gpio(false, LFRM_PIN);
    prog[1] = pio_encode_wait_gpio(true, LFRM_PIN);
    prog[2] = pio_encode_mov_not(pio_x, pio_null);
    prog[3] = pio_encode_jmp_pin(5);
    prog[4] = pio_encode_jmp_x_dec(3);
    prog[5] = pio_encode_mov_not(pio_isr, pio_x);
    prog[6] = pio_encode_push(false, false);
    prog[7] = pio_encode_wait_gpio(false, LINK_MARK_PIN);
    prog[8] = pio_encode_jmp(0);
    off_watch = pio_add_program(pio_w, &q);
}

void bench_link(void)
{
    bench_pio_reset();
    off_tx = pio_add_program(pio, &link_tx_program);
    off_rx = pio_add_program(pio, &link_rx_program);
    watch_load();
    pins_setup();
    if (dma_rx < 0)
        dma_rx = dma_claim_unused_channel(true);

    stream_n = bench_confirm_cycles;
    integrity(1.0f);
    integrity(1.0f + 3.0f / 256.0f);
    integrity(1.5f);

    lat_n = bench_confirm_cycles / 20;
    latency();
}
