/* ppu_main.c -- RP-B, the FujiNet NES cart's PPU-bus RP2354B (Rev1).
 *
 * core1 serves the PPU bus from the slot table (ppu_bus.c); PIO0 drives the
 * CIRAM lines from a 16-entry table indexed by PA13:PA10; core0 runs the link
 * from RP-A (nes_link.h): it takes an image's header and CHR at install,
 * then every mapper write the console makes, which it runs through the CHR
 * half of the same mapper engine RP-A runs (nes_fast.h) and publishes as new
 * slots and, for a mirroring change, a new CIRAM table.
 *
 * RP-A loads this image into SRAM over SWD at every boot and starts it; it
 * never runs from flash (binary type no_flash).
 */

#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "hardware/structs/sio.h"

#include "nes_ppu.h"
#include "nes_link.h"
#include "nes_fast.h"
#include "nesmap.h"
#include "ppu.h"
#include "ppu_link.h"
#include "ppu_ciram.pio.h"
#include "nes_link.pio.h"

#define CIRAM_PIO pio0
#define CIRAM_SM  0u
#define LINK_PIO  pio1
#define LINK_SM   0u

#define CHR_MAX   (256u * 1024)                /* MMC3's ceiling; Tier 1 needs no more */

/* RP-A reads this over SWD (its address is in rpb_image.h). */
volatile rpb_status_t rpb_status;

static uint8_t chr_mem[CHR_MAX] __attribute__((aligned(1024)));
static uint8_t nt_ram[4096];                   /* four-screen nametables */
static uint32_t rx_ring[4096] __attribute__((aligned(16384)));
static int rx_dma = -1;

static ppu_link_t link;
static bool pwrok, serving;

/* ---------------- the CIRAM table ---------------- */

static uint8_t ciram_side(unsigned mir, unsigned slot)
{
    unsigned pa13 = slot >> 3, pa11 = (slot >> 1) & 1, pa10 = slot & 1;
    unsigned ce_n = 1, a10 = 0;

    if (pa13 && mir != NESMAP_MIR_4) {
        ce_n = 0;
        a10 = mir == NESMAP_MIR_V ? pa10 : mir == NESMAP_MIR_H ? pa11 : mir == NESMAP_MIR_1HI ? 1 : 0;
    }
    return (uint8_t)((ce_n << 1) | a10);
}

static void ciram_patch(unsigned mir)
{
    unsigned i;

    for (i = 0; i < 16; i++)
        CIRAM_PIO->instr_mem[i] = pio_encode_mov(pio_pc, pio_pins) | pio_encode_sideset(2, ciram_side(mir, i));
}

static void ciram_init(void)
{
    pio_sm_config c;

    pio_add_program_at_offset(CIRAM_PIO, &ppu_ciram_program, 0);
    ciram_patch(NESMAP_MIR_V);
    c = ppu_ciram_program_get_default_config(0);
    sm_config_set_in_pins(&c, PA0_PIN + 10);
    sm_config_set_in_pin_count(&c, 4);
    sm_config_set_sideset_pins(&c, CIRAM_A10_PIN);
    pio_gpio_init(CIRAM_PIO, CIRAM_A10_PIN);
    pio_gpio_init(CIRAM_PIO, CIRAM_CE_PIN);
    pio_sm_init(CIRAM_PIO, CIRAM_SM, 0, &c);
    pio_sm_set_consecutive_pindirs(CIRAM_PIO, CIRAM_SM, CIRAM_A10_PIN, 2, true);
    pio_sm_set_enabled(CIRAM_PIO, CIRAM_SM, true);
}

/* ---------------- the bus gate ---------------- */

/* PD0-PD7 drive only while the console is powered and RP-A says serve; the
 * CIRAM lines whenever the console is powered (the console has no default
 * for them). Never into an unpowered console. */
static void bus_gate(void)
{
    uint32_t pd = (pwrok && serving) ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_LOW;
    uint32_t ci = pwrok ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_LOW;
    unsigned p;

    for (p = PD0_PIN; p < PD0_PIN + 8; p++)
        gpio_set_oeover(p, pd);
    gpio_set_oeover(CIRAM_A10_PIN, ci);
    gpio_set_oeover(CIRAM_CE_PIN, ci);
}

/* ---------------- the link ---------------- */

static void link_init(void)
{
    pio_sm_config c;
    dma_channel_config k;
    uint off;
    unsigned p;

    pio_set_gpio_base(LINK_PIO, 16);
    off = pio_add_program(LINK_PIO, &nes_link_rx_program);
    for (p = RPB_LFRM_PIN; p < RPB_LD7_PIN + 8; p++) {
        gpio_init(p);
        gpio_set_dir(p, false);
        gpio_disable_pulls(p);
    }
    c = nes_link_rx_program_get_default_config(off);
    sm_config_set_in_pins(&c, RPB_LFRM_PIN);
    sm_config_set_in_pin_count(&c, 10);
    sm_config_set_in_shift(&c, true, true, 32);    /* right, autopush */
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(LINK_PIO, LINK_SM, off, &c);

    rx_dma = dma_claim_unused_channel(true);
    k = dma_channel_get_default_config(rx_dma);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, true);
    channel_config_set_ring(&k, true, 14);         /* 4096 words */
    channel_config_set_dreq(&k, pio_get_dreq(LINK_PIO, LINK_SM, false));
    dma_channel_configure(rx_dma, &k, rx_ring, &LINK_PIO->rxf[LINK_SM],
                          dma_encode_endless_transfer_count(), true);
    pio_sm_set_enabled(LINK_PIO, LINK_SM, true);

    gpio_init(RPB_LACK_PIN);
    gpio_set_dir(RPB_LACK_PIN, true);
    gpio_put(RPB_LACK_PIN, 0);
}

static void gate(bool on)
{
    serving = on;
    bus_gate();
}

static uint32_t cycles(void)
{
    return sio_hw->mtime;
}

/* ---------------- main ---------------- */

int main(void)
{
    uint32_t last_wr;
    unsigned p;

    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_us(2000);
    set_sys_clock_khz(266000, true);
    sio_hw->mtime_ctrl = SIO_MTIME_CTRL_EN_BITS | SIO_MTIME_CTRL_FULLSPEED_BITS;

    rpb_status.version = RPB_VERSION;
    for (p = PA0_PIN; p < PA0_PIN + 14; p++) {
        gpio_init(p);
        gpio_set_dir(p, false);
        gpio_disable_pulls(p);                  /* RP2350-E9: no pull-downs on the bus */
    }
    for (p = PD0_PIN; p < PD0_PIN + 8; p++) {
        gpio_init(p);
        gpio_put(p, 0);
        gpio_set_dir(p, false);
        gpio_disable_pulls(p);
        gpio_set_drive_strength(p, GPIO_DRIVE_STRENGTH_4MA);
    }
    for (p = PPU_RD_PIN; p <= PPU_PWROK_PIN; p++) {
        gpio_init(p);
        gpio_set_dir(p, false);
        gpio_disable_pulls(p);
    }
    pwrok = false;
    serving = false;
    bus_gate();
    link.chr_mem = chr_mem;
    link.chr_max = CHR_MAX;
    link.nt_ram = nt_ram;
    link.slots = &ppu_slots;
    link.status = &rpb_status;
    link.ciram = ciram_patch;
    link.gate = gate;
    link.cycles = cycles;
    ppu_link_init(&link);
    ciram_init();
    link_init();
    multicore_launch_core1(ppu_core1_main);

    last_wr = (uint32_t)rx_ring;
    rpb_status.magic = RPB_STATUS_MAGIC;
    gpio_put(RPB_LACK_PIN, 1);                  /* ready for frames */

    for (;;) {
        uint32_t wr = dma_hw->ch[rx_dma].write_addr;
        bool p_now = gpio_get(PPU_PWROK_PIN);

        /* LACK low while the ring is more than half full: RP-A checks it
         * every thousand CHR frames. */
        gpio_put(RPB_LACK_PIN, ((wr - last_wr) & (sizeof rx_ring - 1)) < sizeof rx_ring / 2);
        while (last_wr != wr) {
            ppu_link_frame(&link, nes_link_unmirror(*(volatile uint32_t *)last_wr));
            last_wr += 4;
            if (last_wr >= (uint32_t)rx_ring + sizeof rx_ring)
                last_wr = (uint32_t)rx_ring;
        }
        if (p_now != pwrok) {
            pwrok = p_now;
            rpb_status.pwrok = pwrok;
            bus_gate();
        }
    }
}
