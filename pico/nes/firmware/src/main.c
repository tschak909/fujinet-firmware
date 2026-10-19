/* main.c -- FujiNet NES cartridge firmware (Rev1), RP-A: the RP2354B on the
 * CPU bus.
 *
 * core1 and a PIO + DMA engine serve the CPU bus (nes_engine.c); RP-B, the
 * second RP2354B, serves the PPU bus and takes CHR and the mapper writes
 * over the link (nes_link_tx.c); core0's job is the USB CDC link to the
 * ESP32-S3, the mailbox service, the installs and console power.
 *
 * The RP sits on the console's 5 V bus with no buffers: RP2350 pads are
 * 5 V-tolerant while powered, and the cart is powered from the console or
 * USB, whichever is up. Pull-downs stay off on every bus pin (RP2350-E9);
 * D0-D7 and /IRQ are held off (OEOVER) until PWR_OK (fuji_cart.c).
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "tusb.h"

#include "nes_cart.h"
#include "nes_irq.h"
#include "nes_link_tx.h"
#include "nes_qmi.h"
#include "rpb_swd.h"
#include "fujiboot.h"
#include "fujinet.h"
#include "fuji_cart.h"
#include "fuji_store.h"

volatile rpb_err_t fuji_rpb_err;                /* for the diagnostics page */

int main(void)
{
    /* 266 MHz at 1.20 V: the engine's flash path needs SCK at 133 MHz
     * (clk_sys / 2, the flash's limit) and core1 every cycle of M2 low.
     * Phase 0 measured it there (pico/nes/bench). */
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_us(2000);
    set_sys_clock_khz(266000, true);
    nes_qmi_init();

    gpio_init_mask(BUS_IN_MASK);
    gpio_set_dir_in_masked(BUS_IN_MASK);
    for (unsigned pin = 0; pin < 32; pin++)
        if (BUS_IN_MASK & (1u << pin))
            gpio_disable_pulls(pin);
    for (unsigned pin = CD0_PIN; pin < CD0_PIN + 8; pin++)
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);

    /* /IRQ: released (input) until a mapper pulls it. */
    gpio_init(IRQ_PIN);
    gpio_disable_pulls(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, false);

    nes_link_tx_init();
    nes_irq_init();
    fuji_config_boot();                         /* the engine on the boot bank */
    multicore_launch_core1(nes_core1_main);
    /* RP-B after the bus is served: the console may already be fetching
     * its vectors. The first install waits for RP-B's LACK. */
    fuji_rpb_err = rpb_boot();

    tusb_init();
    for (;;) {
        tud_task();
        fuji_mailbox_service();
        fuji_cart_service();
        fuji_store_idle();
    }
    return 0;
}

/* TinyUSB device callbacks; nothing to do, the CDC link is polled. */
void tud_mount_cb(void) { }
void tud_umount_cb(void) { }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void) { }
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf; (void)dtr; (void)rts;
}
void tud_cdc_rx_cb(uint8_t itf) { (void)itf; }
