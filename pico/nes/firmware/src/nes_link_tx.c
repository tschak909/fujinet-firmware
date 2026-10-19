/* nes_link_tx.c -- RP-A's end of the link. See nes_link_tx.h. */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "nes_link_tx.h"
#include "nes_link.h"
#include "nes_cart.h"
#include "nes_link.pio.h"

#define LINK_PIO pio1
#define LINK_SM  0u

/* How long RP-B may hold LACK low: at power-on it may still be loading. */
#define LACK_TIMEOUT_US 1000000u

void nes_link_tx_init(void)
{
    pio_sm_config c;
    uint off;
    unsigned p;

    pio_set_gpio_base(LINK_PIO, 16);
    off = pio_add_program(LINK_PIO, &nes_link_tx_program);
    for (p = LD0_PIN; p < LD0_PIN + 8; p++)
        pio_gpio_init(LINK_PIO, p);
    pio_gpio_init(LINK_PIO, LCLK_PIN);
    pio_gpio_init(LINK_PIO, LFRM_PIN);
    pio_sm_set_consecutive_pindirs(LINK_PIO, LINK_SM, LD0_PIN, 8, true);
    pio_sm_set_consecutive_pindirs(LINK_PIO, LINK_SM, LCLK_PIN, 2, true);

    c = nes_link_tx_program_get_default_config(off);
    sm_config_set_out_pins(&c, LD0_PIN, 8);
    sm_config_set_sideset_pins(&c, LCLK_PIN);
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(LINK_PIO, LINK_SM, off, &c);
    pio_sm_set_enabled(LINK_PIO, LINK_SM, true);

    gpio_init(LACK_PIN);
    gpio_set_dir(LACK_PIN, false);
    gpio_pull_down(LACK_PIN);                 /* RP-B not there = not ready */
}

io_wo_32 *nes_link_tx_fifo(void)
{
    return &LINK_PIO->txf[LINK_SM];
}

bool nes_link_rpb_ready(void)
{
    return gpio_get(LACK_PIN);
}

void nes_link_send(uint32_t frame)
{
    pio_sm_put_blocking(LINK_PIO, LINK_SM, frame);
}

void nes_link_flush(void)
{
    while (!pio_sm_is_tx_fifo_empty(LINK_PIO, LINK_SM))
        tight_loop_contents();
    busy_wait_us(1);                          /* the frame in the shifter */
}

static bool wait_ready(void)
{
    absolute_time_t until = make_timeout_time_us(LACK_TIMEOUT_US);

    while (!nes_link_rpb_ready())
        if (time_reached(until))
            return false;
    return true;
}

/* One install frame; every 3K of CHR, RP-B's LACK first (it drops it while
 * its ring is filling). */
static bool send_paced(void *ctx, uint32_t frame, uint32_t chr_done)
{
    (void)ctx;
    if (NES_LINK_OP(frame) == NES_LINK_OP_DATA && !(chr_done % 3072) && !wait_ready())
        return false;
    nes_link_send(frame);
    return true;
}

bool nes_link_send_image(const uint8_t header[16], const uint8_t *chr, uint32_t chr_len)
{
    if (!wait_ready())
        return false;
    return nes_link_encode_image(header, chr, chr_len, send_paced, NULL);
}
