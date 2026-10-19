/* rpb_swd.c -- RP-A loads and starts RP-B over SWD. See rpb_swd.h.
 *
 * SWD on GP44 (SWCLK) and GP45 (SWDIO) through swd_host.c, with GP46 on
 * RP-B's RUN. A rescue reset through the RP-AP (CTRL bit 31, set then
 * cleared) restarts RP-B with a flag the bootrom checks before anything
 * else: it halts core0 with the clocks running (datasheet 3.5.8), which is
 * the clean state to load into.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "rpb_swd.h"
#include "swd_host.h"
#include "nes_cart.h"
#include "nes_ppu.h"
#include "rpb_image.h"

#define VTOR        0xE000ED08u
#define RPAP_RESCUE (1u << 31)

bool rpb_read32(uint32_t addr, uint32_t *value)
{
    return swd_mem_read(addr, value);
}

rpb_err_t rpb_boot(void)
{
    uint32_t v;
    int n;

    swd_init(RPB_SWCLK_PIN, RPB_SWDIO_PIN);
    gpio_init(RPB_RUN_PIN);
    gpio_set_dir(RPB_RUN_PIN, true);

    /* A clean start, then the rescue reset. */
    gpio_put(RPB_RUN_PIN, 0);
    sleep_ms(2);
    gpio_put(RPB_RUN_PIN, 1);
    sleep_ms(10);
    if (!swd_connect(NULL))
        return RPB_ERR_NO_DP;
    if (!swd_ap_write(SWD_AP_RPAP, RPAP_RESCUE) || !swd_ap_write(SWD_AP_RPAP, 0))
        return RPB_ERR_NO_DP;
    sleep_ms(10);
    if (!swd_connect(NULL))
        return RPB_ERR_NO_DP;

    if (!swd_power_up() || !swd_mem_ap(SWD_AP_CORE0))
        return RPB_ERR_POWER;
    if (!swd_core_halt())
        return RPB_ERR_HALT;
    if (!swd_mem_write_block(RPB_IMAGE_LOAD, _rpb_image, RPB_IMAGE_WORDS) ||
        !swd_mem_verify_block(RPB_IMAGE_LOAD, _rpb_image, RPB_IMAGE_WORDS))
        return RPB_ERR_VERIFY;

    /* Run it: its vectors, stack and entry, Thumb state. */
    if (!swd_mem_write(VTOR, RPB_IMAGE_LOAD) ||
        !swd_core_set_reg(13, RPB_IMAGE_SP) ||
        !swd_core_set_reg(15, RPB_IMAGE_ENTRY & ~1u) ||
        !swd_core_set_reg(16, 1u << 24) ||
        !swd_core_resume())
        return RPB_ERR_START;

    /* It reports in through its status block (and raises LACK). */
    for (n = 0; n < 200; n++) {
        sleep_ms(1);
        if (rpb_read32(RPB_STATUS_ADDR, &v) && v == RPB_STATUS_MAGIC)
            return RPB_OK;
    }
    return RPB_ERR_START;
}
