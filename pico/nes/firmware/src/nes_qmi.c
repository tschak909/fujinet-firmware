/* nes_qmi.c -- RP-A's flash interface. See nes_qmi.h. Everything here runs
 * from SRAM (copy_to_ram), so nothing executes from flash while the QMI's
 * timing changes under it.
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/structs/qmi.h"
#include "hardware/xip_cache.h"

#include "nes_qmi.h"

#define UNCACHED(off) ((volatile const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + (off)))

/* The firmware's own text: in flash at __ram_text_source__, and the same
 * bytes in SRAM -- a known pattern to read back. */
extern char __ram_text_start__, __ram_text_end__, __ram_text_source__;
#define CHECK_LEN 16384u

static uint32_t chosen_div, chosen_rx;

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
 * address, no EBh command. The first read must carry the command; entering
 * the mode while already in it is the one thing never to do (EBh would be
 * clocked in as an address and every prefix-less read after it would be an
 * address taken as a command). Call only with the flash out of the mode:
 * at boot, and after a bootrom flash function. */
static void continuous_mode(void)
{
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

static bool reads_clean(void)
{
    uint32_t src = (uint32_t)&__ram_text_source__ - XIP_BASE;
    uint32_t len = (uint32_t)(&__ram_text_end__ - &__ram_text_start__);
    const uint8_t *ram = (const uint8_t *)&__ram_text_start__;
    uint32_t i;

    if (len > CHECK_LEN)
        len = CHECK_LEN;
    for (i = 0; i < len; i++)
        if (*UNCACHED(src + i) != ram[i])
            return false;
    return true;
}

bool nes_qmi_init(void)
{
    uint32_t div = clock_get_hz(clk_sys) > 266000000u ? 3 : 2;
    uint32_t rx, first = 0xFF, last = 0xFF;

    continuous_mode();
    for (rx = 0; rx <= 6; rx++) {
        set_timing(div, rx);
        if (reads_clean()) {
            if (first == 0xFF)
                first = rx;
            last = rx;
        }
    }
    if (first == 0xFF) {
        chosen_div = div + 1;
        chosen_rx = 2;
    } else {
        chosen_div = div;
        chosen_rx = (first + last) / 2;
    }
    set_timing(chosen_div, chosen_rx);
    xip_cache_invalidate_all();
    return first != 0xFF;
}

void nes_qmi_setup(void)
{
    set_timing(chosen_div, chosen_rx);
    continuous_mode();
}
