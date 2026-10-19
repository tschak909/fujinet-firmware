/* dut_ppu.c -- RP-B's Rev1 core1 loop: the PPU bus.
 *
 * PA0-PA13 are stable before /RD falls (PA8-13 from the PPU directly, PA0-7
 * from the console's '373 at ALE), so the loop polls for /RD low, looks the
 * byte up through a 16-slot table indexed by PA10-PA13 (8 CHR slots, then 8
 * nametable slots that are NULL unless the image is four-screen), drives it,
 * and releases PD when /RD rises. A /WR cycle stores the last sample taken
 * while /WR was low, through the write table (NULL where CHR is ROM).
 *
 * On RP-B the only SIO outputs in GPIO 0-31 are PD0-PD7, so one store to
 * GPIO_OUT sets the byte; a second enables it.
 */

#include "pico/stdlib.h"
#include "hardware/structs/sio.h"

#include "bench.h"
#include "dut_ppu.h"

dut_ppu_t dut_ppu;

#pragma GCC push_options
#pragma GCC optimize("O3")

static inline __attribute__((always_inline)) void ppu_loop(dut_ppu_t *d, bool marker)
{
    for (;;) {
        uint32_t pins = sio_hw->gpio_in;

        if (!(pins & RD_MASK)) {
            const uint8_t *q = d->rslot[(pins >> 10) & 15];

            if (q) {
                sio_hw->gpio_out = (uint32_t)q[pins & 0x3FF] << PD0_PIN;
                sio_hw->gpio_oe_set = PD_MASK;
            }
            while (!(sio_hw->gpio_in & RD_MASK))
                ;
            sio_hw->gpio_oe_clr = PD_MASK;
            if (marker)
                sio_hw->gpio_set = 1u << PPU_MARK_PIN;
            continue;
        }
        if (!(pins & WR_MASK)) {
            uint32_t prev;
            uint8_t *w;

            do {
                prev = pins;
                pins = sio_hw->gpio_in;
            } while (!(pins & WR_MASK));
            w = d->wslot[(prev >> 10) & 15];
            if (w)
                w[prev & 0x3FF] = (uint8_t)(prev >> PD0_PIN);
        }
    }
}

void __not_in_flash_func(dut_ppu_main)(void)
{
    dut_ppu.running = 1;
    ppu_loop(&dut_ppu, false);
}

void __not_in_flash_func(dut_ppu_main_marker)(void)
{
    dut_ppu.running = 1;
    ppu_loop(&dut_ppu, true);
}

#pragma GCC pop_options
