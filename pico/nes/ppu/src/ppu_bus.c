/* ppu_bus.c -- RP-B's core1: the PPU bus.
 *
 * PA0-PA13 are stable before /RD falls (PA8-13 from the PPU directly, PA0-7
 * from the console's '373 at ALE), so the loop polls for /RD low, looks the
 * byte up through the 16-slot table (ppu.h), drives it, and releases PD when
 * /RD rises. A /WR cycle stores the last sample taken while /WR was low,
 * through the write table. The Phase 0 bench measured this loop at 94 ns
 * from /RD falling to PD valid at 266 MHz, and 41 ns from /RD rising to PD
 * released.
 *
 * On RP-B the only SIO outputs in GPIO 0-31 are PD0-PD7, so one store to
 * GPIO_OUT sets the byte; a second enables it.
 */

#include "pico/stdlib.h"
#include "hardware/structs/sio.h"

#include "nes_ppu.h"
#include "ppu.h"

ppu_slots_t ppu_slots;

#pragma GCC push_options
#pragma GCC optimize("O3")

void __not_in_flash_func(ppu_core1_main)(void)
{
    ppu_slots_t *d = &ppu_slots;

    for (;;) {
        uint32_t pins = sio_hw->gpio_in;

        if (!(pins & PPU_RD_MASK)) {
            const uint8_t *q = d->rslot[(pins >> 10) & 15];

            if (q) {
                sio_hw->gpio_out = (uint32_t)q[pins & 0x3FF] << PD0_PIN;
                sio_hw->gpio_oe_set = PD_MASK;
            }
            while (!(sio_hw->gpio_in & PPU_RD_MASK))
                ;
            sio_hw->gpio_oe_clr = PD_MASK;
            continue;
        }
        if (!(pins & PPU_WR_MASK)) {
            uint32_t prev;
            uint8_t *w;

            do {
                prev = pins;
                pins = sio_hw->gpio_in;
            } while (!(pins & PPU_WR_MASK));
            w = d->wslot[(prev >> 10) & 15];
            if (w)
                w[prev & 0x3FF] = (uint8_t)(prev >> PD0_PIN);
        }
    }
}

#pragma GCC pop_options
