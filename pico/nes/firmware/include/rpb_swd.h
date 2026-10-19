/* rpb_swd.h -- RP-A loads and starts RP-B over SWD (Rev1).
 *
 * RP-B runs entirely from SRAM, from an image RP-A carries in its own flash
 * (rpb_image.h, packed by tools/mkrpbh.py from pico/nes/ppu). At every boot
 * RP-A wakes RP-B's SW-DP, rescue-resets it (the bootrom then halts before
 * running anything), writes the image through core0's AHB-AP, reads it back,
 * points SP and PC at it and lets it run. RP2350 datasheet 3.5.
 */
#ifndef RPB_SWD_H
#define RPB_SWD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RPB_OK = 0,
    RPB_ERR_NO_DP,          /* no SWD answer: RP-B unpowered, or not fitted  */
    RPB_ERR_POWER,          /* the debug domain did not power up             */
    RPB_ERR_HALT,           /* core0 would not halt                          */
    RPB_ERR_VERIFY,         /* the image read back wrong                     */
    RPB_ERR_START,          /* it ran, but never reported in                 */
} rpb_err_t;

/* Load and start RP-B. Takes ~0.2 s. */
rpb_err_t rpb_boot(void);

/* One word of RP-B's memory, e.g. its status block (nes_ppu.h). */
bool rpb_read32(uint32_t addr, uint32_t *value);

#endif /* RPB_SWD_H */
