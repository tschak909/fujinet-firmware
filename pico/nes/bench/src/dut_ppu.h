/* dut_ppu.h -- RP-B's Rev1 PPU-bus loop, as the bench runs it. */
#ifndef DUT_PPU_H
#define DUT_PPU_H

#include <stdint.h>

typedef struct {
    const uint8_t *rslot[16];   /* PA13:PA10 -> 1K the cart answers from    */
    uint8_t *wslot[16];         /* ...and takes /WR writes into (CHR-RAM)   */
    volatile uint32_t running;
} dut_ppu_t;

extern dut_ppu_t dut_ppu;

void dut_ppu_main(void);
/* The same loop, raising PPU_MARK_PIN right after it releases PD, so a PIO
 * stopwatch can time /RD rising -> PD released. */
void dut_ppu_main_marker(void);

#endif
