/* ppu.h -- RP-B (Rev1): what core0 publishes and core1 serves from. */
#ifndef PPU_H
#define PPU_H

#include <stdint.h>

/* PA13:PA10 picks a slot: 0-7 the eight 1K CHR slots, 8-15 the nametable
 * slots (NULL unless the image is four-screen; CIRAM serves them). A NULL
 * read slot leaves PD alone; a NULL write slot ignores /WR (CHR-ROM). core0
 * changes one word at a time; core1 reads them every access. */
typedef struct {
    const uint8_t *volatile rslot[16];
    uint8_t *volatile wslot[16];
} ppu_slots_t;

extern ppu_slots_t ppu_slots;

void ppu_core1_main(void);

#endif /* PPU_H */
