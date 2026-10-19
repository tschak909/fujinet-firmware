/* ppu_link.h -- RP-B's side of the link (nes_link.h), hardware-free: the
 * install (header, CHR, mode) and every mapper write, turned into the slot
 * table core1 serves from and the CIRAM mode. ppu_main.c gives it the
 * hardware through three hooks; host_test/test_ppu.c gives it a model.
 */
#ifndef PPU_LINK_H
#define PPU_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "nes_ppu.h"
#include "nes_fast.h"
#include "nesmap.h"
#include "ppu.h"

typedef struct {
    /* set by the owner */
    uint8_t *chr_mem;                   /* CHR ROM or RAM, 1K-aligned        */
    uint32_t chr_max;
    uint8_t *nt_ram;                    /* 4K: the four-screen nametables    */
    ppu_slots_t *slots;
    volatile rpb_status_t *status;
    void (*ciram)(unsigned mirror);     /* repatch the CIRAM table           */
    void (*gate)(bool serving);         /* PD on or off the bus              */
    uint32_t (*cycles)(void);           /* for status.max_apply, or NULL     */

    /* the link's own state */
    uint8_t hdr[18];
    nesmap_t map;
    fastchr_t fc;
    bool chr_is_ram, have_image, serving;
    uint32_t chr_pos;
} ppu_link_t;

void ppu_link_init(ppu_link_t *l);
void ppu_link_frame(ppu_link_t *l, uint32_t frame);

#endif /* PPU_LINK_H */
