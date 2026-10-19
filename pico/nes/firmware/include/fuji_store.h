/* fuji_store.h -- where a DBC-pushed image lives, and where RP-A serves its
 * PRG from (Rev1: there are no SRAMs to copy it into).
 *
 * Two tiers, chosen by size at OPEN time: a static RAM pool for images that
 * fit, and the flash above the firmware for the rest (a 512K+256K MMC3 game
 * does not fit any RAM here). The 16-byte iNES header is kept apart, so the
 * body (PRG, then CHR) starts where the engine needs it: 8K-aligned in RAM
 * (the DMA ORs the offset in), or on a 4K flash sector (the ATRANS windows
 * are placed in 4K units). A store that is staged, or live, cannot be
 * overwritten, so open() refuses with FN_BOOT_ERR_STOREBUSY when the only
 * fitting store is spoken for.
 *
 * Flash writes happen on core0 while core1 keeps serving: the firmware runs
 * from SRAM (copy_to_ram), so suspending XIP for an erase or program stalls
 * nothing -- unless the live image is the flash one, which is why that store
 * is busy while it runs. Every flash operation ends with nes_qmi_setup().
 *
 * Erases are kept OUT of the push path. A 4K sector erase is 45 ms typical and
 * 400 ms worst case on a W25Q; a program is ~11 ms. Doing both before the
 * chunk's ACK put the reply outside the ESP32's read window on the Astrocade,
 * so fuji_store_idle() blanks the store ahead of time, from the service loop.
 */

#ifndef FUJI_STORE_H
#define FUJI_STORE_H

#include <stdbool.h>
#include <stdint.h>

#define FUJI_STORE_RAM_SIZE   (352u * 1024) /* body bytes: PRG + CHR        */
#define FUJI_STORE_FLASH_OFF  0x00080000u   /* the firmware must stay below */
#define FUJI_STORE_FLASH_SIZE 0x00180000u   /* 1.5 MB of the 2 MB part      */
#define FUJI_STORE_HDR_LEN    16u

/* A stored image. */
typedef struct {
    const uint8_t *hdr;             /* the iNES header                      */
    const uint8_t *body;            /* PRG then CHR; for the flash tier the */
                                    /* uncached XIP alias of the same bytes */
    uint32_t size;                  /* header + body, as pushed             */
    uint32_t flash_off;             /* flash tier: the body's offset in     */
                                    /* flash (4K-aligned); 0 for RAM        */
    const uint8_t *chr;             /* CHR elsewhere (the resident image    */
                                    /* keeps it in flash), or NULL: after   */
                                    /* PRG in the body                      */
} fuji_image_t;

/* 0 to accept, else FN_BOOT_ERR_TOOBIG / FN_BOOT_ERR_STOREBUSY. size 0 is
 * legal (an older peer omitting it) and lands in the RAM tier. */
uint8_t fuji_store_open(uint32_t size);
void fuji_store_write(const uint8_t *chunk, unsigned len);
/* Commit (or abort). True and *img filled for a stored image. */
bool fuji_store_close(bool aborted, fuji_image_t *img);

/* Erase one sector of the flash store, or nothing if there is none to do.
 * Call only where a 45 ms stall is free: no push open, no reply owed. */
void fuji_store_idle(void);

#endif /* FUJI_STORE_H */
