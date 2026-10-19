/* fuji_store.c -- the image store. See fuji_store.h.
 *
 * Flash layout, from FUJI_STORE_FLASH_OFF: sector 0 the header (16 bytes,
 * the rest 0xFF), then the body from sector 1, so every 8K PRG bank starts on
 * a 4K sector boundary.
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "fuji_store.h"
#include "fuji_cart.h"
#include "fuji_mailbox.h"
#include "nes_qmi.h"
#include "nesmap.h"

extern char __flash_binary_end;

#ifndef FUJI_STORE_BINARY_END
#define FUJI_STORE_BINARY_END ((uint32_t)&__flash_binary_end - XIP_BASE)
#endif

_Static_assert(FUJI_STORE_FLASH_OFF + FUJI_STORE_FLASH_SIZE
               <= PICO_FLASH_SIZE_BYTES, "flash store past the end of the part");
_Static_assert(FUJI_STORE_FLASH_SIZE - FLASH_SECTOR_SIZE >= NESMAP_IMAGE_MAX - FUJI_STORE_HDR_LEN,
               "a 512K+256K image must fit the flash store");

#define BODY_FLASH_OFF (FUJI_STORE_FLASH_OFF + FLASH_SECTOR_SIZE)
#define FLASH_BODY     ((const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + BODY_FLASH_OFF))

enum tier { TIER_NONE, TIER_RAM, TIER_FLASH };

/* 8K-aligned: the engine's DMA ORs a bank offset into a bank's base. */
static uint8_t ram_body[FUJI_STORE_RAM_SIZE] __attribute__((aligned(8192)));
static uint8_t ram_hdr[FUJI_STORE_HDR_LEN];
static uint8_t flash_hdr[FUJI_STORE_HDR_LEN];
/* A sector of scratch: the boot bank's spare half (fuji_cart.c). */
#define secbuf fuji_scratch4k

static enum tier open_tier;
static uint32_t written;        /* header + body bytes so far             */
static uint32_t sec_fill;
static uint32_t flash_off;      /* next body sector to program, body-rel  */
static uint32_t blank_off;      /* [0, blank_off) is erased, store-rel    */

uint8_t fuji_store_open(uint32_t size)
{
    uint32_t body = size > FUJI_STORE_HDR_LEN ? size - FUJI_STORE_HDR_LEN : 0;

    written = 0;
    sec_fill = 0;
    if (body <= FUJI_STORE_RAM_SIZE && !fuji_cart_store_busy(ram_body)) {
        open_tier = TIER_RAM;
        return 0;
    }
    if (body <= FUJI_STORE_FLASH_SIZE - FLASH_SECTOR_SIZE && !fuji_cart_store_busy(FLASH_BODY)) {
        if (FUJI_STORE_BINARY_END > FUJI_STORE_FLASH_OFF) {
            open_tier = TIER_NONE;
            return FN_BOOT_ERR_TOOBIG;
        }
        open_tier = TIER_FLASH;
        if (flash_off != 0)
            blank_off = 0;                     /* an abandoned session left dirt */
        flash_off = 0;
        return 0;
    }
    open_tier = TIER_NONE;
    return (body <= FUJI_STORE_FLASH_SIZE - FLASH_SECTOR_SIZE) ? FN_BOOT_ERR_STOREBUSY
                                                               : FN_BOOT_ERR_TOOBIG;
}

/* off is store-relative. */
static void flash_op(uint32_t off, const uint8_t *src)
{
    uint32_t irq = save_and_disable_interrupts();

    if (src == NULL)
        flash_range_erase(FUJI_STORE_FLASH_OFF + off, FLASH_SECTOR_SIZE);
    else
        flash_range_program(FUJI_STORE_FLASH_OFF + off, src, FLASH_SECTOR_SIZE);
    nes_qmi_setup();                          /* the bootrom left it slow */
    restore_interrupts(irq);
}

void fuji_store_idle(void)
{
    if (open_tier != TIER_NONE)
        return;
    if (blank_off >= FUJI_STORE_FLASH_SIZE)
        return;
    if (fuji_cart_store_busy(FLASH_BODY))
        return;
    flash_op(blank_off, NULL);
    blank_off += FLASH_SECTOR_SIZE;
}

/* One sector at store offset `off`: erased first unless idle got there. */
static void program_sector(uint32_t off, uint32_t len)
{
    if (len < FLASH_SECTOR_SIZE)
        memset(secbuf + len, 0xFF, FLASH_SECTOR_SIZE - len);
    if (off >= blank_off)
        flash_op(off, NULL);
    flash_op(off, secbuf);
}

static void flush_sector(uint32_t len)
{
    if (FLASH_SECTOR_SIZE + flash_off + FLASH_SECTOR_SIZE > FUJI_STORE_FLASH_SIZE)
        return;                                /* peer lied about the size */
    program_sector(FLASH_SECTOR_SIZE + flash_off, len);
    flash_off += FLASH_SECTOR_SIZE;
}

static void body_write(const uint8_t *chunk, unsigned len)
{
    uint32_t at = written - FUJI_STORE_HDR_LEN;

    switch (open_tier) {
    case TIER_RAM:
        if (at + len <= sizeof ram_body)
            memcpy(ram_body + at, chunk, len);
        break;
    case TIER_FLASH:
        while (len > 0) {
            unsigned n = FLASH_SECTOR_SIZE - sec_fill;

            if (n > len)
                n = len;
            memcpy(secbuf + sec_fill, chunk, n);
            sec_fill += n;
            chunk += n;
            len -= n;
            if (sec_fill == FLASH_SECTOR_SIZE) {
                flush_sector(sec_fill);
                sec_fill = 0;
            }
        }
        break;
    default:
        break;
    }
}

void fuji_store_write(const uint8_t *chunk, unsigned len)
{
    if (open_tier == TIER_NONE)
        return;
    while (len > 0 && written < FUJI_STORE_HDR_LEN) {
        (open_tier == TIER_RAM ? ram_hdr : flash_hdr)[written++] = *chunk++;
        len--;
    }
    if (len > 0) {
        body_write(chunk, len);
        written += len;
    }
}

bool fuji_store_close(bool aborted, fuji_image_t *img)
{
    enum tier t = open_tier;

    open_tier = TIER_NONE;
    if (t == TIER_FLASH) {
        if (!aborted && written > FUJI_STORE_HDR_LEN) {
            if (sec_fill > 0)
                flush_sector(sec_fill);
            /* The header sector last: a store with a header is complete. */
            memcpy(secbuf, flash_hdr, FUJI_STORE_HDR_LEN);
            program_sector(0, FUJI_STORE_HDR_LEN);
        }
        sec_fill = 0;
        blank_off = 0;
        flash_off = 0;
    }
    if (aborted || written < FUJI_STORE_HDR_LEN || t == TIER_NONE)
        return false;
    img->size = written;
    img->chr = NULL;
    if (t == TIER_RAM) {
        img->hdr = ram_hdr;
        img->body = ram_body;
        img->flash_off = 0;
    } else {
        img->hdr = flash_hdr;
        img->body = FLASH_BODY;
        img->flash_off = BODY_FLASH_OFF;
    }
    return true;
}
