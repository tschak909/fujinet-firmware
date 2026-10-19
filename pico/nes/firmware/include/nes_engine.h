/* nes_engine.h -- RP-A's CPU bus (Rev1): PIO + DMA serve PRG $8000-$FFFF,
 * core1 handles every write and the cart's own pages ($5000-$5FFF, WRAM).
 *
 * Two PRG paths, chosen per image (nes_engine.pio):
 *   RAM    nes_capture + four DMA hops through the 16-byte slot table
 *          (fp.slot_mem); banks 8K-aligned in RAM. ~140-175 ns M2 -> data
 *          at 266 MHz.
 *   flash  nes_capture_x + one hop; the slots are the QMI's ATRANS0-3
 *          (fastprg_mirror), banks 4K-aligned in flash, read through the
 *          uncached alias. ~210-215 ns. Window 0 is a slot, so nothing else
 *          may read flash while it runs (the firmware is copy_to_ram).
 *
 * A write holds the engine's next fetch (PIO IRQ 4) from M2 rising until
 * the slots are what the write makes them; core1 does only that much after
 * M2 falls, then lets the engine go, then the rest. MMC1 and MMC3 work out
 * their candidates while M2 is high (nes_fast.h).
 *
 * The Phase 0 bench (pico/nes/bench) runs this file on a Pico 2.
 */
#ifndef NES_ENGINE_H
#define NES_ENGINE_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "nes_bus.h"
#include "nes_fast.h"

#define NES_ENGINE_SM_CAP   0u
#define NES_ENGINE_SM_ROM   1u
#define NES_ENGINE_SM_PAGE  2u
#define NES_ENGINE_HOLD_IRQ 4u

/* What core1 hands core0: a mailbox hotspot write, or a mapper write core0
 * applies (MMC3's IRQ registers). The firmware's ring; a stub in the bench.
 * Called after the engine has been let go. */
void nes_engine_note(uint16_t offset, uint8_t data, uint8_t kind, uint32_t cycle);

typedef struct {
    fastprg_t fp;                   /* first: slot_mem is the 16-byte-aligned */
                                    /* table nes_capture indexes             */
    const uint8_t *page[2][128];    /* $0000-$7FFF by 256-byte page: the     */
                                    /* cart's own memory, or NULL; [1] with  */
                                    /* WRAM in, [0] without, so the mapper's */
                                    /* WRAM gate is one pointer for core1    */
    uint8_t *wram;                  /* the 8K window at $6000                */
    const uint8_t *arena;           /* FN_ARENA_SIZE: reply, status, loader  */
    PIO pio;                        /* GPIOBASE 0                            */
    io_wo_32 *link_txf;             /* RP-B: one frame per mapper write, or  */
                                    /* NULL                                  */
    uint32_t romsel_settle;         /* cycles, M2 seen -> /ROMSEL valid      */
    uint32_t adjacent;              /* cycles: M2 rises closer than this are */
                                    /* adjacent bus cycles                   */
    bool atrans;                    /* PRG in flash (nes_capture_x)          */
    bool mailbox;                   /* the arena's reply/status answer       */

    /* core0 -> core1: 1 = leave the mapper loop and only serve the cart's
     * own pages and mailbox writes (while the engine is set up anew). */
    volatile uint32_t park;
    volatile uint32_t parked;       /* core1: in the parked loop             */

    /* core1's counters, for diagnostics and core0's console watchdog */
    volatile uint32_t cycle;        /* M2 rising edges                       */
    volatile uint32_t writes;       /* mapper writes                         */
    volatile uint32_t late;         /* M2 already high at the top of a loop  */
    volatile uint32_t max_page;     /* cycles, M2 seen -> own page offered   */
    volatile uint32_t max_release;  /* cycles, M2 fall -> engine let go      */
    volatile uint32_t worst_write;  /* address << 8 | data of that write     */
    volatile uint32_t max_wapply;   /* cycles, M2 fall -> write path done    */
    volatile uint32_t past_fall;    /* writes whose M2-high work ran past M2 */
                                    /* falling: must stay 0                  */
    volatile uint32_t max_idle;     /* cycles, M2 fall -> spare-time work    */
                                    /* done (must fit M2 low)                */
#ifdef NES_ENGINE_DIAG
    volatile uint32_t hist[4][16];  /* page-offer time, 8-cycle buckets:     */
                                    /* path 2 on time, 3 late                */
#endif
} nes_engine_t;

/* Load the output programs into e->pio, claim four DMA channels, start the
 * cycle counter (SIO mtime at clk_sys) and the page machine, which runs from
 * then on. Once. */
void nes_engine_init(nes_engine_t *e);

/* The cart's own pages from arena / wram / mailbox (core1 picks the table by
 * fp.wram_en). Park core1 first if it is running. */
void nes_engine_pages(nes_engine_t *e);

/* The PRG side, PIO + DMA, for e->atrans (fp must be set up: for flash, its
 * bank table holds ATRANS values and start moves the slots into ATRANS0-3).
 * The page machine is not touched, so core1 serves its pages throughout. */
void nes_engine_start(nes_engine_t *e);
/* ...and back: identity ATRANS, slots back in slot_mem. */
void nes_engine_stop(nes_engine_t *e);

/* core1's entry: the loop for e->fp.mapper, forever (park to change it). */
void nes_engine_core1(nes_engine_t *e);

/* An 8K bank as its ATRANS value: flash offset `off` (4K-aligned), an 8K
 * aperture. */
static inline const uint8_t *nes_engine_atrans_bank(uint32_t off)
{
    return (const uint8_t *)(uintptr_t)((off >> 12) | (2u << 16));
}

#endif /* NES_ENGINE_H */
