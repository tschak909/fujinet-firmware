/* fuji_cart.h -- the cartridge-side state (Rev1): what the engine serves,
 * what core0 drives, and the ring between them.
 *
 * core1 (the engine, nes_engine.c) records every mailbox-page write into a
 * ring and returns to the bus; core0 drains the ring into fujimail.c. Mapper
 * writes core1 applies itself (PRG) and forwards to RP-B (CHR); only MMC3's
 * interrupt registers cross to core0.
 *
 * The image is never copied: it is staged in fuji_store (RAM or flash) and
 * installed in place -- CHR sent to RP-B, the engine's banks pointed at the
 * store -- while the console waits in the loader ROM.
 */

#ifndef FUJI_CART_H
#define FUJI_CART_H

#include <stdbool.h>
#include <stdint.h>

#include "nes_cart.h"
#include "nesmap.h"
#include "fuji_store.h"

/* One transaction is a dozen register writes plus at most a 320-byte stream,
 * and the console cannot start another until it sees ACKSEQ, so the ring
 * only ever has to hold one transaction's worth. */
#define FUJI_RING_LEN 1024

typedef struct {
    volatile nes_event_t buf[FUJI_RING_LEN];
    volatile uint16_t head;                /* written by core1 only */
    volatile uint16_t tail;                /* written by core0 only */
    volatile bool overflow;
} fuji_ring_t;

extern fuji_ring_t fuji_ring;
extern uint8_t *const fuji_arena;              /* FN_ARENA_SIZE: reply, status, loader ROM */
extern uint8_t *const fuji_scratch4k;          /* 4K of core0 scratch (the boot bank's other half) */
extern uint8_t fuji_wram[FN_WRAM_SIZE];       /* the 8K window the engine serves */
extern volatile nes_serve_t fuji_serve;        /* core0's state: loading, mailbox, prg_live */
extern nesmap_t fuji_map;                      /* the live mapper, for MMC3's IRQ (core0) */
extern volatile bool fuji_boot_armed;
extern volatile bool fuji_have_staged;

/* core1: record one event. Inlined into the bus loop, so it stays a compare
 * and a few stores. */
static inline void fuji_cart_note(uint16_t offset, uint8_t data, uint8_t kind, uint32_t cycle)
{
    uint16_t head = fuji_ring.head;
    uint16_t next = (uint16_t)((head + 1u) % FUJI_RING_LEN);

    if (next == fuji_ring.tail) {
        fuji_ring.overflow = true;  /* core0 fell behind; drop, do not wrap */
        return;
    }
    fuji_ring.buf[head].offset = offset;
    fuji_ring.buf[head].data = data;
    fuji_ring.buf[head].kind = kind;
    fuji_ring.buf[head].cycle = cycle;
    fuji_ring.head = next;
}

/* core0 */
uint32_t fuji_cart_bus_cycle(void);            /* M2 rising edges so far     */
bool fuji_cart_next_event(nes_event_t *ev);
void fuji_cart_poke(unsigned offset, uint8_t value);
void fuji_cart_init(const uint8_t *loader_rom, unsigned loader_len);
void fuji_cart_set_resident(const fuji_image_t *image, const nesmap_plan_t *plan);
void fuji_cart_stage(const fuji_image_t *image, const nesmap_plan_t *plan);
bool fuji_cart_store_busy(const uint8_t *body);
void fuji_cart_request_load(void);             /* FN_HOT_SWAP, armed        */
void fuji_cart_slice_acked(void);              /* FN_REG_SLICE_ACK          */
void fuji_cart_service(void);                  /* installs, power, watchdog */
void fuji_cart_set_irq(bool asserted);         /* /IRQ: drive low or release */

#endif /* FUJI_CART_H */
