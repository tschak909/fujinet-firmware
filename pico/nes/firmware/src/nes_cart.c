/* nes_cart.c -- core1: the CPU bus (Rev1). The loop itself is the engine
 * (nes_engine.c, shared with the Phase 0 bench); this is the cart's instance
 * of it and its one hand-off to core0.
 */

#include "pico/stdlib.h"

#include "nes_cart.h"
#include "nes_engine.h"
#include "fuji_cart.h"

/* 16-byte aligned: nes_capture indexes the slot table at the struct's start. */
nes_engine_t nes_eng __attribute__((aligned(16)));

/* A mailbox hotspot write, or one of MMC3's IRQ registers: core0's, through
 * the ring. */
void __not_in_flash_func(nes_engine_note)(uint16_t offset, uint8_t data, uint8_t kind, uint32_t cycle)
{
    fuji_cart_note(offset, data, kind, cycle);
}

void __not_in_flash_func(nes_core1_main)(void)
{
    nes_engine_core1(&nes_eng);
}
