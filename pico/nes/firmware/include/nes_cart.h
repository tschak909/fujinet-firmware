/* nes_cart.h -- the NES cartridge (Rev1) as seen from RP-A, the RP2354B on
 * the CPU bus. RP-B, the second RP2354B, owns the PPU bus (nes_ppu.h); the
 * two talk over the link (nes_link.h).
 *
 * RP-A's jobs on the CPU bus:
 *
 *   1. Serve PRG $8000-$FFFF: the PIO + DMA engine (nes_engine.h) from RAM or
 *      flash. Before a game, every slot is the boot bank, whose last page
 *      holds the cart's vectors.
 *   2. Serve $5000-$5FFF (the mailbox arena + the loader ROM) and $6000-$7FFF
 *      (WRAM) from its own memory: core1, beside the engine.
 *   3. Watch every write. Mapper writes change the engine's slots inline and
 *      go to RP-B as link frames; mailbox hotspot writes go to core0 through
 *      a ring.
 *
 * The decode lives here as static inline so the MAME device and the host
 * tests share one model of it (the ColecoVision pattern); core1's own copy is
 * nes_engine.c.
 *
 * PROVISIONAL until a scope says otherwise: how long write data holds after
 * M2 falls, hence the last-sample-while-high model.
 */
#ifndef NES_CART_H
#define NES_CART_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "fuji_mailbox.h"
#include "nes_bus.h"

/* --------------------------------------------------------------------------
 * GPIO map, RP-A (U1, RP2354B), 44 of 48. The CPU bus, GP0-GP25 (nes_bus.h),
 * is SIO's and PIO0's (GPIOBASE 0); everything PIO1 and PIO2 touch is in
 * 16-47 (GPIOBASE 16). The RP sits on the 5 V bus directly: its pads are
 * 5 V-tolerant while powered, and OEOVER keeps D0-D7 off until PWR_OK.
 * -------------------------------------------------------------------------- */
#define PWROK_PIN     26u   /* console 5 V present: 2.2k/3.3k of CONS_5V     */
#define LED_PIN       27u   /* low-Vf LED to GND through 330R                */
                            /* GP28-GP31: spare, test pads                   */
#define IRQ_PIN       32u   /* /IRQ: driven low or released, never high      */
#define LACK_PIN      33u   /* RP-B -> RP-A: link acknowledge / ready        */
#define LD0_PIN       34u   /* link LD0-LD7 on GP34-GP41 (PIO1 out)          */
#define LCLK_PIN      42u   /* link clock, both edges (PIO1 side-set bit 0)  */
#define LFRM_PIN      43u   /* link frame (side-set bit 1)                   */
#define RPB_SWCLK_PIN 44u   /* RP-B's SWD, driven by RP-A (PIO2)             */
#define RPB_SWDIO_PIN 45u
#define RPB_RUN_PIN   46u   /* RP-B's RUN: low holds it in reset             */
#define PA12_PIN      47u   /* PPU A12, for the MMC3 scanline filter (PIO1)  */

#define BUS_IN_MASK  (CA_LO_MASK | DATA_MASK | M2_MASK | RW_MASK | ROMSEL_MASK \
                      | CA13_MASK | CA14_MASK)

/* --------------------------------------------------------------------------
 * The decode.
 * -------------------------------------------------------------------------- */
typedef enum {
    NES_R_NONE = 0,   /* console RAM, PPU, APU: not ours                    */
    NES_R_EXP,        /* $4000-$4FFF: a few boards keep registers here      */
    NES_R_MCU,        /* $5000-$5FFF: mailbox arena + loader ROM            */
    NES_R_WRAM,       /* $6000-$7FFF                                        */
    NES_R_ROM,        /* $8000-$FFFF                                        */
} nes_region_t;

/* A15 is not on the edge; while M2 is high, /ROMSEL low means A15 = 1. */
static inline uint16_t nes_addr_from_pins(uint32_t pins)
{
    uint16_t a = (uint16_t)((pins & CA_LO_MASK) | (((pins >> CA13_PIN) & 3u) << 13));

    if (!(pins & ROMSEL_MASK))
        a |= 0x8000u;
    return a;
}

static inline uint8_t nes_data_from_pins(uint32_t pins)
{
    return (uint8_t)((pins >> CD0_PIN) & 0xFFu);
}

/* Only meaningful for a sample taken while M2 is high: /ROMSEL carries A15
 * and the low decode assumes the address is stable. */
static inline nes_region_t nes_region_from_pins(uint32_t pins)
{
    if (!(pins & ROMSEL_MASK))
        return NES_R_ROM;
    if (!(pins & CA14_MASK))
        return NES_R_NONE;
    if (pins & CA13_MASK)
        return NES_R_WRAM;
    if (pins & (1u << 12))
        return NES_R_MCU;
    return NES_R_EXP;
}

/* The same decode from a plain address, for the MAME device and the tests. */
static inline nes_region_t nes_region_from_addr(uint16_t a)
{
    if (a >= 0x8000)
        return NES_R_ROM;
    if (a >= 0x6000)
        return NES_R_WRAM;
    if (a >= 0x5000)
        return NES_R_MCU;
    if (a >= 0x4000)
        return NES_R_EXP;
    return NES_R_NONE;
}

/* What the cart answers with, as the MAME device and the host tests model it
 * (the cart itself: nes_engine.h's page tables and slots). */
typedef struct {
    const uint8_t *arena;     /* FN_ARENA_SIZE: reply, status, (hotspots), loader */
    uint8_t *wram;            /* NESMAP_WRAM_MAX, or NULL                   */
    uint32_t wram_size;
    const uint8_t *vectors;   /* 256 bytes served at $FF00 while !prg_live  */
    bool prg_live;            /* the game's PRG answers $8000+, not the boot bank */
    bool mailbox;             /* the live image keeps the mailbox           */
    bool wram_en, wram_wp;    /* from the mapper                            */
    bool loading;             /* an install is under way: no mapper decode  */
} nes_serve_t;

/* The byte the cart drives for a read, or NULL to leave the bus alone. */
static inline const uint8_t *nes_serve_ptr(const volatile nes_serve_t *s, nes_region_t r, uint16_t a)
{
    switch (r) {
    case NES_R_MCU: {
        unsigned off = a & (FN_ARENA_SIZE - 1);
        if (off >= FN_LOADER)
            return s->arena + off;                 /* the loader, always  */
        if (off < FN_R_PAINT_END && s->mailbox)
            return s->arena + off;                 /* reply + status      */
        return NULL;                               /* hotspots: inert     */
    }
    case NES_R_WRAM:
        if (s->wram && s->wram_en) {
            unsigned off = a & (FN_WRAM_SIZE - 1);
            if (off < s->wram_size)
                return s->wram + off;
        }
        return NULL;
    case NES_R_ROM:
        if (!s->prg_live && a >= FN_VECTOR_BASE)
            return s->vectors + (a & 0xFF);
        return NULL;
    default:
        return NULL;
    }
}

/* What a console write meant. */
typedef enum {
    NES_W_NONE = 0,
    NES_W_MAILBOX,    /* a hotspot page: offset for fujimail, via the ring  */
    NES_W_WRAM,       /* stored by core1 itself                             */
    NES_W_MAPPER,     /* nesmap_write, inline                               */
} nes_write_t;

static inline nes_write_t nes_write_kind(const volatile nes_serve_t *s, nes_region_t r, uint16_t a)
{
    switch (r) {
    case NES_R_MCU:
        if (!s->mailbox)
            return s->loading ? NES_W_NONE : NES_W_MAPPER;   /* MMC5-class boards */
        if ((a & (FN_ARENA_SIZE - 1)) >= FN_H_REGSEL && (a & (FN_ARENA_SIZE - 1)) < FN_LOADER)
            return NES_W_MAILBOX;
        return NES_W_NONE;
    case NES_R_WRAM:
        /* NINA-001 keeps registers at $7FFD-$7FFF; the mapper sees every
         * WRAM write and decides. The store happens regardless. */
        return NES_W_WRAM;
    case NES_R_ROM:
    case NES_R_EXP:
        return s->loading ? NES_W_NONE : NES_W_MAPPER;
    default:
        return NES_W_NONE;
    }
}

/* core1 -> core0 ring entry: a raw hotspot write, tagged with the bus cycle
 * so the drain can spot an RMW's dummy write (same address, adjacent cycle). */
typedef struct {
    uint16_t offset;          /* arena offset (mailbox) or CPU address (mapper) */
    uint8_t  data;
    uint8_t  kind;            /* NES_EV_MAILBOX / NES_EV_MAPPER             */
    uint32_t cycle;
} nes_event_t;

#define NES_EV_MAILBOX 0      /* a hotspot write, for fujimail              */
#define NES_EV_MAPPER  1      /* a mapper write core1 forwards to core0 (MMC3 IRQ regs) */

/* core1's entry point; SRAM-resident, see nes_cart.c. */
void nes_core1_main(void);

#endif /* NES_CART_H */
