/* bench.h -- the NES Rev1 timing bench (Phase 0).
 *
 * One stock Pico 2 plays both the console and the cart: PIO state machines
 * drive the 2A03 / 2C02 bus waveforms onto the cart's own pins and sample
 * the cart's answers, while core1 runs the Rev1 bus loop exactly as the cart
 * would. Everything loops back inside the chip's pads, so the bench needs one
 * USB cable and no wiring. Pin numbers are the Rev1 cart's: RP-A's map for
 * the CPU tests, RP-B's for the PPU tests.
 */
#ifndef BENCH_H
#define BENCH_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "nes_bus.h"       /* RP-A (CPU bus) pins: the cart's own        */
#include "nes_link.h"

#define CPU_MARK_PIN 26u   /* bench only: a spare pin core1 may toggle      */

/* ---- RP-B (PPU bus) pins, Rev1 ---- */
#define PA0_PIN      0u
#define PD0_PIN      14u
#define RD_PIN       22u
#define WR_PIN       23u
#define PWROK_PIN    24u   /* the Pico 2's VBUS sense: high on USB          */
#define CIRAM_A10_PIN 25u  /* the Pico 2's LED; CIRAM /CE on 26            */
#define PPU_MARK_PIN 27u

#define PA_MASK      0x3FFFu
#define PD_MASK      (0xFFu << PD0_PIN)
#define RD_MASK      (1u << RD_PIN)
#define WR_MASK      (1u << WR_PIN)

/* ---- the link, bench placement ---- */
#define LD0_PIN      0u
#define LCLK_PIN     8u
#define LFRM_PIN     9u
#define LINK_MARK_PIN 10u


/* ---- images: content is a function of the offset, so a checker needs no
 * copy and never touches the memory the cart serves from ---- */
static inline uint8_t hbyte(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return (uint8_t)x;
}
#define PRG_SEED 0x5A5A0000u
#define CHR_SEED 0xC4C40000u
#define NT_SEED  0x3E3E0000u
static inline uint8_t prg_byte(uint32_t off) { return hbyte(off ^ PRG_SEED); }
static inline uint8_t chr_byte(uint32_t off) { return hbyte(off ^ CHR_SEED); }

/* xorshift32 */
static inline uint32_t rnd(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *s = x;
}

/* ---- shared state ---- */
#define PRG_RAM_SIZE   (256u * 1024)
#define FLASH_IMG_OFF  (1024u * 1024)    /* the 512K XIP test image          */
#define FLASH_IMG_SIZE (512u * 1024)

extern uint8_t bench_big[PRG_RAM_SIZE];  /* PRG (CPU tests) or CHR (PPU)    */

/* One block of bus cycles, generated once per run and replayed by an endless
 * DMA ring, so core0 only has to check. A block is state-closed: it starts
 * by putting the mapper in a known state, so every pass expects the same
 * bytes. 4096 cycles = 8192 words = the largest DMA ring (32K). */
#define BLOCK        4096u
extern uint32_t bench_stim[2 * BLOCK];   /* two words per bus cycle          */
extern uint32_t bench_exp[BLOCK];        /* what each cycle must return      */
extern uint8_t bench_cap0[BLOCK];        /* capture rings, one byte a cycle  */
extern uint8_t bench_cap1[BLOCK];
extern uint32_t bench_mhz;

static inline uint32_t ns_to_cycles(uint32_t ns) { return (ns * bench_mhz + 500) / 1000; }
static inline uint32_t cycles_to_ns(uint32_t c) { return (c * 1000 + bench_mhz / 2) / bench_mhz; }

void bench_dwt_enable(void);           /* starts the cycle counter (mtime) */
/* Stop every state machine on all three PIOs and free their memory. */
void bench_pio_reset(void);
static inline uint32_t bench_cyc(void);

/* Results go out as one line each: "R <test> key=value ...". */
#define REPORT(...) printf(__VA_ARGS__)

/* The tests. Each prints its own R lines. */
void bench_flash_image(void);
void bench_qmi_report(const char *tag);
void bench_qmi_tune(void);
void bench_micro_xip(void);
void bench_micro_nesmap(void);
void bench_micro_ipc(void);
void bench_micro_apb(void);
void bench_swd(void);
void bench_cpu(void);
void bench_ppu(void);
void bench_link(void);

/* Long runs scale with this: cycles per confirmation run. */
extern uint32_t bench_confirm_cycles;
extern bool bench_stress;
extern bool bench_sync_bypass;   /* engine PIO: bypass input synchronisers on the bus pins */
extern bool bench_atrans;        /* ... with the slots as QMI ATRANS windows (XIP only) */

/* A cycle counter: the SIO's RISC-V platform timer in full-speed mode. One
 * cycle to read, where a DWT read costs about ten: on a 13-instruction bus
 * path that difference is the whole budget. */
#include "hardware/structs/sio.h"
static inline uint32_t bench_cyc(void) { return sio_hw->mtime; }

#endif
