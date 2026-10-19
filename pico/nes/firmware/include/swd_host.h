/* swd_host.h -- a small SWD host for an RP2350 target (ADIv6, RP2350
 * datasheet 3.5): enough to wake its SW-DP, reach its RP-AP and a core's
 * AHB-AP, read and write memory, and halt, set up and resume a core.
 *
 * Bit-banged. On the cart it drives RP-B's SWCLK/SWDIO from two RP-A GPIOs
 * (nes_cart.h); built with SWD_HOST_DBGFORCE (the Phase 0 bench) it drives
 * the chip's OWN debug port through SYSCFG DBGFORCE instead, which is how the
 * bench tests this code with no second chip and no wiring.
 */
#ifndef SWD_HOST_H
#define SWD_HOST_H

#include <stdbool.h>
#include <stdint.h>

#define SWD_AP_CORE0 0x02000u        /* AHB-APs in the debug address space */
#define SWD_AP_CORE1 0x04000u
#define SWD_AP_RPAP  0x80000u        /* RP-AP: CTRL at +0 (bit 31 rescue)  */

/* Pins (GPIO build). */
void swd_init(unsigned swclk, unsigned swdio);
void swd_release(void);              /* DBGFORCE build: give the pads back */

/* Dormant -> SWD, line reset, DPIDR read; sticky errors cleared. */
bool swd_connect(uint32_t *dpidr);
/* CDBGPWRUPREQ + CSYSPWRUPREQ, waiting for both ACKs. */
bool swd_power_up(void);

/* Any AP register by its full debug-space address (AP base + offset). */
bool swd_ap_write(uint32_t apaddr, uint32_t v);
bool swd_ap_read(uint32_t apaddr, uint32_t *v);

/* Memory through one core's AHB-AP: word accesses, TAR auto-increment. */
bool swd_mem_ap(uint32_t ap_base);
bool swd_mem_write(uint32_t addr, uint32_t v);
bool swd_mem_read(uint32_t addr, uint32_t *v);
bool swd_mem_write_block(uint32_t addr, const uint32_t *w, uint32_t n);
bool swd_mem_verify_block(uint32_t addr, const uint32_t *w, uint32_t n);

/* The core behind the current AHB-AP. */
bool swd_core_halt(void);
bool swd_core_set_reg(unsigned regsel, uint32_t v);   /* 13 SP, 15 PC, 16 xPSR */
bool swd_core_resume(void);

#endif /* SWD_HOST_H */
