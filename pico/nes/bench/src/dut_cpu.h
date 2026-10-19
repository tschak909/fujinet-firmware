/* dut_cpu.h -- RP-A's CPU bus as the bench runs it: the firmware's own engine
 * (firmware/src/nes_engine.c) on PIO2, with the bench's images and oracle.
 */
#ifndef DUT_CPU_H
#define DUT_CPU_H

#include <stdbool.h>
#include <stdint.h>

#include "nesmap.h"
#include "nes_engine.h"

typedef struct {
    nes_engine_t eng;               /* first: its slot table is 16-aligned   */
    const uint8_t *bank_ptr[64];    /* where each 8K PRG bank lives          */
    nesmap_t map;
    volatile uint32_t running;      /* core1 is in the loop                  */
} dut_cpu_t;

extern dut_cpu_t dut_cpu;

/* Load once per bench_cpu() after the PIO reset; start/stop per run;
 * dut_engine_main is core1's entry. */
void dut_engine_load(void);
void dut_engine_start(void);
void dut_engine_stop(void);
void dut_engine_main(void);

#endif /* DUT_CPU_H */
