/* dut_cpu.c -- RP-A's CPU bus on the bench: the firmware's engine
 * (nes_engine.c, built here with NES_ENGINE_BENCH_DATA: bench writes carry
 * their data in A1-A8, as SIO owns D0-D7 for the reads) on PIO2, PIO0 and
 * PIO1 being the console and the logic analyser.
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"

#include "bench.h"
#include "dut_cpu.h"

dut_cpu_t dut_cpu __attribute__((aligned(16)));

/* The firmware's ring to core0; nothing listens on the bench. */
void nes_engine_note(uint16_t offset, uint8_t data, uint8_t kind, uint32_t cycle)
{
    (void)offset; (void)data; (void)kind; (void)cycle;
}

void dut_engine_load(void)
{
    dut_cpu.eng.pio = pio2;
    nes_engine_init(&dut_cpu.eng);
}

void dut_engine_start(void)
{
    unsigned p;

    /* The bench's calibration hands D0-D7 to SIO; the engine's PIO drives
     * them on the cart. */
    for (p = CD0_PIN; p < CD0_PIN + 8; p++)
        pio_gpio_init(pio2, p);
    nes_engine_start(&dut_cpu.eng);
    /* Optional: no 2-cycle input synchroniser on the bus pins the engine
     * reads. The address is settled well before M2 rises; M2 itself is the
     * one asynchronous edge. */
    pio2->input_sync_bypass = bench_sync_bypass ?
        (CA_LO_MASK | M2_MASK | RW_MASK | ROMSEL_MASK | CA13_MASK | CA14_MASK) : 0;
}

void dut_engine_stop(void)
{
    nes_engine_stop(&dut_cpu.eng);
}

void __not_in_flash_func(dut_engine_main)(void)
{
    while (sio_hw->gpio_in & M2_MASK)
        ;
    dut_cpu.running = 1;
    nes_engine_core1(&dut_cpu.eng);
}
