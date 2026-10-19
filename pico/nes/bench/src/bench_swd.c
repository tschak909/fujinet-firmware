/* bench_swd.c -- RP-A's SWD host (firmware/src/swd_host.c) against this
 * chip's own debug port, through SYSCFG DBGFORCE: the code that loads RP-B,
 * tested with no second chip and no wiring. It reaches core1's AHB-AP,
 * checks memory both ways against core0's view, then loads a four-
 * instruction program into core1, points it there and lets it run. Only
 * RUN and the rescue reset (they would reset this chip) are left out.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/timer.h"

#include "bench.h"
#include "swd_host.h"

static uint32_t pattern[1024];
static uint32_t target[1024];
static uint32_t prog[4] __attribute__((aligned(16)));
static uint32_t stack1[64] __attribute__((aligned(8)));
static volatile uint32_t flag;

static void __not_in_flash_func(core1_spin)(void)
{
    for (;;)
        __asm volatile("nop");
}

void bench_swd(void)
{
    uint32_t id = 0, v = 0, i, dbgpause;
    bool connect, power, ap, rd, wr, halt, run = false;

    /* core1 in a plain loop to be halted in (not the bootrom's FIFO wait) */
    multicore_reset_core1();
    multicore_launch_core1(core1_spin);
    /* The timer pauses while either core is debug-halted (DBGPAUSE): with
     * host and target on one chip, core0's waits would never end. */
    dbgpause = timer_hw->dbgpause;
    timer_hw->dbgpause = 0;
    for (i = 0; i < 1024; i++) {
        pattern[i] = rnd(&id) ^ (i * 0x9E3779B9u);
        target[i] = 0;
    }
    flag = 0;
    /* ldr r0,[pc,#4]; ldr r1,[pc,#8]; str r1,[r0]; b . ; &flag ; value */
    prog[0] = 0x49024801u;
    prog[1] = 0xE7FE6001u;
    prog[2] = (uint32_t)&flag;
    prog[3] = 0xC0DE1234u;

    swd_init(0, 0);
    connect = swd_connect(&id);
    power = connect && swd_power_up();
    ap = power && swd_mem_ap(SWD_AP_CORE1);
    rd = ap && swd_mem_read((uint32_t)&pattern[7], &v) && v == pattern[7];
    wr = ap && swd_mem_write_block((uint32_t)target, pattern, 1024) &&
         swd_mem_verify_block((uint32_t)target, pattern, 1024) &&
         memcmp(target, pattern, sizeof target) == 0;
    halt = ap && swd_core_halt();
    if (halt && swd_core_set_reg(13, (uint32_t)&stack1[64]) &&
        swd_core_set_reg(15, (uint32_t)prog) && swd_core_set_reg(16, 1u << 24) &&
        swd_core_resume()) {
        for (i = 0; i < 1000 && flag != 0xC0DE1234u; i++)
            busy_wait_us(10);
        run = flag == 0xC0DE1234u;
    }
    if (halt)
        swd_core_halt();
    swd_release();
    multicore_reset_core1();
    timer_hw->dbgpause = dbgpause;
    REPORT("R swd dpidr=%08lx connect=%d power=%d ap=%d read=%d write=%d halt=%d run=%d\n",
           (unsigned long)id, connect, power, ap, rd, wr, halt, run);
}
