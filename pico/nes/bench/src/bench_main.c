/* bench_main.c -- the NES Rev1 timing bench: commands over USB serial.
 *
 *   run [mhz ...]   the whole suite (default: 200 then 250 MHz)
 *   quick           the same with a tenth of the cycles
 *   clk <mhz>       change the system clock (and the core voltage with it)
 *   xip | map | cpu | ppu | link    one test at the current clock
 *   stress 0|1      a DMA hammering RAM while the CPU test runs
 *   cycles <n>      cycles per confirmation run (default 10 000 000)
 *
 * Every result is one line starting "R "; tools/bench_report.py reads them
 * and applies the pass/fail thresholds. "R done" ends a run.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/structs/sio.h"

#include "bench.h"

uint8_t bench_big[PRG_RAM_SIZE] __attribute__((aligned(8192)));   /* 8K banks: the PRG engine ORs offsets in */
uint32_t bench_stim[2 * BLOCK] __attribute__((aligned(2 * BLOCK * 4)));
uint32_t bench_exp[BLOCK];
uint8_t bench_cap0[BLOCK] __attribute__((aligned(BLOCK)));
uint8_t bench_cap1[BLOCK] __attribute__((aligned(BLOCK)));
uint32_t bench_mhz = 150;
uint32_t bench_confirm_cycles = 10000000;
bool bench_stress;
bool bench_atrans;
bool bench_sync_bypass;

void bench_dwt_enable(void)
{
    sio_hw->mtime_ctrl = SIO_MTIME_CTRL_EN_BITS | SIO_MTIME_CTRL_FULLSPEED_BITS;
}

/* A fault (a stray read of a remapped XIP window, say) would otherwise take
 * USB down with it and need a replug: reboot instead, so picotool can still
 * reach the bench. In RAM, like everything else here, and not the SDK's
 * default, which lives in flash. */
void __not_in_flash_func(isr_hardfault)(void)
{
    watchdog_reboot(0, 0, 0);
    for (;;)
        ;
}

void bench_pio_reset(void)
{
    PIO pios[3] = { pio0, pio1, pio2 };
    unsigned i, sm;

    multicore_reset_core1();
    for (i = 0; i < 3; i++) {
        for (sm = 0; sm < 4; sm++) {
            pio_sm_set_enabled(pios[i], sm, false);
            pio_sm_clear_fifos(pios[i], sm);
            pio_sm_restart(pios[i], sm);
        }
        pio_clear_instruction_memory(pios[i]);
        pios[i]->irq = 0xFF;
        pios[i]->input_sync_bypass = 0;
    }
}

static bool set_clock(uint32_t mhz)
{
    if (mhz > 200) {
        vreg_set_voltage(mhz > 266 ? VREG_VOLTAGE_1_30 : mhz > 250 ? VREG_VOLTAGE_1_20 : VREG_VOLTAGE_1_15);
        sleep_ms(2);
    }
    if (!set_sys_clock_khz(mhz * 1000, false)) {
        REPORT("R clk mhz=%lu error=unreachable\n", (unsigned long)mhz);
        return false;
    }
    if (mhz <= 200)
        vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
    bench_mhz = mhz;
    bench_dwt_enable();
    bench_qmi_tune();
    REPORT("R clk mhz=%lu sys_hz=%lu\n", (unsigned long)mhz, (unsigned long)clock_get_hz(clk_sys));
    return true;
}

static void suite(uint32_t mhz)
{
    if (!set_clock(mhz))
        return;
    bench_micro_xip();
    bench_micro_nesmap();
    bench_stress = false;
    bench_cpu();
    bench_stress = true;
    bench_cpu();
    bench_stress = false;
    bench_ppu();
    bench_link();
    REPORT("R suite_done mhz=%lu\n", (unsigned long)mhz);
}

static bool read_line(char *buf, size_t n)
{
    size_t len = 0;

    for (;;) {
        int c = getchar_timeout_us(100000);

        if (c == PICO_ERROR_TIMEOUT)
            continue;
        if (c == '\r' || c == '\n') {
            if (len == 0)
                continue;
            buf[len] = 0;
            return true;
        }
        if (len + 1 < n)
            buf[len++] = (char)c;
    }
}

int main(void)
{
    char line[80];

    stdio_init_all();
    while (!stdio_usb_connected())
        sleep_ms(50);
    sleep_ms(200);
    REPORT("R hello bench=nes-rev1 board=%s\n", PICO_BOARD);
    bench_dwt_enable();
    bench_flash_image();
    bench_qmi_report("boot");
    REPORT("R ready\n");

    for (;;) {
        char *arg;

        read_line(line, sizeof line);
        arg = strchr(line, ' ');
        if (arg)
            *arg++ = 0;

        if (!strcmp(line, "run") || !strcmp(line, "quick")) {
            uint32_t keep = bench_confirm_cycles;

            if (!strcmp(line, "quick"))
                bench_confirm_cycles /= 10;
            if (arg && *arg) {
                while (arg && *arg) {
                    suite((uint32_t)strtoul(arg, &arg, 10));
                    while (arg && *arg == ' ')
                        arg++;
                }
            } else {
                suite(200);
                suite(250);
            }
            bench_confirm_cycles = keep;
            REPORT("R done\n");
        } else if (!strcmp(line, "clk") && arg) {
            set_clock((uint32_t)strtoul(arg, NULL, 10));
            REPORT("R done\n");
        } else if (!strcmp(line, "xip")) {
            bench_micro_xip();
            REPORT("R done\n");
        } else if (!strcmp(line, "swd")) {
            bench_swd();
            REPORT("R done\n");
        } else if (!strcmp(line, "apb")) {
            bench_micro_apb();
            REPORT("R done\n");
        } else if (!strcmp(line, "ipc")) {
            bench_micro_ipc();
            REPORT("R done\n");
        } else if (!strcmp(line, "map")) {
            bench_micro_nesmap();
            REPORT("R done\n");
        } else if (!strcmp(line, "cpu")) {
            bench_cpu();
            REPORT("R done\n");
        } else if (!strcmp(line, "ppu")) {
            bench_ppu();
            REPORT("R done\n");
        } else if (!strcmp(line, "link")) {
            bench_link();
            REPORT("R done\n");
        } else if (!strcmp(line, "engine") && arg) {
            bench_atrans = !strcmp(arg, "atrans");
            REPORT("R engine=%s\nR done\n", bench_atrans ? "atrans" : "dma");
        } else if (!strcmp(line, "bypass") && arg) {
            bench_sync_bypass = atoi(arg) != 0;
            REPORT("R bypass=%d\nR done\n", bench_sync_bypass);
        } else if (!strcmp(line, "stress") && arg) {
            bench_stress = atoi(arg) != 0;
            REPORT("R stress=%d\nR done\n", bench_stress);
        } else if (!strcmp(line, "cycles") && arg) {
            bench_confirm_cycles = (uint32_t)strtoul(arg, NULL, 10);
            REPORT("R cycles=%lu\nR done\n", (unsigned long)bench_confirm_cycles);
        } else {
            REPORT("R error unknown=%s\nR done\n", line);
        }
    }
}
