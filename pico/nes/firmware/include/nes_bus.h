/* nes_bus.h -- the CPU bus pins on RP-A: Rev0's map, kept in Rev1, and the
 * Phase 0 bench's (pico/nes/bench) as well, so the bench runs the engine on
 * the cart's own pins. Everything core1 reads is in GPIO 0-31, so one SIO
 * read covers the bus.
 *
 * Timing facts (nesdev "Cartridge connector", not to be re-derived): NTSC M2
 * is high 350 ns and low 209 ns of a 559 ns cycle; the address is guaranteed
 * stable only while M2 is high; /ROMSEL = NAND(M2, A15) through the console's
 * LS139 lags M2 by up to ~38 ns; write data is valid at M2's falling edge.
 */
#ifndef NES_BUS_H
#define NES_BUS_H

#define CA0_PIN      0u     /* CPU A0-A12 on GP0-GP12                       */
#define CD0_PIN      13u    /* CPU D0-D7 on GP13-GP20, bidirectional        */
#define M2_PIN       21u    /* also PWM slice 2 input B: the cycle counter  */
#define RW_PIN       22u    /* 1 = the console reads                        */
#define ROMSEL_PIN   23u    /* /ROMSEL = NAND(M2, A15): low = $8000-$FFFF   */
#define CA13_PIN     24u    /* CPU A13, A14 on GP24-GP25                    */

#define CA_LO_MASK   0x1FFFu                      /* A0-A12                 */
#define DATA_MASK    (0xFFu << CD0_PIN)
#define M2_MASK      (1u << M2_PIN)
#define RW_MASK      (1u << RW_PIN)
#define ROMSEL_MASK  (1u << ROMSEL_PIN)
#define CA13_MASK    (1u << CA13_PIN)
#define CA14_MASK    (1u << (CA13_PIN + 1))

#endif /* NES_BUS_H */
