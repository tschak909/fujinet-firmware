/* nes_qmi.h -- RP-A's flash interface, set up for the engine's flash path.
 *
 * A 512K image's PRG is read through the uncached XIP alias one byte at a
 * time, inside M2 high: SCK at clk_sys / 2 (133 MHz at 266 MHz, the flash's
 * limit), COOLDOWN 0, and the flash in continuous-read mode (EBh with A0 mode
 * bits once, then address-only reads: 14 SCK a byte instead of 22). 117 ns a
 * read at 266 MHz; the whole engine path is ~215 ns from M2.
 *
 * The bootrom's flash functions take the flash out of continuous mode and
 * reset the QMI, so every erase or program is followed by nes_qmi_setup().
 */
#ifndef NES_QMI_H
#define NES_QMI_H

#include <stdbool.h>
#include <stdint.h>

/* Once at boot (clk_sys final): clock, continuous mode, and the RX delay in
 * the middle of the window that reads flash back clean. False if no delay
 * read back clean (then a slower clock is left in place). */
bool nes_qmi_init(void);

/* After a flash erase or program: the same clock and delay, and continuous
 * mode entered again. */
void nes_qmi_setup(void);

#endif /* NES_QMI_H */
