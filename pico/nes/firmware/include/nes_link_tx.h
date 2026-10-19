/* nes_link_tx.h -- RP-A's end of the link (nes_link.h): PIO1 SM0.
 *
 * core1 (the engine) stores one WRITE frame per mapper write straight into
 * the FIFO (nes_link_tx_fifo); core0 sends the install frames, only while the
 * engine is parked, so the two never share the FIFO. The FIFO is joined, 8
 * deep; a frame takes 36 PIO cycles, and mapper writes come at most every
 * other bus cycle (~300 cycles), so core1's stores never find it full.
 */
#ifndef NES_LINK_TX_H
#define NES_LINK_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"

void nes_link_tx_init(void);
io_wo_32 *nes_link_tx_fifo(void);

/* RP-B's LACK: high while it takes frames. */
bool nes_link_rpb_ready(void);

/* core0, engine parked. Block until each frame is in the FIFO. */
void nes_link_send(uint32_t frame);
/* The install: MODE OFF, the header, CHR (or CLEAR for CHR-RAM). Returns
 * false if RP-B stopped taking frames (LACK low for too long). */
bool nes_link_send_image(const uint8_t header[16], const uint8_t *chr, uint32_t chr_len);
/* Wait until the FIFO and the shift register are empty. */
void nes_link_flush(void);

#endif /* NES_LINK_TX_H */
