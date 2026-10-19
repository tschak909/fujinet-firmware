/* nes_link.h -- the RP-A -> RP-B link (Rev1): what crosses it.
 *
 * Eight data lines, LCLK (data changes, then LCLK toggles; both edges carry
 * a beat) and LFRM (high for a frame, low between frames, so a receiver that
 * lost a beat realigns on the next one). A frame is one 32-bit word, four
 * beats, most significant byte first. RP-B runs the same mapper engine as
 * RP-A on the CHR side, so the common frame is a raw console write. LACK is
 * RP-B's one line back: high while it is ready for frames.
 *
 * An install, as RP-A sends it (the console is in the loader, so no WRITE
 * frames interleave):
 *   MODE OFF, HEADER x6 (the iNES header, 3 bytes a frame), CHR length, then
 *   CHR DATA (3 bytes a frame) or CLEAR for CHR-RAM, then MODE SERVE.
 * RP-B parses the header with the same nesmap as RP-A, so both halves start
 * from one plan.
 *
 * Hardware-free: RP-A, RP-B, the bench and the host tests share it.
 */
#ifndef NES_LINK_H
#define NES_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* Bits 31-28: the op. */
#define NES_LINK_OP_WRITE   1u  /* a mapper write: bit 27 consecutive (the */
                                /* cycle after a write), 23-8 the CPU     */
                                /* address, 7-0 the data                  */
#define NES_LINK_OP_MODE    2u  /* 3-0: NES_LINK_MODE_*                    */
#define NES_LINK_OP_HEADER  3u  /* 27-24: frame index 0-5; 23-0: header   */
                                /* bytes 3i..3i+2, first in bits 23-16    */
#define NES_LINK_OP_CHRLEN  4u  /* 23-0: CHR bytes to follow (0: CHR-RAM) */
#define NES_LINK_OP_DATA    5u  /* 23-0: the next three CHR bytes          */
#define NES_LINK_OP_CLEAR   6u  /* zero the CHR-RAM and the nametables     */

#define NES_LINK_MODE_OFF   0u  /* PD0-PD7 and CIRAM off the bus (OEOVER)  */
#define NES_LINK_MODE_SERVE 1u  /* serve the PPU                           */

#define NES_LINK_OP(f)     ((f) >> 28)

/* A console write as one frame. `consec`: the write came the cycle after
 * another (MMC1 ignores it). */
#define NES_LINK_WRITE(consec, a, d) \
    ((NES_LINK_OP_WRITE << 28) | ((uint32_t)(consec) << 27) | ((uint32_t)(uint16_t)(a) << 8) | (uint8_t)(d))

#define NES_LINK_WRITE_CONSEC(f) (((f) >> 27) & 1u)
#define NES_LINK_WRITE_ADDR(f)   ((uint16_t)((f) >> 8))
#define NES_LINK_WRITE_DATA(f)   ((uint8_t)(f))

#define NES_LINK_MODE(m)         ((NES_LINK_OP_MODE << 28) | ((m) & 0xFu))
#define NES_LINK_HEADER(i, b0, b1, b2) \
    ((NES_LINK_OP_HEADER << 28) | ((uint32_t)(i) << 24) | ((uint32_t)(b0) << 16) | ((uint32_t)(b1) << 8) | (b2))
#define NES_LINK_CHRLEN(n)       ((NES_LINK_OP_CHRLEN << 28) | ((n) & 0xFFFFFFu))
#define NES_LINK_DATA(b0, b1, b2) \
    ((NES_LINK_OP_DATA << 28) | ((uint32_t)(b0) << 16) | ((uint32_t)(b1) << 8) | (b2))
#define NES_LINK_CLEAR           (NES_LINK_OP_CLEAR << 28)

#define NES_LINK_HEADER_FRAMES   6u      /* 16 bytes, three a frame (+2 pad) */

/* An install as frames, in order (see the top): `send` takes each one and
 * returns false to stop (RP-B not taking frames). RP-A's nes_link_tx.c and
 * the host tests share it. */
typedef bool (*nes_link_send_fn)(void *ctx, uint32_t frame, uint32_t chr_done);

static inline bool nes_link_encode_image(const uint8_t header[16], const uint8_t *chr, uint32_t chr_len,
                                         nes_link_send_fn send, void *ctx)
{
    uint8_t h[18] = { 0 };
    uint32_t i;

    if (!send(ctx, NES_LINK_MODE(NES_LINK_MODE_OFF), 0))
        return false;
    for (i = 0; i < 16; i++)
        h[i] = header[i];
    for (i = 0; i < NES_LINK_HEADER_FRAMES; i++)
        if (!send(ctx, NES_LINK_HEADER(i, h[3 * i], h[3 * i + 1], h[3 * i + 2]), 0))
            return false;
    if (!chr)
        chr_len = 0;
    if (!send(ctx, NES_LINK_CHRLEN(chr_len), 0))
        return false;
    if (!chr_len)
        return send(ctx, NES_LINK_CLEAR, 0);
    for (i = 0; i < chr_len; i += 3) {
        uint8_t b1 = i + 1 < chr_len ? chr[i + 1] : 0;
        uint8_t b2 = i + 2 < chr_len ? chr[i + 2] : 0;

        if (!send(ctx, NES_LINK_DATA(chr[i], b1, b2), i))
            return false;
    }
    return true;
}

/* RP-B's pins are LD7..LD0 (mirrored, for a planar bus): a receiver that
 * shifts its ISR right gets each frame bit-reversed. */
static inline uint32_t nes_link_unmirror(uint32_t raw)
{
    uint32_t x = raw;

    x = ((x >> 1) & 0x55555555u) | ((x & 0x55555555u) << 1);
    x = ((x >> 2) & 0x33333333u) | ((x & 0x33333333u) << 2);
    x = ((x >> 4) & 0x0F0F0F0Fu) | ((x & 0x0F0F0F0Fu) << 4);
    x = ((x >> 8) & 0x00FF00FFu) | ((x & 0x00FF00FFu) << 8);
    return (x >> 16) | (x << 16);
}

#endif /* NES_LINK_H */
