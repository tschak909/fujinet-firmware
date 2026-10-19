/* nes_ppu.h -- the NES cartridge (Rev1) as seen from RP-B, the RP2354B on the
 * PPU bus: CHR, CHR-RAM and the CIRAM lines. RP-A loads RP-B over SWD at
 * every boot and drives it over the link (nes_link.h); RP-B runs the CHR half
 * of the same mapper engine on the writes RP-A forwards.
 *
 * Timing facts (nesdev "PPU rendering"): a PPU access is two dots of 186 ns,
 * the address phase then /RD (or /WR) low; the PPU latches read data as /RD
 * rises, one dot after it fell.
 *
 * Hardware-free: RP-B's firmware, the bench and the board check share it.
 */
#ifndef NES_PPU_H
#define NES_PPU_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * GPIO map, RP-B (U2, RP2354B), 38 of 48. The PPU bus, GP0-GP23, is SIO's
 * (one read covers it); the CIRAM table is PIO0 side-set (GPIOBASE 0); the
 * link is PIO1 (GPIOBASE 16). PD0-PD7 and the CIRAM lines stay off (OEOVER)
 * until PWR_OK and RP-A's MODE SERVE frame.
 * -------------------------------------------------------------------------- */
#define PA0_PIN        0u   /* PPU A0-A13 on GP0-GP13                        */
#define PD0_PIN       14u   /* PPU D0-D7 on GP14-GP21                        */
#define PPU_RD_PIN    22u   /* /RD                                           */
#define PPU_WR_PIN    23u   /* /WR                                           */
#define PPU_PWROK_PIN 24u   /* console 5 V present (same divider as RP-A's)  */
#define CIRAM_A10_PIN 25u   /* CIRAM A10, PIO0 side-set bit 0                */
#define CIRAM_CE_PIN  26u   /* CIRAM /CE, side-set bit 1                     */
                            /* GP27-GP32: spare                              */
#define RPB_LFRM_PIN  33u   /* link frame, from RP-A                         */
#define RPB_LCLK_PIN  34u   /* link clock                                    */
#define RPB_LD7_PIN   35u   /* LD7..LD0 on GP35-GP42: mirrored, so the bus   */
                            /* between the two RPs is planar; RP-B bit-      */
                            /* reverses each frame                           */
#define RPB_LACK_PIN  43u   /* RP-B -> RP-A                                  */
#define VSENSE_PIN    44u   /* ADC4: console 5 V through the PWR_OK divider  */
                            /* GP45-GP47: spare                              */

#define PA_MASK       0x3FFFu
#define PD_MASK       (0xFFu << PD0_PIN)
#define PPU_RD_MASK   (1u << PPU_RD_PIN)
#define PPU_WR_MASK   (1u << PPU_WR_PIN)

/* RP-B's status, at an address RP-A learns from the image header
 * (rpb_image.h) and reads over SWD for its diagnostics. */
#define RPB_STATUS_MAGIC 0x52504221u           /* "RPB!" */
#define RPB_VERSION      1u

typedef struct {
    uint32_t magic;               /* RPB_STATUS_MAGIC once running           */
    uint32_t version;             /* RPB_VERSION                             */
    uint32_t mode;                /* NES_LINK_MODE_*                         */
    uint32_t frames;              /* link frames taken                       */
    uint32_t writes;              /* WRITE frames applied                    */
    uint32_t bad_frames;          /* unknown ops, out-of-order data          */
    uint32_t chr_len;             /* CHR bytes expected / received           */
    uint32_t chr_got;
    uint32_t max_apply;           /* cycles, frame seen -> slots in          */
    uint32_t pwrok;               /* console 5 V seen                        */
} rpb_status_t;

#endif /* NES_PPU_H */
