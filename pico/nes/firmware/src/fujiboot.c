/* fujiboot.c -- bring the cartridge up as FujiNet.
 *
 * The console client is baked into this firmware, because at power-up there
 * is no network and nothing to load it from. It is the resident image: the
 * cart installs it at power-on the way it installs a network image, its PRG
 * served in place from SRAM.
 */

#include <string.h>

#include "fujiboot.h"
#include "fuji_cart.h"
#include "fujinet.h"
#include "fuji_mailbox.h"
#include "nesmap.h"
#include "fujimail.h"
#include "fujiconfigrom.h"
#include "nesloaderrom.h"

void fuji_config_boot(void)
{
    nesmap_plan_t plan;
    fuji_image_t img = {
        .hdr = _configrom_hdr,
        .body = _configrom_prg,       /* SRAM, 8K-aligned: served in place */
        .size = FUJI_CONFIGROM_SIZE,
        .flash_off = 0,
        .chr = FUJI_CONFIGROM_CHR_SIZE ? _configrom_chr : NULL,   /* flash: RP-B's */
    };

    fuji_cart_init(_loaderrom, FUJI_LOADERROM_SIZE);
    if (nesmap_plan_split(img.hdr, img.body, img.size, &plan) == NESMAP_OK)
        fuji_cart_set_resident(&img, &plan);
    fuji_service_init();
    fujimail_paint();
}
