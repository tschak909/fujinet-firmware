/* ppu_link.c -- RP-B's side of the link. See ppu_link.h. */

#include <string.h>

#include "nes_link.h"
#include "ppu_link.h"

static void publish(ppu_link_t *l)
{
    ppu_slots_t *s = l->slots;
    unsigned i;

    for (i = 0; i < 8; i++) {
        s->rslot[i] = l->fc.slot[i];
        s->wslot[i] = l->chr_is_ram ? (uint8_t *)l->fc.slot[i] : NULL;
    }
    for (i = 8; i < 16; i++) {
        uint8_t *nt = l->fc.mirror == NESMAP_MIR_4 ? l->nt_ram + ((i & 3) << 10) : NULL;

        s->rslot[i] = nt;
        s->wslot[i] = nt;
    }
}

static void unpublish(ppu_link_t *l)
{
    unsigned i;

    for (i = 0; i < 16; i++) {
        l->slots->rslot[i] = NULL;
        l->slots->wslot[i] = NULL;
    }
}

void ppu_link_init(ppu_link_t *l)
{
    memset(l->hdr, 0, sizeof l->hdr);
    l->have_image = l->serving = l->chr_is_ram = false;
    l->chr_pos = 0;
    unpublish(l);
}

static void take_header(ppu_link_t *l)
{
    nesmap_plan_t plan;

    l->have_image = false;
    if (nesmap_plan_split(l->hdr, NULL, 0xFFFFFFFFu, &plan) != NESMAP_OK ||
        plan.chr_size > l->chr_max || plan.chr_ram_size > l->chr_max) {
        l->status->bad_frames++;
        return;
    }
    if (nesmap_init(&l->map, &plan) != NESMAP_OK)
        return;
    nesmap_set_outputs(&l->map, NESMAP_OUT_CHR);
    if (!fastchr_init(&l->fc, &l->map, l->chr_mem))
        return;
    l->chr_is_ram = plan.chr_size == 0;
    l->have_image = true;
}

void ppu_link_frame(ppu_link_t *l, uint32_t f)
{
    volatile rpb_status_t *st = l->status;

    st->frames++;
    switch (NES_LINK_OP(f)) {
    case NES_LINK_OP_WRITE: {
        uint32_t t0;

        if (!l->serving)
            break;
        t0 = l->cycles ? l->cycles() : 0;
        fastchr_write(&l->fc, NES_LINK_WRITE_ADDR(f), NES_LINK_WRITE_DATA(f), NES_LINK_WRITE_CONSEC(f));
        publish(l);
        if (l->fc.mirror_changed) {
            l->fc.mirror_changed = false;
            l->ciram(l->fc.mirror);
        }
        if (l->cycles) {
            t0 = l->cycles() - t0;
            if (t0 > st->max_apply)
                st->max_apply = t0;
        }
        st->writes++;
        break;
    }
    case NES_LINK_OP_MODE:
        st->mode = f & 0xF;
        l->serving = (f & 0xF) == NES_LINK_MODE_SERVE && l->have_image;
        if (l->serving) {
            publish(l);
            l->ciram(l->fc.mirror);
        } else {
            unpublish(l);
        }
        l->gate(l->serving);
        break;
    case NES_LINK_OP_HEADER: {
        unsigned i = (f >> 24) & 0xF;

        if (i < NES_LINK_HEADER_FRAMES) {
            l->hdr[3 * i] = (uint8_t)(f >> 16);
            l->hdr[3 * i + 1] = (uint8_t)(f >> 8);
            l->hdr[3 * i + 2] = (uint8_t)f;
        }
        break;
    }
    case NES_LINK_OP_CHRLEN:
        take_header(l);
        st->chr_len = f & 0xFFFFFFu;
        st->chr_got = 0;
        l->chr_pos = 0;
        break;
    case NES_LINK_OP_DATA: {
        uint32_t n = st->chr_len;
        unsigned k;

        for (k = 0; k < 3 && l->chr_pos < n && l->chr_pos < l->chr_max; k++)
            l->chr_mem[l->chr_pos++] = (uint8_t)(f >> (16 - 8 * k));
        st->chr_got = l->chr_pos;
        break;
    }
    case NES_LINK_OP_CLEAR:
        if (l->chr_is_ram)
            memset(l->chr_mem, 0, l->map.plan.chr_ram_size);
        memset(l->nt_ram, 0, 4096);
        break;
    default:
        st->bad_frames++;
        break;
    }
}
