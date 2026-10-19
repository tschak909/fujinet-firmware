/* test_link.c -- the RP-A -> RP-B link, end to end, against nesmap.
 *
 * RP-A's install encoder (nes_link.h) sends an image -- header, CHR or a
 * CHR-RAM clear, MODE SERVE -- then every mapper write as a WRITE frame. Each
 * frame goes over the wire as RP-A's PIO puts it out (four beats, most
 * significant byte first, on LD0-LD7) and comes in as RP-B's receiver sees
 * it (its pins are LD7..LD0, its ISR shifts right), through
 * nes_link_unmirror, into RP-B's link code (ppu/src/ppu_link.c). After the
 * install and after every write, RP-B's CHR, slot table, write gates,
 * nametable slots and CIRAM mode must match nesmap's.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nes_link.h"
#include "ppu_link.h"

static uint32_t rng = 0x11AC0DE5u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

#define CHR_MAX (256u * 1024)

static uint8_t image_chr[CHR_MAX];
static uint8_t chr_mem[CHR_MAX];
static uint8_t nt_ram[4096];
static ppu_slots_t slots;
static volatile rpb_status_t status;
static unsigned ciram_mode;
static bool gated;
static ppu_link_t L;
static unsigned failures;

static void ciram(unsigned m) { ciram_mode = m; }
static void gate(bool on) { gated = on; }

/* RP-A's beats on LD0-LD7, MSB byte first; RP-B reads LD7..LD0 into an
 * ISR shifting right. */
static uint32_t over_the_wire(uint32_t f)
{
    uint32_t raw = 0;
    int k, i;

    for (k = 0; k < 4; k++) {
        uint8_t b = (uint8_t)(f >> (24 - 8 * k)), r = 0;

        for (i = 0; i < 8; i++)
            r |= (uint8_t)(((b >> (7 - i)) & 1) << i);
        raw = (raw >> 8) | ((uint32_t)r << 24);
    }
    return raw;
}

static bool to_rpb(void *ctx, uint32_t frame, uint32_t done)
{
    (void)ctx; (void)done;
    ppu_link_frame(&L, nes_link_unmirror(over_the_wire(frame)));
    return true;
}

/* NES 2.0, so CHR-RAM sizes and submappers say what they mean. */
static void header(uint8_t h[16], uint16_t mapper, uint32_t prg, uint32_t chr_rom, uint32_t chr_ram,
                   bool four, uint8_t sub)
{
    unsigned shift = 0;

    memset(h, 0, 16);
    memcpy(h, "NES\x1A", 4);
    h[4] = (uint8_t)(prg / 16384);
    h[5] = (uint8_t)(chr_rom / 8192);
    h[6] = (uint8_t)(((mapper & 0xF) << 4) | (four ? 8 : 0) | 1);
    h[7] = (uint8_t)((mapper & 0xF0) | 0x08);
    h[8] = (uint8_t)((sub << 4) | ((mapper >> 8) & 0xF));
    h[10] = 7;                                     /* 8K PRG-RAM */
    while (chr_ram && (64u << shift) < chr_ram)
        shift++;
    h[11] = (uint8_t)(chr_ram ? shift : 0);
}

static void check(const char *what, uint16_t mapper, unsigned step, const nesmap_t *m, uint32_t chr_rom)
{
    int i, bad = 0;

    for (i = 0; i < 8; i++) {
        const uint8_t *want = chr_mem + ((uint32_t)m->out.chr[i] << 10);

        if (slots.rslot[i] != want)
            bad |= 1;
        if (slots.wslot[i] != (chr_rom ? NULL : want))
            bad |= 2;
    }
    for (i = 8; i < 16; i++) {
        const uint8_t *nt = m->out.mirror == NESMAP_MIR_4 ? nt_ram + ((i & 3) << 10) : NULL;

        if (slots.rslot[i] != nt || slots.wslot[i] != nt)
            bad |= 4;
    }
    if (ciram_mode != m->out.mirror)
        bad |= 8;
    if (!gated)
        bad |= 16;
    if (!bad)
        return;
    failures++;
    if (failures <= 10)
        printf("FAIL %s mapper %u step %u:%s%s%s%s%s\n", what, mapper, step, bad & 1 ? " chr" : "",
               bad & 2 ? " wgate" : "", bad & 4 ? " nametables" : "", bad & 8 ? " ciram" : "",
               bad & 16 ? " gate" : "");
}

static void run(uint16_t mapper, uint32_t prg, uint32_t chr_rom, uint32_t chr_ram, bool four, uint8_t sub)
{
    uint8_t h[16];
    nesmap_plan_t plan;
    nesmap_t m;
    uint32_t i, cycle = 100, last_w = 0;
    unsigned step;

    header(h, mapper, prg, chr_rom, chr_ram, four, sub);
    for (i = 0; i < chr_rom; i++)
        image_chr[i] = (uint8_t)(rnd() >> 7);
    memset(chr_mem, 0xA5, sizeof chr_mem);         /* a CLEAR must happen */

    assert(nes_link_encode_image(h, chr_rom ? image_chr : NULL, chr_rom, to_rpb, NULL));
    to_rpb(NULL, NES_LINK_MODE(NES_LINK_MODE_SERVE), 0);

    assert(nesmap_plan_split(h, NULL, 0xFFFFFFFFu, &plan) == NESMAP_OK);
    assert(nesmap_init(&m, &plan) == NESMAP_OK);
    if (chr_rom ? memcmp(chr_mem, image_chr, chr_rom) != 0 : chr_mem[0] != 0 || chr_mem[chr_ram - 1] != 0) {
        failures++;
        printf("FAIL install mapper %u: CHR %s\n", mapper, chr_rom ? "differs" : "not cleared");
    }
    check("install", mapper, 0, &m, chr_rom);

    for (step = 1; step <= 20000; step++) {
        uint32_t r = rnd();
        uint16_t a = (uint16_t)(0x8000u | (r & 0x7FFFu));
        uint8_t d = (uint8_t)(r >> 16);
        bool consec;

        if (mapper == 34 && (r >> 31))
            a = (uint16_t)(0x7FFDu + ((r >> 24) % 3));
        cycle += (r >> 28) % 3 + 1;
        consec = cycle == last_w + 1;
        last_w = cycle;
        nesmap_write(&m, a, d, cycle);
        to_rpb(NULL, NES_LINK_WRITE(consec, a, d), 0);
        check("write", mapper, step, &m, chr_rom);
        if (failures > 10)
            return;
    }
    to_rpb(NULL, NES_LINK_MODE(NES_LINK_MODE_OFF), 0);
    for (i = 0; i < 16; i++)
        if (slots.rslot[i] || slots.wslot[i])
            break;
    if (i < 16 || gated) {
        failures++;
        printf("FAIL mode off mapper %u: slot %u still live or bus gated on\n", mapper, i);
    }
}

int main(void)
{
    static const struct { uint16_t mapper; uint32_t prg, chr_rom, chr_ram; bool four; uint8_t sub; } cases[] = {
        {   0,  32, 8, 0, 0, 0 },
        {   1,  32, 0, 8, 0, 0 }, {   1, 128, 128, 0, 0, 0 }, {   1, 512, 0, 8, 0, 0 },
        {   2, 128, 0, 8, 0, 0 },
        {   3,  32, 32, 0, 0, 0 },
        {   4, 512, 256, 0, 0, 0 }, {   4, 128, 0, 8, 0, 0 }, {   4, 256, 128, 0, 1, 0 },
        {   7, 256, 0, 8, 0, 0 },
        {  11, 128, 128, 0, 0, 0 },
        {  30, 512, 0, 32, 1, 0 }, {  30, 256, 0, 32, 0, 0 },
        {  34, 128, 0, 8, 0, 0 }, {  34,  64, 64, 0, 0, 0 },
        {  66, 128, 32, 0, 0, 0 },
        {  71, 128, 0, 8, 0, 1 },
        { 206, 128, 64, 0, 0, 0 },
    };
    unsigned i;

    L.chr_mem = chr_mem;
    L.chr_max = CHR_MAX;
    L.nt_ram = nt_ram;
    L.slots = &slots;
    L.status = &status;
    L.ciram = ciram;
    L.gate = gate;
    L.cycles = NULL;
    ppu_link_init(&L);

    for (i = 0; i < sizeof cases / sizeof cases[0]; i++)
        run(cases[i].mapper, cases[i].prg * 1024u, cases[i].chr_rom * 1024u, cases[i].chr_ram * 1024u,
            cases[i].four, cases[i].sub);
    if (status.bad_frames) {
        failures++;
        printf("FAIL %u frames RP-B did not understand\n", (unsigned)status.bad_frames);
    }
    if (failures) {
        printf("test_link: %u failures\n", failures);
        return 1;
    }
    printf("test_link: %u images installed over the wire, 20000 writes each: RP-B's CHR, slots, "
           "gates and CIRAM match nesmap\n", (unsigned)(sizeof cases / sizeof cases[0]));
    return 0;
}
