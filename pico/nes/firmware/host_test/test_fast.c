/* test_fast.c -- nes_fast.h against nesmap.c.
 *
 * Rev1 serves PRG from RP-A and CHR from RP-B, each running a fast half of
 * the mapper engine on the same writes. For every Tier-1 mapper, a range of
 * power-of-two sizes and both MMC1 write rhythms, random writes go through
 * nesmap (the reference, itself checked against MAME by test_nesmap) and
 * through both halves; the PRG slots, the WRAM gates, the CHR slots and the
 * mirroring must agree after every write.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nes_fast.h"

static uint32_t rng = 0xFA57C0DEu;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* Fake memory: bank n of PRG "lives" at 0x10000000 + n * 8K, so a slot
 * pointer tells which bank it is. CHR likewise, 1K banks from chr_base. */
static const uint8_t *prg_bank[64];
static uint8_t chr_mem[1];
#define CHR_BASE ((uint8_t *)0x20000000u)

static unsigned failures;

static void check(const char *what, uint16_t mapper, uint32_t prg, uint32_t chr, unsigned step,
                  const nesmap_t *m, const fastprg_t *p, const fastchr_t *c)
{
    int i, bad = 0;

    for (i = 0; i < 4; i++)
        if (p->slot[i] != prg_bank[m->out.prg[i]])
            bad |= 1;
    if (p->wram_en != m->out.wram_en || p->wram_wp != m->out.wram_wp)
        bad |= 2;
    for (i = 0; i < 8; i++)
        if (c->slot[i] != CHR_BASE + ((uint32_t)m->out.chr[i] << 10))
            bad |= 4;
    if (c->mirror != m->out.mirror)
        bad |= 8;
    if (!bad)
        return;
    failures++;
    if (failures <= 10)
        printf("FAIL %s mapper %u prg %uK chr %uK step %u: %s%s%s%s\n", what, mapper, prg / 1024,
               chr / 1024, step, bad & 1 ? "prg " : "", bad & 2 ? "wram " : "",
               bad & 4 ? "chr " : "", bad & 8 ? "mirror" : "");
}

/* A PRG write; for MMC1, half the time the way RP-A's engine does it: the
 * guessed row in during M2 high, then the commit. */
static void prg_write(fastprg_t *p, uint16_t a, uint8_t d, bool consec, uint32_t r)
{
    fastprg_mmc1_plan_t pl;

    if (p->mapper != 1 || !(r & 0x10000000u)) {
        fastprg_write(p, a, d, consec);
        return;
    }
    fastprg_mmc1_early(p, a, consec, &pl);
    fastprg_mmc1_guess(p, &pl);
    fastprg_mmc1_commit(p, d, &pl);
    fastprg_mmc1_late(p, a, d, consec);
}

static void run(uint16_t mapper, uint32_t prg, uint32_t chr_rom, uint32_t chr_ram, bool four,
                uint8_t sub, bool mmc1_bursts)
{
    nesmap_plan_t plan;
    nesmap_t m;
    fastprg_t p;
    fastchr_t c;
    uint32_t cycle = 100, last_w = 0;
    unsigned step;

    memset(&plan, 0, sizeof plan);
    plan.mapper = mapper;
    plan.submapper = sub;
    plan.prg_size = prg;
    plan.chr_size = chr_rom;
    plan.chr_ram_size = chr_ram;
    plan.prg_ram_size = 8192;
    plan.four_screen = four;
    plan.mirror = (four && mapper != 30 && mapper != 71) ? NESMAP_MIR_4 : NESMAP_MIR_V;
    assert(nesmap_init(&m, &plan) == NESMAP_OK);
    static const uint8_t *volatile regs[4];
    assert(fastprg_init(&p, &m, prg_bank));
    assert(fastchr_init(&c, &m, CHR_BASE));
    /* Every other case with the slots moved out, as RP-A moves them into
     * the QMI's ATRANS registers. */
    if (rnd() & 1) {
        fastprg_mirror(&p, regs);
        assert(p.slot == regs && regs[3] == p.slot_mem[3]);
    }
    check("reset", mapper, prg, chr_rom, 0, &m, &p, &c);

    for (step = 1; step <= 20000; step++) {
        uint32_t r = rnd();
        uint16_t a = (uint16_t)(0x8000u | (r & 0x7FFFu));
        uint8_t d = (uint8_t)(r >> 16);
        bool consec;

        if (mapper == 34 && (r >> 31))
            a = (uint16_t)(0x7FFDu + ((r >> 24) % 3));
        if (mapper == 1 && mmc1_bursts && (r & 0x300) == 0)
            d |= 0x80;                       /* resets, now and then */
        else if (mapper == 1 && mmc1_bursts)
            d &= 0x7F;
        cycle += (r >> 28) % 3 + 1;          /* 1 = the next cycle   */
        if (a < m.desc->write_lo)
            continue;
        consec = cycle == last_w + 1;
        last_w = cycle;
        nesmap_write(&m, a, d, cycle);
        prg_write(&p, a, d, consec, r);
        fastchr_write(&c, a, d, consec);

        check("write", mapper, prg, chr_rom, step, &m, &p, &c);
        if (failures > 10)
            return;
    }
}

/* MMC1 as games drive it: five writes to one register (sometimes a reset
 * first), bit by bit, with core1's spare time between them. */
static void run_mmc1_sequences(uint32_t prg, uint32_t chr_rom, uint32_t chr_ram)
{
    nesmap_plan_t plan;
    nesmap_t m;
    fastprg_t p;
    fastchr_t c;
    uint32_t cycle = 100;
    unsigned seq, k;

    memset(&plan, 0, sizeof plan);
    plan.mapper = 1;
    plan.prg_size = prg;
    plan.chr_size = chr_rom;
    plan.chr_ram_size = chr_ram;
    plan.prg_ram_size = 8192;
    plan.mirror = NESMAP_MIR_V;
    assert(nesmap_init(&m, &plan) == NESMAP_OK);
    assert(fastprg_init(&p, &m, prg_bank));
    assert(fastchr_init(&c, &m, CHR_BASE));

    for (seq = 0; seq < 4000; seq++) {
        uint32_t r = rnd();
        uint16_t reg = (uint16_t)(0x8000u | (r & 0x6000u));
        uint8_t v = (uint8_t)(r >> 16);

        if ((r >> 30) == 0) {                   /* a reset first, as many games do */
            cycle += 4;
            nesmap_write(&m, 0x8000, 0x80, cycle);
            prg_write(&p, 0x8000, 0x80, false, r);
            fastchr_write(&c, 0x8000, 0x80, false);
            check("mmc1 reset", 1, prg, chr_rom, seq, &m, &p, &c);
        }
        for (k = 0; k < 5; k++) {
            uint16_t a = (k == 4 && ((r >> 24) & 7) == 0) ? (uint16_t)(0x8000u | ((r >> 8) & 0x6000u)) : reg;
            unsigned idle = 1 + ((r >> (k * 3)) & 7);

            (void)idle;
            cycle += 4;
            nesmap_write(&m, a, (uint8_t)((v >> k) & 1), cycle);
            prg_write(&p, a, (uint8_t)((v >> k) & 1), false, r >> k);
            fastchr_write(&c, a, (uint8_t)((v >> k) & 1), false);
            check("mmc1 seq", 1, prg, chr_rom, seq, &m, &p, &c);
        }
        if (failures > 10)
            return;
    }
}

int main(void)
{
    static const struct { uint16_t mapper; uint32_t prg, chr_rom, chr_ram; bool four; uint8_t sub; } cases[] = {
        {   0,  16, 8, 0, 0, 0 }, {   0,  32, 8, 0, 0, 0 },
        {   1,  32, 0, 8, 0, 0 }, {   1, 128, 128, 0, 0, 0 }, {   1, 256, 0, 8, 0, 0 },
        {   1, 512, 0, 8, 0, 0 },                                  /* SUROM */
        {   2, 128, 0, 8, 0, 0 }, {   2, 256, 0, 8, 0, 0 },
        {   3,  32, 32, 0, 0, 0 }, {   3,  16, 8, 0, 0, 0 },
        {   4, 512, 256, 0, 0, 0 }, {   4, 128, 0, 8, 0, 0 }, {   4, 256, 128, 0, 1, 0 },
        {   7, 256, 0, 8, 0, 0 }, {   7,  32, 0, 8, 0, 0 },
        {  11, 128, 128, 0, 0, 0 },
        {  30, 512, 0, 32, 1, 0 }, {  30, 256, 0, 32, 0, 0 },
        {  34, 128, 0, 8, 0, 0 }, {  34,  64, 64, 0, 0, 0 },       /* BNROM, NINA-001 */
        {  66, 128, 32, 0, 0, 0 },
        {  71, 256, 0, 8, 0, 0 }, {  71, 128, 0, 8, 0, 1 }, {  71, 128, 0, 8, 1, 0 },
        { 206, 128, 64, 0, 0, 0 },
    };
    unsigned i;

    for (i = 0; i < 64; i++)
        prg_bank[i] = (const uint8_t *)(uintptr_t)(0x10000000u + i * 8192u);
    (void)chr_mem;
    for (i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        run(cases[i].mapper, cases[i].prg * 1024u, cases[i].chr_rom * 1024u, cases[i].chr_ram * 1024u,
            cases[i].four, cases[i].sub, false);
        if (cases[i].mapper == 1)
            run(cases[i].mapper, cases[i].prg * 1024u, cases[i].chr_rom * 1024u, cases[i].chr_ram * 1024u,
                cases[i].four, cases[i].sub, true);
    }
    run_mmc1_sequences(256u * 1024, 0, 8192);
    run_mmc1_sequences(512u * 1024, 0, 8192);          /* SUROM */
    run_mmc1_sequences(128u * 1024, 128u * 1024, 0);
    if (failures) {
        printf("test_fast: %u failures\n", failures);
        return 1;
    }
    printf("test_fast: %u cases x 20000 writes: PRG half, CHR half and gates match nesmap\n",
           (unsigned)(sizeof cases / sizeof cases[0]));
    return 0;
}
