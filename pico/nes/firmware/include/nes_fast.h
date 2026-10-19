/* nes_fast.h -- the mapper engine's two halves, fast enough for the bus.
 *
 * On the Rev1 cart a mapper write lands on RP-A's core1 between two CPU bus
 * cycles: M2 falls, and 209 ns later the next fetch (usually an opcode, often
 * from the bank just switched) is due. nesmap.c is the reference engine and
 * is far too slow for that (a dispatch, a wrap and a dirty flag per slot).
 * These are the same Tier-1 register semantics written as straight-line code
 * that writes slot pointers directly:
 *
 *   fastprg_t   RP-A: the four 8K PRG slots and the WRAM gates
 *   fastchr_t   RP-B: the eight 1K CHR slots and the mirroring mode
 *
 * Each starts from a nesmap_t that has just been initialised (so the power-on
 * state is nesmap's by construction) and must then agree with nesmap on every
 * write: host_test/test_fast.c fuzzes both against it. Power-of-two images
 * only (every licensed size is): wrapping is a mask. fastprg_init and
 * fastchr_init refuse anything else and the caller falls back to nesmap.
 *
 * Hardware-free: the cart, the bench and the host tests all include it.
 */
#ifndef NES_FAST_H
#define NES_FAST_H

#include <stdbool.h>
#include <stdint.h>

#include "nesmap.h"

/* ------------------------------------------------------------------ PRG */

typedef struct {
    const uint8_t *slot_mem[4];      /* $8000, $A000, $C000, $E000, unless  */
                                     /* fastprg_mirror moved them           */
    /* Where the slots live: slot_mem, or the caller's registers (QMI ATRANS0-3,
     * with bank[] holding ATRANS values), so that a write lands in them
     * directly. Volatile: something else reads them as they change. */
    const uint8_t *volatile *slot;
    bool wram_en, wram_wp;           /* $6000-$7FFF answers / takes writes  */
    const uint8_t *const *bank;      /* bank[n]: where 8K PRG bank n lives  */
    uint32_t mask;                   /* PRG 8K banks - 1                    */
    uint16_t mapper;
    bool has_wram, surom, bnrom;
    uint8_t reg[4], shift, shift_n;  /* MMC1                                */
    uint8_t bank_sel, bank6, bank7;  /* MMC3 / Namcot 108                   */
    /* MMC1: the PRG slots for every (mode, SUROM half, bank) -- 128 rows,
     * built by fastprg_init into memory the caller gives it -- so that a
     * register write only copies four pointers. */
    const uint8_t *(*mmc1_tab)[4];
    /* MMC1, worked out ahead (fastprg_mmc1_prepare) so that a write's M2-high
     * half is a few loads: the row the slots hold, the row after a reset
     * write, and after a fifth write to register r with data bit 0 = b
     * (m1_cur unless four writes are in). Stale after a state change; the
     * early half prepares it itself if nobody did. */
    const uint8_t *const *m1_cur;
    const uint8_t *const *m1_reset;
    const uint8_t *const *m1_next[4][2];
    bool m1_stale;
} fastprg_t;

/* The table fastprg_init builds for MMC1 images: 128 rows of four. */
#define FASTPRG_MMC1_ROWS 128

#define FASTPRG_B(f, n) ((f)->bank[(uint32_t)(n) & (f)->mask])

/* The helpers read the bank table and mask into locals and store the slots
 * last: a slot store could alias f->bank or f->mask as far as the compiler
 * knows, and reloading them after every store doubles the write path. */
static inline void fastprg_16(fastprg_t *f, unsigned half, uint32_t b)
{
    const uint8_t *const *bank = f->bank;
    const uint32_t m = f->mask;
    const uint8_t *lo = bank[(b * 2) & m], *hi = bank[(b * 2 + 1) & m];

    f->slot[half * 2] = lo;
    f->slot[half * 2 + 1] = hi;
}

static inline void fastprg_32(fastprg_t *f, uint32_t b)
{
    const uint8_t *const *bank = f->bank;
    const uint32_t m = f->mask, n = b * 4;
    const uint8_t *s0 = bank[n & m], *s1 = bank[(n + 1) & m];
    const uint8_t *s2 = bank[(n + 2) & m], *s3 = bank[(n + 3) & m];

    f->slot[0] = s0;
    f->slot[1] = s1;
    f->slot[2] = s2;
    f->slot[3] = s3;
}

/* MMC1's PRG slots for one (mode, SUROM half, bank), into out[]. */
static inline void fastprg_mmc1_eval(const fastprg_t *f, unsigned mode, unsigned hi16,
                                     unsigned prg, const uint8_t **out)
{
    const uint8_t *const *bank = f->bank;
    const uint32_t m = f->mask;
    uint32_t b0, b1;

    switch (mode) {
    case 0: case 1:                              /* 32K, low bit ignored     */
        b0 = (hi16 + (prg & 0x0E)) / 2 * 4;
        out[0] = bank[b0 & m];
        out[1] = bank[(b0 + 1) & m];
        out[2] = bank[(b0 + 2) & m];
        out[3] = bank[(b0 + 3) & m];
        return;
    case 2:                                      /* fix first, switch $C000 */
        b0 = hi16 * 2;
        b1 = (hi16 + prg) * 2;
        break;
    default:                                     /* switch $8000, fix last  */
        b0 = (hi16 + prg) * 2;
        b1 = (hi16 + 15) * 2;
        break;
    }
    out[0] = bank[b0 & m];
    out[1] = bank[(b0 + 1) & m];
    out[2] = bank[b1 & m];
    out[3] = bank[(b1 + 1) & m];
}

/* An MMC1 write in pieces, so that what follows M2's fall is a compare and,
 * now and then, a copy of four slots:
 * - fastprg_mmc1_early, while M2 is high, from the address alone (the data
 *   is not valid yet): the rows the write can end in. Changes nothing.
 * - fastprg_mmc1_guess (optional), also while M2 is high, once the write is
 *   known to be a ROM write and while nothing reads the slots: puts in the
 *   likeliest row (no reset; if one of the two rows is the current one,
 *   that, so a SUROM half write that keeps its half moves nothing).
 * - fastprg_mmc1_commit, once M2 has fallen, with the data: the right row,
 *   if the slots do not already hold it.
 * - fastprg_mmc1_late: the shift register and registers.
 * early + commit + late is fastprg_write_m's MMC1 case. */
typedef struct {
    const uint8_t *const *row[2];    /* the row after the write, data bit 0 */
                                     /* clear / set (no reset)              */
    const uint8_t *const *reset;     /* ... after a reset write             */
    const uint8_t *const *now;       /* the row the slots hold              */
} fastprg_mmc1_plan_t;

static inline __attribute__((always_inline)) void fastprg_mmc1_put(fastprg_t *f, const uint8_t *const *t)
{
    const uint8_t *s0 = t[0], *s1 = t[1], *s2 = t[2], *s3 = t[3];

    f->slot[0] = s0;
    f->slot[1] = s1;
    f->slot[2] = s2;
    f->slot[3] = s3;
}

static inline void fastprg_mmc1_prepare(fastprg_t *f)
{
    const uint8_t *(*tab)[4] = f->mmc1_tab;
    const unsigned row = ((f->reg[0] >> 2) & 3u) << 5 |
                         (f->surom ? ((f->reg[1] >> 4) & 1u) : 0u) << 4 |
                         (f->reg[3] & 0x0Fu);
    const unsigned v = (unsigned)f->shift >> 1;  /* a fifth write's bits 0-3 */
    const uint8_t *const *cur = tab[row];

    f->m1_cur = cur;
    f->m1_reset = tab[row | 0x60];               /* PRG mode 3               */
    f->m1_stale = false;
    if (f->shift_n != 4)
        return;                                  /* m1_next unused till then */
    /* control: the PRG mode; CHR0: SUROM's 256K half; CHR1: nothing;
     * PRG: the bank */
    f->m1_next[0][0] = f->m1_next[0][1] = tab[(row & 0x1Fu) | ((v >> 2) & 3u) << 5];
    if (f->surom) {
        f->m1_next[1][0] = tab[row & ~0x10u];
        f->m1_next[1][1] = tab[row | 0x10u];
    } else {
        f->m1_next[1][0] = f->m1_next[1][1] = cur;
    }
    f->m1_next[2][0] = f->m1_next[2][1] = cur;
    f->m1_next[3][0] = f->m1_next[3][1] = tab[(row & 0x70u) | (v & 0x0Fu)];
}

static inline __attribute__((always_inline)) void
fastprg_mmc1_early(fastprg_t *f, uint16_t a, bool consec, fastprg_mmc1_plan_t *pl)
{
    const unsigned r = (a >> 13) & 3;

    if (f->m1_stale)
        fastprg_mmc1_prepare(f);
    pl->now = f->m1_cur;
    if (consec) {                                /* ignored: nothing moves   */
        pl->row[0] = pl->row[1] = pl->reset = pl->now;
        return;
    }
    pl->reset = f->m1_reset;
    if (f->shift_n != 4) {                       /* not a fifth write        */
        pl->row[0] = pl->row[1] = pl->now;
        return;
    }
    pl->row[0] = f->m1_next[r][0];
    pl->row[1] = f->m1_next[r][1];
}

static inline __attribute__((always_inline)) void fastprg_mmc1_guess(fastprg_t *f, fastprg_mmc1_plan_t *pl)
{
    const uint8_t *const *t = pl->row[0];

    if (t == pl->now || pl->row[1] == pl->now)
        return;                      /* likeliest: nothing moves (SUROM half) */
    fastprg_mmc1_put(f, t);
    pl->now = t;
}

static inline __attribute__((always_inline)) void
fastprg_mmc1_commit(fastprg_t *f, uint8_t d, const fastprg_mmc1_plan_t *pl)
{
    const uint8_t *const *t = (d & 0x80) ? pl->reset : pl->row[d & 1];

    if (t != pl->now)
        fastprg_mmc1_put(f, t);
}

static inline __attribute__((always_inline)) void
fastprg_mmc1_late(fastprg_t *f, uint16_t a, uint8_t d, bool consec)
{
    if (consec)
        return;
    if (d & 0x80) {
        f->shift = 0;
        f->shift_n = 0;
        f->reg[0] |= 0x0C;
        f->m1_stale = true;
        return;
    }
    if (f->shift_n != 4) {
        f->shift = (uint8_t)((f->shift >> 1) | ((d & 1) << 4));
        if (++f->shift_n == 4)
            f->m1_stale = true;      /* the fifth write's rows from here     */
        return;
    }
    f->reg[(a >> 13) & 3] = (uint8_t)((f->shift >> 1) | ((d & 1) << 4));
    f->shift = 0;
    f->shift_n = 0;
    f->m1_stale = true;
    if (((a >> 13) & 3) == 3)
        f->wram_en = f->has_wram && !(d & 1);
}

/* MMC3 in the same pieces: early from the address (which register, and for
 * R6/R7 the slot the bank lands in), commit with the data (one table load and
 * one store, or the $8000/$C000 swap), late the registers. */
enum { FASTPRG_M3_NONE, FASTPRG_M3_SEL, FASTPRG_M3_BANK, FASTPRG_M3_GATE };

typedef struct {
    const uint8_t *volatile *dst;    /* FASTPRG_M3_BANK: the slot           */
    const uint8_t *const *bank;
    uint32_t mask;                   /* PRG 8K banks - 1, within 6 bits     */
    uint8_t kind, r;                 /* r: 6 or 7 for a bank write          */
} fastprg_mmc3_plan_t;

static inline __attribute__((always_inline)) void
fastprg_mmc3_early(fastprg_t *f, uint16_t a, fastprg_mmc3_plan_t *pl)
{
    const uint8_t sel = f->bank_sel;

    pl->kind = FASTPRG_M3_NONE;
    pl->dst = NULL;
    pl->r = 0;
    pl->bank = f->bank;
    pl->mask = f->mask & 0x3F;
    switch (a & 0xE001) {
    case 0x8000:
        pl->kind = FASTPRG_M3_SEL;
        break;
    case 0x8001:
        pl->r = sel & 7;
        if (pl->r == 6) {
            pl->kind = FASTPRG_M3_BANK;
            pl->dst = &f->slot[(sel & 0x40) ? 2 : 0];
        } else if (pl->r == 7) {
            pl->kind = FASTPRG_M3_BANK;
            pl->dst = &f->slot[1];
        }
        break;
    case 0xA001:
        pl->kind = FASTPRG_M3_GATE;
        break;
    default:
        break;
    }
}

static inline __attribute__((always_inline)) void
fastprg_mmc3_commit(fastprg_t *f, uint8_t d, const fastprg_mmc3_plan_t *pl)
{
    if (pl->kind == FASTPRG_M3_BANK) {
        *pl->dst = pl->bank[d & pl->mask];
    } else if (pl->kind == FASTPRG_M3_SEL && ((f->bank_sel ^ d) & 0x40)) {
        const uint8_t *t = f->slot[0];
        f->slot[0] = f->slot[2];
        f->slot[2] = t;
    }
}

static inline __attribute__((always_inline)) void
fastprg_mmc3_late(fastprg_t *f, uint8_t d, const fastprg_mmc3_plan_t *pl)
{
    switch (pl->kind) {
    case FASTPRG_M3_SEL:
        f->bank_sel = d;
        break;
    case FASTPRG_M3_BANK:
        if (pl->r == 6)
            f->bank6 = d & 0x3F;
        else
            f->bank7 = d & 0x3F;
        break;
    case FASTPRG_M3_GATE:
        f->wram_en = f->has_wram && (d & 0x80);
        f->wram_wp = (d & 0x40) != 0;
        break;
    default:
        break;
    }
}

static inline void fastprg_mmc3(fastprg_t *f, bool namcot)
{
    const uint8_t *const *bank = f->bank;
    const uint32_t m = f->mask;                  /* banks - 1                */
    const uint8_t *r6 = bank[f->bank6 & m], *r7 = bank[f->bank7 & m];
    const uint8_t *l1 = bank[(m - 1) & m], *l0 = bank[m];

    if (!namcot && (f->bank_sel & 0x40)) {
        f->slot[0] = l1;
        f->slot[2] = r6;
    } else {
        f->slot[0] = r6;
        f->slot[2] = l1;
    }
    f->slot[1] = r7;
    f->slot[3] = l0;
}

/* One console write at or above the mapper's write_lo. `consec` is true
 * when the previous write was on the cycle before (MMC1 ignores it).
 * fastprg_write_m takes the mapper as an argument: called with a constant
 * from a loop specialised per mapper, the dispatch folds away. */
static inline __attribute__((always_inline)) void
fastprg_write_m(fastprg_t *f, unsigned mapper, uint16_t a, uint8_t d, bool consec)
{
    switch (mapper) {
    case 1: {
        fastprg_mmc1_plan_t pl;

        fastprg_mmc1_early(f, a, consec, &pl);
        fastprg_mmc1_commit(f, d, &pl);
        fastprg_mmc1_late(f, a, d, consec);
        return;
    }
    case 2:
        fastprg_16(f, 0, d);
        return;
    case 4: {
        /* Only what the write changes: a new PRG mode swaps $8000 and
         * $C000; R6 and R7 are one slot each. */
        fastprg_mmc3_plan_t pl;

        fastprg_mmc3_early(f, a, &pl);
        fastprg_mmc3_commit(f, d, &pl);
        fastprg_mmc3_late(f, d, &pl);
        return;
    }
    case 7:
    case 11:
        fastprg_32(f, d & 0x0F);
        return;
    case 30:
        fastprg_16(f, 0, d & 0x1F);
        return;
    case 34:
        if (f->bnrom) {
            if (a >= 0x8000)
                fastprg_32(f, d);
        } else if (a == 0x7FFD) {
            fastprg_32(f, d & 1);
        }
        return;
    case 66:
        fastprg_32(f, (d >> 4) & 3);
        return;
    case 71:
        if (a >= 0xC000)
            fastprg_16(f, 0, d & 0x0F);
        return;
    case 206:
        if ((a & 0x8001) == 0x8000) {
            f->bank_sel = d & 7;
        } else if ((a & 0x8001) == 0x8001) {
            switch (f->bank_sel & 7) {
            case 6: f->bank6 = d & 0x0F; f->slot[0] = FASTPRG_B(f, d & 0x0F); break;
            case 7: f->bank7 = d & 0x0F; f->slot[1] = FASTPRG_B(f, d & 0x0F); break;
            default: break;
            }
        }
        return;
    default:                                    /* 0 NROM, 3 CNROM          */
        return;
    }
}

static inline void fastprg_write(fastprg_t *f, uint16_t a, uint8_t d, bool consec)
{
    fastprg_write_m(f, f->mapper, a, d, consec);
}

/* Take the state of a freshly initialised (or reset) nesmap. `bank` must have
 * one entry per 8K bank of the image; `mmc1_tab` FASTPRG_MMC1_ROWS rows for
 * an MMC1 image (NULL otherwise). False: not a power-of-two image. */
static inline bool fastprg_init_tab(fastprg_t *f, const nesmap_t *m, const uint8_t *const *bank,
                                    const uint8_t *(*mmc1_tab)[4])
{
    unsigned i;

    if (m->prg8 & (m->prg8 - 1))
        return false;
    f->bank = bank;
    f->mask = m->prg8 - 1;
    f->mapper = m->plan.mapper;
    f->has_wram = m->plan.prg_ram_size != 0;
    f->surom = m->plan.prg_size > 256u * 1024;
    f->bnrom = m->plan.chr_size == 0;
    f->slot = f->slot_mem;
    for (i = 0; i < 4; i++) {
        f->reg[i] = m->reg[i];
        f->slot[i] = FASTPRG_B(f, m->out.prg[i]);
    }
    f->shift = m->shift;
    f->shift_n = m->shift_n;
    f->bank_sel = m->bank_sel;
    f->bank6 = m->bank[6];
    f->bank7 = m->bank[7];
    f->wram_en = m->out.wram_en;
    f->wram_wp = m->out.wram_wp;
    f->mmc1_tab = mmc1_tab;
    if (f->mapper == 1 && mmc1_tab) {
        unsigned row;

        for (row = 0; row < FASTPRG_MMC1_ROWS; row++)
            fastprg_mmc1_eval(f, row >> 5, ((row >> 4) & 1) ? 16 : 0, row & 15, mmc1_tab[row]);
        fastprg_mmc1_prepare(f);
    }
    return true;
}

/* Move the slots to the caller's registers (four words), starting them at the
 * current slots. Writes then land there and nowhere else; slot_mem goes
 * stale. A copy of a fastprg_t keeps pointing at the original's slots. */
static inline void fastprg_mirror(fastprg_t *f, const uint8_t *volatile *dst)
{
    unsigned i;

    for (i = 0; i < 4; i++)
        dst[i] = f->slot[i];
    f->slot = dst;
}

/* For a caller that keeps one table per image anyway. */
static inline bool fastprg_init(fastprg_t *f, const nesmap_t *m, const uint8_t *const *bank)
{
    static const uint8_t *tab[FASTPRG_MMC1_ROWS][4];

    return fastprg_init_tab(f, m, bank, tab);
}

/* ------------------------------------------------------------------ CHR */

typedef struct {
    const uint8_t *slot[8];          /* $0000-$1FFF in 1K slots             */
    uint8_t mirror;                  /* NESMAP_MIR_*                        */
    bool mirror_changed;             /* the caller repatches CIRAM and clears it */
    uint8_t *base;                   /* CHR (ROM image or RAM), 1K banks    */
    uint32_t mask;                   /* CHR 1K banks - 1                    */
    uint16_t mapper;
    uint8_t submapper;
    bool four_wired, four_flag, bnrom;
    uint8_t reg[4], shift, shift_n;  /* MMC1                                */
    uint8_t bank_sel, bank[6];       /* MMC3 / Namcot 108                   */
} fastchr_t;

#define FASTCHR_P(f, n) ((f)->base + (((uint32_t)(n) & (f)->mask) << 10))

static inline void fastchr_n(fastchr_t *f, unsigned first, unsigned count, uint32_t b)
{
    uint32_t n = b * count;
    unsigned i;

    for (i = 0; i < count; i++)
        f->slot[first + i] = FASTCHR_P(f, n + i);
}

static inline void fastchr_mirror(fastchr_t *f, uint8_t mir)
{
    if (f->four_wired)
        return;                                  /* wired four-screen        */
    if (f->mirror != mir) {
        f->mirror = mir;
        f->mirror_changed = true;
    }
}

static inline void fastchr_mmc1(fastchr_t *f)
{
    if (f->reg[0] & 0x10) {
        fastchr_n(f, 0, 4, f->reg[1]);
        fastchr_n(f, 4, 4, f->reg[2]);
    } else {
        fastchr_n(f, 0, 8, f->reg[1] >> 1);
    }
    switch (f->reg[0] & 3) {
    case 0: fastchr_mirror(f, NESMAP_MIR_1LO); break;
    case 1: fastchr_mirror(f, NESMAP_MIR_1HI); break;
    case 2: fastchr_mirror(f, NESMAP_MIR_V); break;
    default: fastchr_mirror(f, NESMAP_MIR_H); break;
    }
}

static inline void fastchr_mmc3(fastchr_t *f, bool namcot)
{
    unsigned base = (!namcot && (f->bank_sel & 0x80)) ? 4 : 0;
    unsigned k = (base + 4) & 7;

    fastchr_n(f, base, 2, f->bank[0] >> 1);
    fastchr_n(f, base + 2, 2, f->bank[1] >> 1);
    f->slot[k] = FASTCHR_P(f, f->bank[2]);
    f->slot[k + 1] = FASTCHR_P(f, f->bank[3]);
    f->slot[k + 2] = FASTCHR_P(f, f->bank[4]);
    f->slot[k + 3] = FASTCHR_P(f, f->bank[5]);
}

static inline __attribute__((always_inline)) void
fastchr_write_m(fastchr_t *f, unsigned mapper, uint16_t a, uint8_t d, bool consec)
{
    switch (mapper) {
    case 1:
        if (consec)
            return;
        if (d & 0x80) {
            f->shift = 0;
            f->shift_n = 0;
            f->reg[0] |= 0x0C;
            fastchr_mmc1(f);
            return;
        }
        f->shift = (uint8_t)((f->shift >> 1) | ((d & 1) << 4));
        if (++f->shift_n == 5) {
            f->reg[(a >> 13) & 3] = f->shift & 0x1F;
            f->shift = 0;
            f->shift_n = 0;
            fastchr_mmc1(f);
        }
        return;
    case 3:
        fastchr_n(f, 0, 8, d);
        return;
    case 4:
        switch (a & 0xE001) {
        case 0x8000:
            f->bank_sel = d;
            fastchr_mmc3(f, false);
            return;
        case 0x8001: {
            unsigned r = f->bank_sel & 7;
            if (r < 6) {
                f->bank[r] = d;
                fastchr_mmc3(f, false);
            }
            return;
        }
        case 0xA000:
            fastchr_mirror(f, (d & 1) ? NESMAP_MIR_H : NESMAP_MIR_V);
            return;
        default:
            return;
        }
    case 7:
        fastchr_mirror(f, (d & 0x10) ? NESMAP_MIR_1HI : NESMAP_MIR_1LO);
        return;
    case 11:
        fastchr_n(f, 0, 8, d >> 4);
        return;
    case 30:
        if (f->four_flag)
            fastchr_mirror(f, (d & 0x80) ? NESMAP_MIR_1HI : NESMAP_MIR_1LO);
        fastchr_n(f, 0, 8, (d >> 5) & 3);
        return;
    case 34:
        if (f->bnrom)
            return;
        if (a == 0x7FFE)
            fastchr_n(f, 0, 4, d & 0x0F);
        else if (a == 0x7FFF)
            fastchr_n(f, 4, 4, d & 0x0F);
        return;
    case 66:
        fastchr_n(f, 0, 8, d & 3);
        return;
    case 71:
        if (a < 0xA000 && (f->submapper == 1 || f->four_flag))
            fastchr_mirror(f, (d & 0x10) ? NESMAP_MIR_1HI : NESMAP_MIR_1LO);
        return;
    case 206:
        if ((a & 0x8001) == 0x8000) {
            f->bank_sel = d & 7;
        } else if ((a & 0x8001) == 0x8001) {
            unsigned r = f->bank_sel & 7;
            if (r < 6) {
                f->bank[r] = d & 0x3F;
                fastchr_mmc3(f, true);
            }
        }
        return;
    default:                                    /* 0 NROM, 2 UxROM          */
        return;
    }
}

static inline void fastchr_write(fastchr_t *f, uint16_t a, uint8_t d, bool consec)
{
    fastchr_write_m(f, f->mapper, a, d, consec);
}

static inline bool fastchr_init(fastchr_t *f, const nesmap_t *m, uint8_t *base)
{
    unsigned i;

    if (m->chr1 & (m->chr1 - 1))
        return false;
    f->base = base;
    f->mask = m->chr1 - 1;
    f->mapper = m->plan.mapper;
    f->submapper = m->plan.submapper;
    f->four_flag = m->plan.four_screen;
    f->four_wired = m->plan.four_screen && m->plan.mirror == NESMAP_MIR_4;
    f->bnrom = m->plan.chr_size == 0;
    for (i = 0; i < 8; i++)
        f->slot[i] = FASTCHR_P(f, m->out.chr[i]);
    for (i = 0; i < 4; i++)
        f->reg[i] = m->reg[i];
    for (i = 0; i < 6; i++)
        f->bank[i] = m->bank[i];
    f->shift = m->shift;
    f->shift_n = m->shift_n;
    f->bank_sel = m->bank_sel;
    f->mirror = m->out.mirror;
    f->mirror_changed = false;
    return true;
}

#endif
