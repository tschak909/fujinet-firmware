/* swd_host.c -- a small SWD host for an RP2350 target. See swd_host.h.
 *
 * The host drives SWDIO before a rising SWCLK edge; the target's bits are
 * read before the next one; one turnaround clock whenever the driver
 * changes. ADIv6 addressing: SELECT holds an AP register's address bits
 * 31:4, the request's A[3:2] the rest; a Mem-AP's CSW, TAR and DRW are at
 * +0xD00, +0xD04, +0xD0C.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/syscfg.h"

#include "swd_host.h"

#define DP_DPIDR    0x0u      /* read */
#define DP_ABORT    0x0u      /* write */
#define DP_CTRLSTAT 0x4u
#define DP_SELECT   0x8u
#define DP_RDBUFF   0xCu

#define DHCSR       0xE000EDF0u
#define DCRSR       0xE000EDF4u
#define DCRDR       0xE000EDF8u
#define DHCSR_KEY   0xA05F0000u
#define DHCSR_DEBUGEN  (1u << 0)
#define DHCSR_HALT     (1u << 1)
#define DHCSR_S_REGRDY (1u << 16)
#define DHCSR_S_HALT   (1u << 17)

enum { ACK_OK = 1, ACK_WAIT = 2, ACK_FAULT = 4 };

static uint32_t cur_select = 0xFFFFFFFFu;
uint32_t swd_last_ack;                         /* diagnostics: the last ACK */
static uint32_t mem_ap = SWD_AP_CORE0;

/* ---------------- the wire ---------------- */

#ifdef SWD_HOST_DBGFORCE
/* The chip's own SW-DP: SWCLK and SWDI driven, SWDO observed. SWDI idles
 * high (as the pad's pull-up would) while the target drives. */
static uint32_t dbg = SYSCFG_DBGFORCE_ATTACH_BITS | SYSCFG_DBGFORCE_SWDI_BITS;

void swd_init(unsigned swclk, unsigned swdio)
{
    (void)swclk; (void)swdio;
    dbg = SYSCFG_DBGFORCE_ATTACH_BITS | SYSCFG_DBGFORCE_SWDI_BITS;
    syscfg_hw->dbgforce = dbg;
}

void swd_release(void)
{
    syscfg_hw->dbgforce = SYSCFG_DBGFORCE_RESET;
}

static inline void dly(void)
{
    for (volatile int i = 0; i < 4; i++)
        ;
}

static inline void pin_clk(bool v)
{
    dbg = v ? dbg | SYSCFG_DBGFORCE_SWCLK_BITS : dbg & ~SYSCFG_DBGFORCE_SWCLK_BITS;
    syscfg_hw->dbgforce = dbg;
}

static inline void pin_out(bool v)
{
    dbg = v ? dbg | SYSCFG_DBGFORCE_SWDI_BITS : dbg & ~SYSCFG_DBGFORCE_SWDI_BITS;
    syscfg_hw->dbgforce = dbg;
}

static inline void pin_drive(bool on)
{
    if (!on)
        pin_out(true);
}

static inline bool pin_in(void)
{
    return syscfg_hw->dbgforce & SYSCFG_DBGFORCE_SWDO_BITS;
}
#else
static unsigned p_clk, p_dio;

void swd_init(unsigned swclk, unsigned swdio)
{
    p_clk = swclk;
    p_dio = swdio;
    gpio_init(p_clk);
    gpio_set_dir(p_clk, true);
    gpio_put(p_clk, 0);
    gpio_init(p_dio);
    gpio_pull_up(p_dio);
    gpio_put(p_dio, 1);
    gpio_set_dir(p_dio, true);
}

void swd_release(void)
{
    gpio_set_dir(p_dio, false);
}

static inline void dly(void)
{
    for (volatile int i = 0; i < 12; i++)
        ;
}

static inline void pin_clk(bool v)   { gpio_put(p_clk, v); }
static inline void pin_out(bool v)   { gpio_put(p_dio, v); }
static inline void pin_drive(bool on) { gpio_set_dir(p_dio, on); }
static inline bool pin_in(void)      { return gpio_get(p_dio); }
#endif

static inline void clock(void)
{
    pin_clk(true);
    dly();
    pin_clk(false);
    dly();
}

static void write_bits(uint32_t v, unsigned n)
{
    pin_drive(true);
    while (n--) {
        pin_out(v & 1);
        v >>= 1;
        clock();
    }
}

static uint32_t read_bits(unsigned n)
{
    uint32_t v = 0;
    unsigned i;

    pin_drive(false);
    for (i = 0; i < n; i++) {
        dly();
        v |= (uint32_t)pin_in() << i;
        clock();
    }
    return v;
}

static void turnaround(void)
{
    pin_drive(false);
    clock();
}

static inline unsigned parity32(uint32_t v)
{
    return (unsigned)__builtin_parity(v);
}

/* ---------------- transactions ---------------- */

/* Retries WAIT; anything else not OK is returned. */
static int xfer(bool ap, bool rnw, uint32_t a, uint32_t *data)
{
    int tries;

    for (tries = 0; tries < 100; tries++) {
        unsigned a2 = (a >> 2) & 1, a3 = (a >> 3) & 1;
        unsigned req = 1u | ((unsigned)ap << 1) | ((unsigned)rnw << 2) | (a2 << 3) | (a3 << 4) |
                       ((ap ^ rnw ^ a2 ^ a3) << 5) | (1u << 7);
        unsigned ack;

        write_bits(req, 8);
        turnaround();
        ack = read_bits(3);
        swd_last_ack = ack;
        if (ack == ACK_OK) {
            if (rnw) {
                uint32_t v = read_bits(32);
                unsigned par = read_bits(1);

                turnaround();
                write_bits(0, 8);
                if (par != parity32(v))
                    return -1;
                *data = v;
            } else {
                turnaround();
                write_bits(*data, 32);
                write_bits(parity32(*data), 1);
                write_bits(0, 8);              /* idle: let it complete */
            }
            return ACK_OK;
        }
        turnaround();
        write_bits(0, 8);
        if (ack != ACK_WAIT)
            return (int)ack;
    }
    return ACK_WAIT;
}

static bool dp_read(uint32_t a, uint32_t *v)   { return xfer(false, true, a, v) == ACK_OK; }
static bool dp_write(uint32_t a, uint32_t v)   { return xfer(false, false, a, &v) == ACK_OK; }

static bool select_ap(uint32_t apaddr)
{
    uint32_t sel = apaddr & ~0xFu;

    if (sel == cur_select)
        return true;
    if (!dp_write(DP_SELECT, sel))
        return false;
    cur_select = sel;
    return true;
}

bool swd_ap_write(uint32_t apaddr, uint32_t v)
{
    return select_ap(apaddr) && xfer(true, false, apaddr & 0xC, &v) == ACK_OK;
}

/* AP reads are posted: the value comes with the next read, so RDBUFF. */
bool swd_ap_read(uint32_t apaddr, uint32_t *v)
{
    uint32_t dummy;

    return select_ap(apaddr) && xfer(true, true, apaddr & 0xC, &dummy) == ACK_OK &&
           dp_read(DP_RDBUFF, v);
}

bool swd_connect(uint32_t *dpidr)
{
    /* The 128-bit selection alert 0x19bc0ea2_e3ddafe9_86852d95_6209f392,
     * least significant bit first: the datasheet lists the words high to
     * low, so the last one goes out first (as openocd's swd.h has it). */
    static const uint32_t alert[4] = { 0x6209f392u, 0x86852d95u, 0xe3ddafe9u, 0x19bc0ea2u };
    uint32_t id;
    unsigned i;

    cur_select = 0xFFFFFFFFu;
    write_bits(0xFF, 8);                       /* >= 8 with SWDIO high */
    for (i = 0; i < 4; i++)
        write_bits(alert[i], 32);              /* selection alert, LSB first */
    write_bits(0, 4);
    write_bits(0x1A, 8);                       /* SWD activation code */
    write_bits(0xFFFFFFFFu, 32);               /* line reset: >= 50 high */
    write_bits(0xFFFFFFFFu, 24);
    write_bits(0, 8);
    if (!dp_read(DP_DPIDR, &id) || id == 0 || id == 0xFFFFFFFFu)
        return false;
    if (dpidr)
        *dpidr = id;
    return dp_write(DP_ABORT, 0x1E);           /* clear any sticky error */
}

bool swd_power_up(void)
{
    uint32_t v;
    int n;

    if (!dp_write(DP_CTRLSTAT, (1u << 30) | (1u << 28)))
        return false;
    for (n = 0; n < 1000; n++) {
        if (dp_read(DP_CTRLSTAT, &v) && (v & 0xA0000000u) == 0xA0000000u)
            return true;
        busy_wait_us(100);
    }
    return false;
}

/* ---------------- memory ---------------- */

bool swd_mem_ap(uint32_t ap_base)
{
    uint32_t csw;

    mem_ap = ap_base;
    /* Word accesses, TAR auto-increment, and Secure: CSW bit 30 (HNONSEC)
     * resets set, and Secure SRAM faults a Non-secure access (only the PPB
     * answers both). The rest of CSW as it was. */
    return swd_ap_read(mem_ap + 0xD00, &csw) &&
           swd_ap_write(mem_ap + 0xD00, (csw & ~(0x37u | (1u << 30))) | 0x12u);
}

bool swd_mem_write(uint32_t addr, uint32_t v)
{
    return swd_ap_write(mem_ap + 0xD04, addr) && swd_ap_write(mem_ap + 0xD0C, v);
}

bool swd_mem_read(uint32_t addr, uint32_t *v)
{
    return swd_ap_write(mem_ap + 0xD04, addr) && swd_ap_read(mem_ap + 0xD0C, v);
}

/* TAR auto-increments only within a 1K page. */
bool swd_mem_write_block(uint32_t addr, const uint32_t *w, uint32_t n)
{
    uint32_t i;

    for (i = 0; i < n; i++, addr += 4) {
        if ((i == 0 || !(addr & 0x3FF)) && !swd_ap_write(mem_ap + 0xD04, addr))
            return false;
        if (!swd_ap_write(mem_ap + 0xD0C, w[i]))
            return false;
    }
    return true;
}

bool swd_mem_verify_block(uint32_t addr, const uint32_t *w, uint32_t n)
{
    uint32_t i, v;

    for (i = 0; i < n; i++, addr += 4) {
        if ((i == 0 || !(addr & 0x3FF)) && !swd_ap_write(mem_ap + 0xD04, addr))
            return false;
        if (!swd_ap_read(mem_ap + 0xD0C, &v) || v != w[i])
            return false;
    }
    return true;
}

/* ---------------- the core ---------------- */

bool swd_core_halt(void)
{
    uint32_t v;
    int n;

    if (!swd_mem_write(DHCSR, DHCSR_KEY | DHCSR_HALT | DHCSR_DEBUGEN))
        return false;
    for (n = 0; n < 100; n++)
        if (swd_mem_read(DHCSR, &v) && (v & DHCSR_S_HALT))
            return true;
    return false;
}

bool swd_core_set_reg(unsigned regsel, uint32_t v)
{
    uint32_t s;
    int n;

    if (!swd_mem_write(DCRDR, v) || !swd_mem_write(DCRSR, (1u << 16) | regsel))
        return false;
    for (n = 0; n < 100; n++)
        if (swd_mem_read(DHCSR, &s) && (s & DHCSR_S_REGRDY))
            return true;
    return false;
}

bool swd_core_resume(void)
{
    return swd_mem_write(DHCSR, DHCSR_KEY | DHCSR_DEBUGEN);
}
