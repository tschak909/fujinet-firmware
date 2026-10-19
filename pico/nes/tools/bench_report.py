#!/usr/bin/env python3
"""bench_report.py -- drive the NES Rev1 timing bench and judge it.

Runs a command on the bench (a Pico 2 flashed with bench/build-pico2/
nesbench.uf2), collects its "R ..." lines, and prints PASS / MARGINAL / FAIL
against the Phase-0 thresholds. Exit status 0 only when every gating row
passes at one clock or more.

  tools/bench_report.py                      # "run": 200 and 250 MHz
  tools/bench_report.py --cmd quick          # a tenth of the cycles
  tools/bench_report.py --cmd "run 200"
  tools/bench_report.py --parse bench.log    # judge a saved log offline
  tools/bench_report.py --md report.md       # also write a Markdown report
"""

import argparse
import glob
import sys
import time

# (row, threshold for PASS, threshold for MARGINAL); values in ns.
LIMITS = {
    'prg_ram': (150, 200),       # M2 rise -> D0-D7 valid, PRG from RAM
    'prg_xip': (230, 250),       # ... from uncached XIP (2A03 wants ~50 ns setup of 350)
    'ppu_data': (110, 126),      # /RD fall -> PD valid (PPU latches at /RD rise, 186 ns)
    'ppu_release': (40, 60),     # /RD rise -> PD released
    'ciram': (40, 40),           # PA10/11/13 -> CIRAM A10 / CE
    'link': (1000, 1000),        # write's M2 fall -> RP-B slot updated
}
M2_FALL_TO_PUSH_NS = 40          # core1's decode before the frame starts (dut_cpu.c)


def parse_line(line):
    """'R cpu ram_mmc3 mhz=200 a=1' -> (['cpu', 'ram_mmc3'], {'mhz': '200', 'a': '1'})"""
    words, kv = [], {}
    for tok in line.split()[1:]:
        if '=' in tok:
            k, v = tok.split('=', 1)
            kv[k] = v
        else:
            words.append(tok)
    return words, kv


def num(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def grade(value, limits):
    ok, marginal = limits
    if value is None:
        return 'FAIL'
    if value <= ok:
        return 'PASS'
    if value <= marginal:
        return 'MARGINAL'
    return 'FAIL'


def judge(lines):
    """Return rows: (mhz, item, value, verdict, gating, note)."""
    rows = []
    engine, last_mhz = 'cpu', None
    for line in lines:
        if not line.startswith('R '):
            continue
        words, kv = parse_line(line)
        if not words:
            continue
        mhz = num(kv.get('mhz'))
        if mhz:
            last_mhz = mhz
        kind = words[0]
        if kind == 'cpu_cal':
            engine = kv.get('engine', engine)
        elif kind == 'cpu_dbg' and len(words) > 1:
            # core1's write path must be back at its data sampling before M2
            # falls: on a console the write data is only valid late in M2 high
            # (the bench carries it on A1-A8, valid all cycle, so only this
            # counter shows the miss)
            past = num(kv.get('guess_past_fall'))
            if past:
                rows.append((last_mhz, 'cpu %s [%s] write data' % (words[1], engine), past, 'FAIL',
                             True, 'writes whose M2-high work ran past M2 fall'))
        elif kind == 'cpu' and len(words) > 1:
            scn = words[1]
            stress = kv.get('stress') == '1'
            tmin = num(kv.get('tmin_ns'))
            errs = num(kv.get('errors')) or 0
            under = num(kv.get('underrun')) or 0
            lim = LIMITS['prg_ram'] if scn.startswith('ram_') else LIMITS['prg_xip']
            v = grade(tmin, lim)
            if errs or under:
                v = 'FAIL'
            gating = not stress and not scn.endswith('_cd1')
            note = 'late=%s drive=%sns wapply=%sns' % (kv.get('late'), kv.get('max_drive_ns'),
                                                        kv.get('max_wapply_ns'))
            if under:
                note += ' UNDERRUN (bench, not the cart: rerun)'
            rows.append((mhz, 'cpu %s%s [%s]' % (scn, ' +stress' if stress else '', engine), tmin, v,
                         gating, note))
        elif kind == 'ppu' and len(words) > 1 and words[1] == 'read':
            mode = kv.get('mode')
            d, c = num(kv.get('rd_to_data_ns')), num(kv.get('ciram_ns'))
            vd = grade(d, LIMITS['ppu_data'])
            vc = grade(c, LIMITS['ciram'])
            if num(kv.get('underrun')):
                vd = vc = 'FAIL'
            rows.append((mhz, 'ppu /RD->PD mode=%s' % mode, d, vd, True, ''))
            rows.append((mhz, 'ppu CIRAM mode=%s' % mode, c, vc, True, ''))
        elif kind == 'ppu' and len(words) > 2 and words[1] == 'write':
            werr = num(kv.get('werr'))
            hold = words[2] == 'hold'
            v = 'PASS' if werr == 0 and not num(kv.get('underrun')) else 'FAIL'
            rows.append((mhz, 'ppu CHR-RAM write %s' % words[2], werr, v, hold,
                         'errors in %s writes%s' % (kv.get('writes'),
                                                    '' if hold else ' (zero-hold race, information)')))
        elif kind == 'ppu' and len(words) > 1 and words[1] == 'release':
            m = num(kv.get('max_ns'))
            rows.append((mhz, 'ppu release', m, grade(m, LIMITS['ppu_release']), True,
                         'avg %sns, %s samples' % (kv.get('avg_ns'), kv.get('samples'))))
        elif kind == 'link' and len(words) > 1 and words[1] == 'integrity':
            e, lost = num(kv.get('errors')), num(kv.get('lost'))
            v = 'PASS' if e == 0 and lost == 0 else 'FAIL'
            rows.append((mhz, 'link integrity div=%s' % kv.get('tx_div'), e, v, True,
                         '%s frames, %s lost' % (kv.get('frames'), lost)))
        elif kind == 'link' and len(words) > 1 and words[1] == 'latency':
            m = num(kv.get('max_ns'))
            total = None if m is None else m + M2_FALL_TO_PUSH_NS
            v = grade(total, LIMITS['link'])
            if num(kv.get('lockstep_mismatch')) or num(kv.get('lost')):
                v = 'FAIL'
            rows.append((mhz, 'link M2 fall->slot', total, v, True,
                         'avg %sns + %dns decode; lockstep mismatch %s' %
                         (kv.get('avg_ns'), M2_FALL_TO_PUSH_NS, kv.get('lockstep_mismatch'))))
        elif kind == 'qmi_tune' and kv.get('result') == 'fail':
            rows.append((mhz, 'qmi tune', None, 'FAIL', True, 'no RX delay reads the image back'))
        elif kind in ('xip', 'map'):
            pass  # information; printed raw below
    return rows


def summary(lines, rows):
    out = []
    clocks = sorted({r[0] for r in rows if r[0]})
    out.append('| MHz | Item | ns / count | Verdict | Gating | Note |')
    out.append('|---|---|---|---|---|---|')
    for r in rows:
        out.append('| %s | %s | %s | %s | %s | %s |' % (r[0], r[1], '-' if r[2] is None else r[2],
                                                         r[3], 'yes' if r[4] else 'info', r[5]))
    out.append('')
    info = [l for l in lines if l.startswith('R xip ') or l.startswith('R map ') or
            l.startswith('R qmi') or l.startswith('R cpu_cal') or l.startswith('R ppu_cal')]
    if info:
        out.append('Information:')
        out.append('```')
        out.extend(info)
        out.append('```')
        out.append('')
    verdicts = {}
    for mhz in clocks:
        gating = [r for r in rows if r[0] == mhz and r[4]]
        if any(r[3] == 'FAIL' for r in gating):
            verdicts[mhz] = 'FAIL'
        elif any(r[3] == 'MARGINAL' for r in gating):
            verdicts[mhz] = 'MARGINAL'
        else:
            verdicts[mhz] = 'PASS' if gating else 'NO DATA'
        out.append('**%d MHz: %s**' % (mhz, verdicts[mhz]))
    out.append('')
    if any(v == 'PASS' for v in verdicts.values()):
        best = min(m for m, v in verdicts.items() if v == 'PASS')
        out.append('Phase 0 passes: Rev1 runs both RPs at %d MHz.' % best)
    else:
        out.append('Phase 0 does not pass. Per the plan: try 266 MHz (`run 266`); then the '
                   'speculative fetch during M2 low; otherwise the PRG cap (384K) vs one PRG '
                   'SRAM decision goes to the user.')
    return '\n'.join(out), any(v == 'PASS' for v in verdicts.values())


def find_port():
    ports = sorted(glob.glob('/dev/serial/by-id/*Raspberry_Pi*') + glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else None


def run_bench(port, cmd, log):
    import serial  # pyserial
    lines = []
    with serial.Serial(port, 115200, timeout=1) as s:
        s.dtr = True
        # The bench greets once, when the port first opens; keep what it said
        # (the flash-image check among it) and wait for "R ready" if it is new.
        t_end = time.time() + 1.0
        while time.time() < t_end:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode(errors='replace').strip()
            if line.startswith('R '):
                print(line, flush=True)
                lines.append(line)
                if log:
                    log.write(line + '\n')
                if line != 'R ready':
                    t_end = time.time() + 30.0      # still writing the image
                else:
                    break
        s.write((cmd + '\n').encode())
        idle_since = time.time()
        while True:
            raw = s.readline()
            if not raw:
                if time.time() - idle_since > 600:
                    print('bench_report: no output for 10 minutes; giving up', file=sys.stderr)
                    break
                continue
            idle_since = time.time()
            line = raw.decode(errors='replace').strip()
            if not line:
                continue
            print(line, flush=True)
            lines.append(line)
            if log:
                log.write(line + '\n')
                log.flush()
            if line == 'R done':
                break
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--port', help='serial port (default: the first Pico found)')
    ap.add_argument('--cmd', default='run', help='bench command (default: run)')
    ap.add_argument('--parse', help='judge a saved log instead of running the bench')
    ap.add_argument('--log', default='bench.log', help='where to save the raw lines (default bench.log)')
    ap.add_argument('--md', help='also write the summary as Markdown here')
    a = ap.parse_args()

    if a.parse:
        with open(a.parse) as f:
            lines = [l.strip() for l in f if l.strip()]
    else:
        port = a.port or find_port()
        if not port:
            sys.exit('bench_report: no Pico found; plug in the Pico 2 running nesbench.uf2')
        with open(a.log, 'w') as log:
            lines = run_bench(port, a.cmd, log)
        print('bench_report: raw lines saved to %s' % a.log)

    rows = judge(lines)
    text, ok = summary(lines, rows)
    print()
    print(text)
    if a.md:
        with open(a.md, 'w') as f:
            f.write('# NES Rev1 Phase-0 bench report\n\n')
            f.write(text + '\n')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
