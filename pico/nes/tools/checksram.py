#!/usr/bin/env python3
"""checksram.py -- core1 and everything it calls must live in SRAM, and the
only data left in flash is what core0 reads at install time.

The firmware runs copy_to_ram, so this should never fail; it is here so a
CMake change that drops that (or a stray section attribute) is caught at
build time rather than as a missed bus cycle. The flash list matters more in
Rev1: a 512K game takes every XIP window (nes_engine.h), so nothing may read
flash while it runs, and anything there must be install-time only.

Usage: checksram.py fujines.elf
"""

import re
import subprocess
import sys

SRAM = 0x20000000
FLASH = 0x10000000
REQUIRED = ("nes_core1_main", "nes_engine_core1", "nes_engine_note", "fuji_cart_note")
INLINED_OK = ("fuji_cart_note",)
# Data allowed in flash: read by core0 only, before a game's engine starts.
FLASH_OK = ("_configrom_chr", "_rpb_image")


def main():
    elf = sys.argv[1]
    out = subprocess.run(["arm-none-eabi-nm", "-S", elf], capture_output=True,
                         text=True, check=True).stdout
    syms = {}
    flash_data = []
    for line in out.splitlines():
        m = re.match(r"^([0-9a-fA-F]+)\s+(?:[0-9a-fA-F]+\s+)?(\w)\s+(\S+)$", line)
        if not m:
            continue
        addr, kind, name = int(m.group(1), 16), m.group(2), m.group(3)
        syms[name] = addr
        if (FLASH <= addr < SRAM and kind in "dDrRbB"
                and not name.startswith(("__", "embedded_"))):   # boot metadata
            flash_data.append(name)
    bad = []
    for name in REQUIRED:
        if name not in syms:
            if name in INLINED_OK:
                continue                     # inlined away: fine
            bad.append("%s is not in the image at all" % name)
        elif syms[name] < SRAM:
            bad.append("%s is at %#010x -- in FLASH, not SRAM" % (name, syms[name]))
    for name in flash_data:
        if name not in FLASH_OK:
            bad.append("%s is data in FLASH: only install-time data may be (FLASH_OK)" % name)
    for p in bad:
        print("checksram: %s" % p, file=sys.stderr)
    if bad:
        return 1
    print("checksram: core1 is SRAM-resident (%s); flash data: %s"
          % (", ".join("%s@%#x" % (n, syms[n]) for n in REQUIRED if n in syms),
             ", ".join(flash_data) or "none"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
