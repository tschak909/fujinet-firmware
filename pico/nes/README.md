# FujiNet for the Nintendo Entertainment System

A two-RP2354B cartridge that gives the NES a FujiNet: a memory-mapped mailbox
the console talks to, a USB link to an ESP32-S3 running the `fujiversal-nes`
build of fujinet-firmware, and PRG and CHR served from the RPs' own memory, so
that any image the ESP32 fetches -- a FujiNet application or a commercial
game -- runs on the console. Rev1 has no SRAMs and no glue logic.

This is the seventh cartridge-mailbox bring-up in `pico/` (Odyssey², Astrocade,
Arcadia, ColecoVision, Channel F, Atari 2600 before it) and shares their
protocol sources byte for byte. What is new is the hardware shape, and this
file is mostly about why it is shaped that way.

## Status

| | Milestone | Artifact | State |
|---|---|---|---|
| M5 | Mapper engine, loader ROM | `nesmap.c`, `nes_fast.h`, `loader.s` | host tests green (6/6) |
| M0 | Toolchain, header, font | `hello.nes` | runs in MAME, direct and through the loader ROM |
| M1 | One mailbox round trip + reset survival | `fujitest.nes` | adapter config on screen; `resettest` ACKSEQ 1 -> 2 across a soft reset |
| M2 | Network boot through the loader | `fujiboot.nes` | MOUNT_HOST -> SET_DEVICE_FULLPATH -> MOUNT_IMAGE -> DBC push -> PRG byte-identical to the file; `asteroids.nes` (CHR-RAM) and `wow.nes` (CHR-ROM) play |
| M3 | Directory browser | fujinet-config `nes/` | `cfgtest` browses SD, pages, boots `hello.bin` and a CHR-ROM image through the loader |
| M4 | RP2354B firmware + ESP32 env + CI | `fujines.uf2`, `fujiversal-nes` | both build; core1 in SRAM (`checksram`); the ESP32-S3 image embeds the cart firmware |
| M6 | Soak | synthetic corpus | 19/19: every Tier-1 mapper, PRG and CHR banking checked against the corpus |
| M7 | Real CONFIG + a FujiNet app | fujinet-config `nes/`, 5 Card Stud `src/nes` | CONFIG boots games; 5 Card Stud fetches the live table list and joins a table |
| R1 | Rev1: two RPs, no SRAM | `bench/`, `ppu/`, `nes_engine.c` | Phase 0 timing passed on a Pico 2 (below); RP-A and RP-B firmware build; RP-A's SWD loader proven against a live RP2350 debug port |
| M8 | Hardware | fujinet-hardware `NES/FujiNet-NES-Rev0`, `-Rev1` | Rev0 schematic + routed board; Rev1 to come; no board built |

No hardware exists. M0-M7 were verified in host tests or in a patched MAME
against a live fujinet-pc, never on a console; the Rev1 timing was verified on
a stock Pico 2 that plays both console and cart over its own pads
(`bench/README.md`). The MAME device has the Rev1 install but has not been
rebuilt since.

## The design in one paragraph

The NES has two buses and the community consensus is that one microcontroller
cannot serve both, so a flash cart needs a CPLD or FPGA -- or, as Rev0 did,
SRAMs the console addresses itself with a microcontroller only changing their
high address lines. Rev1 drops the SRAMs ($6.36 each, out of stock) and their
six glue chips for a second RP2354B: **RP-A** owns the CPU bus and **RP-B** the
PPU bus. On RP-A, PRG $8000-$FFFF is served by a PIO + DMA engine: at every M2
rise a PIO state machine pushes the address, a chain of DMA channels fetches
the byte from SRAM (images up to 352K) or from the in-package flash through the
QMI's four address-translation windows (512K images), and a second state
machine drives it if /ROMSEL went low. core1 only handles writes -- the mapper
engine's PRG half moves the slots inline, before the next fetch -- and the
cart's own pages ($5000 mailbox and loader, $6000 WRAM). RP-B serves CHR and
CHR-RAM from its SRAM with a core1 loop and the CIRAM lines from a PIO table,
and runs the CHR half of the same mapper engine on every write RP-A forwards
over an 8-bit link. RP-A loads RP-B over SWD at every boot.

## Verified facts (do not re-derive)

- NTSC 2A03: 559 ns/cycle; M2 high 350 ns, low 209 ns. The address is
  guaranteed stable only while M2 is high; write data is valid at M2's
  falling edge (nesdev *Cartridge connector*). A15 is not on the edge:
  `/ROMSEL = NAND(M2, A15)` through the console's LS139, up to ~38 ns late.
- PPU: 186 ns/dot, every memory access two dots; `/RD` low for the second.
- MMC3 scanline counter: an A12 rise after A12 has been low for three M2
  falling edges (nesdev *MMC3*).
- RP2350 GPIOs are "5 V-tolerant (powered) and 3.3 V-failsafe (unpowered)",
  so the cart sits on the 5 V bus with no level shifters. Pad pulls stay off on
  every bus pin (RP2350-E9).
- Phase 0, at 266 MHz / 1.20 V, 10^7 bus cycles a scenario, 0 errors: M2 rise
  to D0-D7 valid 139-177 ns from SRAM, 211-241 ns from flash; RP-B /RD fall to
  PD valid 94 ns, /RD rise to PD released 41 ns, PPU A10-A13 to CIRAM 30 ns;
  link 10^7 frames clean, write to RP-B slot <= 566 ns. The uncached flash read
  itself is 117 ns (SCK 133 MHz, continuous-read mode).
- Original EverDrive N8: 512K PRG, 512K CHR, 128K PRG-RAM, mappers 0-255
  minus a handful. Kirby's Adventure (512K+256K, MMC3) is the largest
  licensed NES cart; Metal Slader Glory (512K+512K, MMC5) the largest Famicom.

## Pin maps

RP-A (`nes_bus.h`, `nes_cart.h`), 44 of 48:

| GPIO | Signal | Notes |
|---|---|---|
| 0-12 | CPU A0-A12 | in |
| 13-20 | CPU D0-D7 | PIO0 out (the engine), 4 mA; OEOVER-off until PWR_OK |
| 21, 22, 23 | M2, R/W, /ROMSEL | in |
| 24, 25 | CPU A13, A14 | in |
| 26 | PWR_OK | in: console 5 V through 2.2k/3.3k |
| 27 | LED | low-Vf LED, 330R |
| 32 | /IRQ | driven low or released, never high |
| 33 | LACK | in, from RP-B |
| 34-41, 42, 43 | link LD0-LD7, LCLK, LFRM | PIO1 out |
| 44, 45, 46 | RP-B SWCLK, SWDIO, RUN | SWD loader |
| 47 | PPU A12 | in: the MMC3 filter (PIO1) |

RP-B (`nes_ppu.h`), 38 of 48: PPU A0-A13 on 0-13, PD0-PD7 on 14-21, /RD and
/WR on 22-23, PWR_OK on 24, CIRAM A10 and /CE on 25-26 (PIO0 side-set), link
LFRM/LCLK on 33-34, LD7..LD0 on 35-42 (mirrored, so the bus between the RPs is
planar), LACK on 43, the console 5 V on ADC 44.

## Memory map, as the console sees it

```
$4020-$4FFF  open bus
$5000-$53FF  1K reply window
$5400-$54FF  status page (fuji_mailbox.h)
$5500-$55FF  register writes:  STA $5500+n  => register n = A
$5600-$56FF  raw REGDATA (for the shared decoder; clients do not use it)
$5700-$57FF  TX stream: a write anywhere appends
$5800-$5FFF  loader ROM (2K, cart-served, read-only)
$6000-$7FFF  WRAM (8K), cart-served, mapper-gated
$8000-$FFFF  PRG through the engine's 4 x 8K slots; before a game, the boot
             bank, whose $FF00 page is the loader's last page and its vectors
```

**The one client rule:** only `STA/STX/STY` to `$5500-$57FF`, plain or
indexed. Never an RMW instruction: the 6502 writes the old value back first
and it lands as an event. `tools/checkrom.py` rejects the opcodes; the cart
also recognises the exact signature (same address, adjacent cycles), drops
the dummy write and counts it in `FN_R_DIAG_RMW`.

## Loading

Images are staged on the cart (`fuji_store`: 352K of RAM, or 1.5 MB of flash
above the firmware) exactly as on every sibling, and `fujimail.c` is verbatim.
The store keeps the iNES header apart so that the body starts where the engine
can serve it in place: 8K-aligned in RAM, on a 4K sector in flash. Nothing is
copied: the client sets `FN_REG_BOOTLOCK` and jumps to the loader ROM at
`$5800`, which asks for the load (`FN_H_REGSEL + FN_HOT_SWAP`) and polls; the
cart sends CHR to RP-B over the link (or clears its CHR-RAM), points the
engine's banks at the store, tells RP-B to serve and posts `FN_LOAD_DONE`; the
loader acknowledges and jumps through `$FFFC`. Power-on is the same loader
entered through the boot bank's vectors, installing the CONFIG baked into the
firmware.

## Mappers

`nesmap.c` is a table of per-mapper handlers that all produce one
`nesmap_out_t` (four PRG banks, eight CHR banks, mirroring, WRAM and CHR
write gates); `test_nesmap.c` fuzzes it against transcriptions of MAME's own
handlers. On the cart it is too slow for the bus, so `nes_fast.h` has its two
halves as the cart runs them -- `fastprg` on RP-A (the engine's slots), and
`fastchr` on RP-B -- and `test_fast.c` holds both to nesmap on every write.
MMC1 and MMC3 split each write into the part the address decides (while M2 is
high) and the part the data decides (after it falls).

Tier 1, implemented: 0 NROM, 1 MMC1 (with the consecutive-write rule and
SUROM's 512K), 2 UxROM, 3 CNROM, 4 MMC3, 7 AxROM, 11 Color Dreams,
30 UNROM-512, 34 BNROM/NINA-001, 66 GxROM, 71 Camerica/BF9097, 206 Namcot 108.

Tier 2, table entries to come: VRC1-7 (no audio), FME-7 (no audio), N163 (no
audio, no nametable-as-CHR), Bandai, Jaleco, Irem, Taito, RAMBO-1, Sunsoft-3/4,
the discrete-logic boards. Hardware limits, plainly: MMC5's extended
attributes / ExRAM nametables / vertical split and the MMC2/MMC4 tile latches
would need RP-B to decode them at PPU speed; expansion audio has no path on a
72-pin NES; 10NES is an external CIClone.

## Building

```sh
./build.sh                       # loader.bin + hello/fujitest/fujiboot .nes (cc65 2.19)
./build-cart.sh                  # RP-B (ppu/) then RP-A: firmware/build-fujines_rp2354b/fujines.uf2
make -C firmware/host_test       # the six host tests, no SDK
(cd bench && ./build.sh)         # the Phase 0 bench, a Pico 2 build (bench/README.md)
./emu/apply.sh ~/Workspace/mame  # graft the device; then make REGENIE=1
./run.sh fujitest                # MAME, against fujinet-pc's BoIP on 127.0.0.1:9995
./run.sh fujitest resettest      # headless, driven by emu/resettest.lua
./run.sh config cfgtest          # drive CONFIG with the joypad: open SD, boot hello.bin
CFG_FILE=wow.bin CFG_EXPECT=image:$SD/wow.bin ./run.sh config cfgtest   # page, boot, byte-compare PRG and CHR
```

`FUJINET_LOADER=1 ./run.sh hello` sends the `-cart` image through the loader
ROM instead of running it directly, as the cart does at power-on.
`FUJINET_DEBUG=1` logs every transaction; `FUJINET_BOOTDUMP=path` writes PRG
and CHR after a load for a byte compare.

## What a board has to settle

In-band diagnostics first (`FN_R_ERR`, `FN_R_DIAG_RMW` on screen, RP-B's
status block over SWD), instruments last.

1. RP-B loaded and reporting (`rpb_boot()` returns `RPB_OK`).
2. M2 rise to cart data valid at the edge, from SRAM and from flash, and how
   long write data holds after M2 falls (the last-sample-while-high model is
   PROVISIONAL).
3. /RD to PD valid and PD release on a real PPU; CIRAM mirroring per mode.
4. An MMC3 game with 1K banking and a status-bar split, to settle
   `MMC3_A12_FILTER_NS` and /IRQ latency; a 512K game (Kirby's Adventure,
   Dragon Warrior III) from flash.
5. `PWR_OK` with the console off and USB on: D0-D7, /IRQ, PD and CIRAM off.
6. The power-on race with no USB attached: the RP boots in a few ms and the
   2A03 fetches its reset vector early; a CPU that wins reads open bus until
   Reset.

## Known limits

- RP-A runs at 266 MHz at 1.20 V, an overclock (Rev0 ran 200 MHz); the flash
  path needs it.
- The flash path's worst case is an MMC1 SUROM half switch, 241 ns: under the
  250 ns line, over the 230 ns target.
- WRAM is one 8K bank, as on Rev0; banked PRG-RAM (SOROM, SXROM) is not served.
- Nothing may read flash while a flash-served game runs (its windows include
  XIP window 0); the firmware runs from SRAM and the store refuses flash work.
- Four-screen mirroring is RP-B's (nametables in its SRAM); the MAME device
  does not model it yet.
- Battery saves live in the cart's WRAM and are not persisted yet.

## Licence notes

`emu/fujinet.cpp/.h` are BSD-3-Clause for MAME. The firmware and clients are
under the repository licence. The design references the EverDrive N8 only for
its memory architecture (nesdev wiki); nothing is copied from it.
