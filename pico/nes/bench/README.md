# NES Rev1 timing bench (Phase 0)

Rev1 of the NES cart drops both SRAMs: RP-A serves PRG itself (from RAM, or
from its in-package flash for a 512K image) and a second RP2354B, RP-B,
serves the PPU bus. This bench proves the timing on a stock **Pico 2** before
any board is drawn. One RP2350 plays the console and the cart at once: PIO
state machines drive the 2A03 and 2C02 waveforms onto the cart's own pins,
core1 runs the Rev1 bus loops (`src/dut_cpu.c`, `src/dut_ppu.c`), and other
state machines sample the cart's answers. It all loops back inside the
chip's pads: **one USB cable, no wiring, no instruments.**

## Run it

```sh
./build.sh                                  # build-pico2/nesbench.uf2
# hold BOOTSEL, plug the Pico 2 in, copy nesbench.uf2 to the RP2350 drive
../tools/bench_report.py --md report.md     # ~7 minutes: 200 then 250 MHz
```

After the first time, `picotool load -f build-pico2/nesbench.uf2 && picotool
reboot` reflashes it without BOOTSEL. `bench_report.py --cmd quick` runs a
tenth of the cycles; `--cmd "run 266"` tries 266 MHz; `--parse bench.log`
judges a saved log again. The first start writes a 512K test image at 1 MB
into the Pico 2's flash (a few seconds, once).

The CPU rows run the cart's own engine, `firmware/src/nes_engine.c` and
`nes_engine.pio` (built here with `NES_ENGINE_BENCH_DATA`: bench writes carry
their data on A1-A8). `--cmd "engine dma|atrans"` picks its PRG path, then
`run` as above:

- `dma`: `nes_capture`, four DMA hops through RP-A's slot table. PRG from
  RAM, or from flash for the `xip_`/`hyb_` rows (too slow: 256 ns).
- `atrans`: the path for images in flash. The four PRG slots are the
  QMI's address-translation windows (ATRANS0-3), so `nes_capture_x` pushes
  the final address and one DMA hop starts the read. Runs the `xip_` rows
  only. The windows include window 0, so nothing else may read flash
  meanwhile; a fault reboots the bench rather than taking USB down.

(The first design, core1 answering every read itself, lost Phase 0 at
282-320 ns and is gone.)

`--cmd swd` runs RP-A's SWD host (`firmware/src/swd_host.c`, the RP-B
loader's) against this chip's own debug port through SYSCFG DBGFORCE: it
connects, powers up, reads and writes 1K words through core1's AHB-AP, halts
core1, loads a four-instruction program, points SP/PC at it and resumes;
`run=1` means the program ran. Only RUN and the rescue reset are left out.

`--cmd apb` times QMI register traffic (the ATRANS stores). Each `cpu`
row is followed by `cpu_dbg`: M2 fall to the engine's release after a
write, and `guess_past_fall`, writes whose M2-high work ran past M2 falling.
That must be 0: on a console the write data is valid only late in M2 high.

Leave pins GP0-GP27 unconnected while it runs (GP23-25 are the Pico 2's own
SMPS/VBUS/LED nets; the bench drives them, harmlessly).

## What it measures, and the bar

| Row | Pass |
|---|---|
| PRG from RAM: M2 rising -> D0-D7 valid, including the read right after every mapper write | <= 150 ns |
| PRG from uncached XIP (512K images) | <= 230 ns (<= 250 ns: run at 250 MHz) |
| core1's write path back before M2 falls (`guess_past_fall`) | 0 |
| RP-B: /RD falling -> PD0-PD7 valid | <= 110 ns |
| RP-B: /RD rising -> PD released | <= 40 ns |
| RP-B: PPU A10/A11/A13 -> CIRAM A10 / CIRAM /CE | <= 40 ns |
| Link: mapper write -> RP-B's CHR slot updated | <= 1 us, no frame errors |

The 2A03 samples D0-D7 when M2 falls, 350 ns after it rose; the PPU latches
PD when /RD rises, one dot (186 ns) after it fell. Information rows (no
verdict): raw XIP latency with and without the QMI cooldown, a cached-miss
read, XIP with another master on the QMI, the mapper engine's worst case per
Tier-1 mapper, the CPU test with a DMA hammering RAM, and CHR-RAM writes with
zero data hold.

## Limits of a one-chip bench

- The console's write data cannot reach D0-D7 (SIO owns those pins for the
  cart's reads), so bench writes carry their data in A1-A8.
- Both sides share one clock; the link test runs the transmitter at
  fractional dividers so its edges wander against the receiver.
- The pads see the chip's own drivers, not a console's: rise times, NMOS
  thresholds and the real data-hold after M2 are still bring-up items.
