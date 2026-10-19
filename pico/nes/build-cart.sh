#!/usr/bin/env bash
# build-cart.sh -- build the RP2354B cartridge firmware.
#
# Usage: ./build-cart.sh [board]          default board: fujines_rp2354b
#
# Builds RP-B (ppu/) and packs it into firmware/include/rpb_image.h; regenerates
# firmware/include/fujiconfigrom.h from the baked-in client and
# nesloaderrom.h from the loader ROM, then a plain pico-sdk CMake build.
# Client resolution: $FUJI_CLIENT if set; else build/config.nes -- the REAL
# CONFIG from fujinet-config/nes; else the fujicfg stand-in; else fujitest;
# else hello. The stand-ins exist for bring-up only.
set -euo pipefail
cd "$(dirname "$0")"

BOARD=${1:-fujines_rp2354b}
if [ -n "${FUJI_CLIENT:-}" ]; then
    CLIENT=$FUJI_CLIENT
elif [ -f build/config.nes ]; then
    CLIENT=config
else
    CLIENT=fujicfg
fi
if [ ! -f "build/$CLIENT.nes" ] && [ "$CLIENT" = fujicfg ]; then
    for fallback in fujitest hello; do
        if [ -f "build/$fallback.nes" ]; then
            echo "build-cart.sh: no fujicfg.nes yet; baking $fallback instead" >&2
            CLIENT=$fallback
            break
        fi
    done
fi
[ -f "build/$CLIENT.nes" ] || { echo "build-cart.sh: build/$CLIENT.nes missing; run ./build.sh first" >&2; exit 1; }
[ -f build/loader.bin ] || { echo "build-cart.sh: build/loader.bin missing; run ./build.sh loader" >&2; exit 1; }

export PICO_SDK_PATH="${PICO_SDK_PATH:-/usr/share/pico-sdk}"

# RP-B first: RP-A carries its image and loads it over SWD at every boot.
cmake -S ppu -B ppu/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD="$BOARD"
ninja -C ppu/build
python3 tools/mkrpbh.py ppu/build/fujines_ppu.elf > firmware/include/rpb_image.h

echo "build-cart.sh: baking build/$CLIENT.nes"
python3 tools/mkromh.py "build/$CLIENT.nes" > firmware/include/fujiconfigrom.h
python3 tools/mkloaderh.py build/loader.bin > firmware/include/nesloaderrom.h
for h in fujiconfigrom nesloaderrom rpb_image; do
    grep -q "^static const " "firmware/include/$h.h" \
        || { echo "build-cart.sh: firmware/include/$h.h was not generated" >&2; exit 1; }
done

cmake -S firmware -B "firmware/build-$BOARD" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD="$BOARD"
ninja -C "firmware/build-$BOARD"
python3 tools/checksram.py "firmware/build-$BOARD/fujines.elf"
echo "build-cart.sh: firmware/build-$BOARD/fujines.uf2"
