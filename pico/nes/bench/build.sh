#!/usr/bin/env bash
# build.sh -- build the Rev1 timing bench (default board: a stock Pico 2).
#
# Usage: ./build.sh [board]      then flash build-<board>/nesbench.uf2 and run
#                                ../tools/bench_report.py
set -euo pipefail
cd "$(dirname "$0")"
BOARD=${1:-pico2}
export PICO_SDK_PATH="${PICO_SDK_PATH:-/usr/share/pico-sdk}"
cmake -S . -B "build-$BOARD" -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD="$BOARD"
ninja -C "build-$BOARD"
echo "build.sh: build-$BOARD/nesbench.uf2"
