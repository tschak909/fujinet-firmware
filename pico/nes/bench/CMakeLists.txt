cmake_minimum_required(VERSION 3.13)

# The NES Rev1 timing bench. Default board: a stock Pico 2. The cart's own
# board (fujines_rp2354b) also works, for a power-on self test on hardware.
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../firmware/boards)
if(NOT PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

include(pico_sdk_import.cmake)

project(nesbench C CXX ASM)

pico_sdk_init()

set(FW ${CMAKE_CURRENT_LIST_DIR}/../firmware)

add_executable(nesbench
    src/bench_main.c
    src/bench_cpu.c
    src/bench_ppu.c
    src/bench_link.c
    src/bench_micro.c
    src/dut_cpu.c
    src/dut_ppu.c
    ${FW}/src/nesmap.c
    ${FW}/src/nes_engine.c
    ${FW}/src/swd_host.c
    src/bench_swd.c
)

pico_generate_pio_header(nesbench ${CMAKE_CURRENT_LIST_DIR}/src/bench.pio)
pico_generate_pio_header(nesbench ${FW}/src/nes_engine.pio)

target_compile_options(nesbench PRIVATE -O3 -Wall)
# The firmware's engine with its bench hooks: write data on A1-A8, histograms.
target_compile_definitions(nesbench PRIVATE PICO_BOARD="${PICO_BOARD}"
    NES_ENGINE_BENCH_DATA NES_ENGINE_DIAG SWD_HOST_DBGFORCE)

target_include_directories(nesbench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${FW}/include
)

target_link_libraries(nesbench PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_flash
    hardware_xip_cache
    hardware_sync
    hardware_vreg
    hardware_pwm
)

# Everything runs from SRAM, as on the cart: the QMI belongs to the PRG test
# image while the bench runs, and its timing changes under the code's feet.
pico_set_binary_type(nesbench copy_to_ram)
pico_enable_stdio_usb(nesbench 1)
pico_enable_stdio_uart(nesbench 0)

pico_add_extra_outputs(nesbench)
