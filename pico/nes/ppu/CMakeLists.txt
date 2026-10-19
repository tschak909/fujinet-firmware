cmake_minimum_required(VERSION 3.13)

# RP-B: the FujiNet NES cart's PPU-bus RP2354B (Rev1). Built no_flash: RP-A
# loads it into SRAM over SWD at every boot (firmware/src/rpb_swd.c), from the
# image tools/mkrpbh.py packs into firmware/include/rpb_image.h. It shares
# the board header, the mapper engine and the link with RP-A's firmware.
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../firmware/boards)
if(NOT PICO_BOARD)
    set(PICO_BOARD fujines_rp2354b)
endif()
include(${CMAKE_CURRENT_LIST_DIR}/../firmware/boards/${PICO_BOARD}.cmake)

include(pico_sdk_import.cmake)

project(fujines_ppu C CXX ASM)

pico_sdk_init()

set(FW ${CMAKE_CURRENT_LIST_DIR}/../firmware)

add_executable(fujines_ppu
    src/ppu_main.c
    src/ppu_bus.c
    src/ppu_link.c
    ${FW}/src/nesmap.c
)

pico_generate_pio_header(fujines_ppu ${CMAKE_CURRENT_LIST_DIR}/src/ppu_ciram.pio)
pico_generate_pio_header(fujines_ppu ${FW}/src/nes_link.pio)

target_compile_options(fujines_ppu PRIVATE -O2 -Wall)

target_include_directories(fujines_ppu PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${FW}/include
)

target_link_libraries(fujines_ppu PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_vreg
)

pico_set_binary_type(fujines_ppu no_flash)
pico_enable_stdio_uart(fujines_ppu 0)
pico_enable_stdio_usb(fujines_ppu 0)

pico_add_extra_outputs(fujines_ppu)
