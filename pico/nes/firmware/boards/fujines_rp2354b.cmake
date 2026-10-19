# fujines_rp2354b: the FujiNet NES cartridge (Rev1) -- two RP2354Bs (RP2350B
# die + 2 MB flash in the package, QFN-80, 48 GPIO) on the console's 5 V
# buses: RP-A on the CPU bus (firmware/), RP-B on the PPU bus (ppu/). Pin
# maps in include/nes_bus.h, nes_cart.h and nes_ppu.h. See
# boards/fujines_rp2354b.h.
set(PICO_PLATFORM rp2350)
set(PICO_FLASH_SIZE_BYTES 2097152)

option(CONFIG_FUJINET "Enable the FujiNet mailbox" ON)
