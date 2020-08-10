#include "../../include/debug.h"
#include "../utils/utils.h"

#include "diskType.h"

#define DENSITY_FM 0
#define DENSITY_MFM 4

#define SIDES_SS 0
#define SIDES_DS 1

// Update PERCOM block from the total # of sectors
void DiskType::derive_percom_block(uint16_t numSectors)
{
    // Start with 40T/1S 720 sectors, sector size passed in
    _percomBlock.num_tracks = 40;
    _percomBlock.step_rate = 1;
    _percomBlock.sectors_per_trackH = 0;
    _percomBlock.sectors_per_trackL = 18;
    _percomBlock.num_sides = SIDES_SS;
    _percomBlock.density = DENSITY_FM;
    _percomBlock.sector_sizeH = HIBYTE_FROM_UINT16(_sectorSize);
    _percomBlock.sector_sizeL = LOBYTE_FROM_UINT16(_sectorSize);
    _percomBlock.drive_present = 255;
    _percomBlock.reserved1 = 0;
    _percomBlock.reserved2 = 0;
    _percomBlock.reserved3 = 0;

    if (numSectors == 1040) // 5.25" 1050 density
    {
        _percomBlock.sectors_per_trackL = 26;
        _percomBlock.density = DENSITY_MFM;
    }
    else if (numSectors == 720 && _sectorSize == 256) // 5.25" SS/DD
    {
        _percomBlock.density = DENSITY_MFM;
    }
    else if (numSectors == 1440) // 5.25" DS/DD
    {
        _percomBlock.num_sides = SIDES_DS;
        _percomBlock.density = DENSITY_MFM;
    }
    else if (numSectors == 2880) // 5.25" DS/QD
    {
        _percomBlock.num_sides = SIDES_DS;
        _percomBlock.num_tracks = 80;
        _percomBlock.density = DENSITY_MFM;
    }
    else if (numSectors == 2002 && _sectorSize == 128) // SS/SD 8"
    {
        _percomBlock.num_tracks = 77;
    }
    else if (numSectors == 2002 && _sectorSize == 256) // SS/DD 8"
    {
        _percomBlock.num_tracks = 77;
        _percomBlock.density = DENSITY_MFM;
    }
    else if (numSectors == 4004 && _sectorSize == 128) // DS/SD 8"
    {
        _percomBlock.num_tracks = 77;
    }
    else if (numSectors == 4004 && _sectorSize == 256) // DS/DD 8"
    {
        _percomBlock.num_sides = SIDES_DS;
        _percomBlock.num_tracks = 77;
        _percomBlock.density = DENSITY_MFM;
    }
    else if (numSectors == 5760) // 1.44MB 3.5" High Density
    {
        _percomBlock.num_sides = SIDES_DS;
        _percomBlock.num_tracks = 80;
        _percomBlock.sectors_per_trackL = 36;
        _percomBlock.density = 8; // I think this is right.
    }
    else
    {
        // This is a custom size, one long track.
        _percomBlock.num_tracks = 1;
        _percomBlock.sectors_per_trackH = HIBYTE_FROM_UINT16(numSectors);
        _percomBlock.sectors_per_trackL = LOBYTE_FROM_UINT16(numSectors);
    }

#ifdef VERBOSE_DISK
    dump_percom_block();
#endif
}

// Dump PERCOM block
void DiskType::dump_percom_block()
{
#ifdef VERBOSE_DISK
    Debug_printf("Percom Block Dump\n");
    Debug_printf("-----------------\n");
    Debug_printf("Num Tracks: %d\n", _percomBlock.num_tracks);
    Debug_printf("Step Rate: %d\n", _percomBlock.step_rate);
    Debug_printf("Sectors per Track: %d\n", (_percomBlock.sectors_per_trackH * 256 + _percomBlock.sectors_per_trackL));
    Debug_printf("Num Sides: %d\n", _percomBlock.num_sides);
    Debug_printf("Density: %d\n", _percomBlock.density);
    Debug_printf("Sector Size: %d\n", (_percomBlock.sector_sizeH * 256 + _percomBlock.sector_sizeL));
    Debug_printf("Drive Present: %d\n", _percomBlock.drive_present);
    Debug_printf("Reserved1: %d\n", _percomBlock.reserved1);
    Debug_printf("Reserved2: %d\n", _percomBlock.reserved2);
    Debug_printf("Reserved3: %d\n", _percomBlock.reserved3);
#endif
}

void DiskType::unmount()
{
    if (_file != nullptr)
    {
        fclose(_file);
        _file = nullptr;
    }
}

DiskType::~DiskType()
{
    unmount();
}
