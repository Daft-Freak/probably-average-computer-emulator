#include <filesystem>
#include <iostream>

#include "DiskIO.h"

bool FileFloppyIO::isPresent(int unit)
{
    return unit < maxDrives && file[unit].is_open();
}

bool FileFloppyIO::read(int unit, uint8_t *buf, uint8_t cylinder, uint8_t head, uint8_t sector)
{
    if(unit >= maxDrives)
        return false;

    int heads = doubleSided[unit] ? 2 : 1;
    auto lba = ((cylinder * heads + head) * sectorsPerTrack[unit]) + sector - 1;

    file[unit].clear();

    return file[unit].seekg(lba * 512).read(reinterpret_cast<char *>(buf), 512).gcount() == 512;
}

void FileFloppyIO::openDisk(int unit, std::string path)
{
    if(unit >= maxDrives)
        return;

    file[unit].close();

    file[unit].open(path);
    if(file[unit])
    {
        file[unit].seekg(0, std::ios::end);
        auto fdSize = file[unit].tellg();

        // try to work out geometry
        switch(fdSize / 1024)
        {
            case 160:
                doubleSided[unit] = false;
                sectorsPerTrack[unit] = 8;
                break;
            case 180:
                doubleSided[unit] = false;
                sectorsPerTrack[unit] = 9;
                break;
            case 360:
                doubleSided[unit] = true;
                sectorsPerTrack[unit] = 9;
                // could also be a single-sided 3.5-inch disk
                break;
            case 720: // 3.5 inch
                doubleSided[unit] = true;
                sectorsPerTrack[unit] = 9;
                break;
            case 1200:
                doubleSided[unit] = true;
                sectorsPerTrack[unit] = 15;
                break;
            default:
                std::cerr << "unhandled floppy image size " << fdSize << "(" << fdSize / 1024 << "k)\n";
                // set... something
                doubleSided[unit] = false;
                sectorsPerTrack[unit] = 8;
                break;
        }

        std::cout << "using " << (doubleSided[unit] ? 2 : 1) << " head(s) " << sectorsPerTrack[unit] << " sectors/track for floppy image\n";
    }
}

bool FileFixedIO::isPresent(int unit)
{
    return unit < maxDrives && file[unit].is_open();
}

bool FileFixedIO::read(int unit, uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    file[unit].clear();

    return file[unit].seekg(lba * 512).read(reinterpret_cast<char *>(buf), 512).gcount() == 512;
}

bool FileFixedIO::write(int unit, const uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    file[unit].clear();

    return file[unit].seekp(lba * 512).write(reinterpret_cast<const char *>(buf), 512).good();
}

void FileFixedIO::openDisk(int unit, std::string path)
{
    if(unit >= maxDrives)
        return;

    if(!std::filesystem::exists(path))
    {
        // new disk image
        file[unit].open(path, std::ios::in | std::ios::out | std::ios::trunc | std::ios::binary);

        // fill boot sector
        for(int i = 0; i < 512; i++)
            file[unit].put(0);
    }
    else
        file[unit].open(path, std::ios::in | std::ios::out | std::ios::binary);

    if(file[unit])
        std::cout << "Loaded fixed-disk " << unit << ": " << path << "\n";
}
