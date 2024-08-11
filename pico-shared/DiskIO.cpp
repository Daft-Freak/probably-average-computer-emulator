#include <cstdio>

#include "DiskIO.h"

bool FileFixedIO::isPresent(int unit)
{
    return unit < maxDrives;// && file[unit];
}

bool FileFixedIO::read(int unit, uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    f_lseek(&file[unit], lba * 512);

    UINT read = 0;
    auto res = f_read(&file[unit], buf, 512, &read);

    return res == FR_OK && read == 512;
}

bool FileFixedIO::write(int unit, const uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    f_lseek(&file[unit], lba * 512);

    UINT written = 0;
    auto res = f_write(&file[unit], buf, 512, &written);

    return res == FR_OK && written == 512;
}

void FileFixedIO::openDisk(int unit, const char *path)
{
    if(unit >= maxDrives)
        return;

    auto res = f_open(&file[unit], path, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);

    if(res == FR_OK)
        printf("Loaded fixed-disk %i: %s\n", unit, path);
}