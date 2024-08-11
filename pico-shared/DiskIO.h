#pragma once

#include "FixedDiskAdapter.h"

#include "fatfs/ff.h"

class FileFixedIO final : public FixedDiskIO
{
public:
    bool isPresent(int unit) override;
    bool read(int unit, uint8_t *buf, uint32_t lba) override;
    bool write(int unit, const uint8_t *buf, uint32_t lba) override;

    void openDisk(int unit, const char *path);

    static const int maxDrives = 1;

private:
    FIL file[maxDrives];
};