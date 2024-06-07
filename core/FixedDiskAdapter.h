#pragma once
#include "System.h"

class FixedDiskIO
{
public:
    // is drive connected to controller
    virtual bool isPresent(int unit) = 0;

    // reads a 512 byte sector
    virtual bool read(int unit, uint8_t *buf, uint32_t lba) = 0;
    virtual bool write(int unit, const uint8_t *buf, uint32_t lba) = 0;
};

class FixedDiskAdapter final : public IODevice
{
public:
    FixedDiskAdapter(System &sys);

    void setIOInterface(FixedDiskIO *io);

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

private:
    System &sys;

    uint8_t status = 0;
    uint8_t dmaIntrMask = 0;

    uint8_t controlBlock[6];
    uint8_t controlBlockOffset = 0;
    uint8_t commandDataLen = 0, commandDataOffset = 0;

    uint8_t sense[4];

    uint8_t data[8];
    uint8_t responseOffset = 0, responseLen = 0;

    uint16_t numCylinders[2];
    uint8_t numHeads[2];

    FixedDiskIO *io = nullptr;
};