#pragma once
#include "System.h"

class FloppyDiskIO
{
public:
    // is there a disk in the drive
    virtual bool isPresent(int unit) = 0;

    // reads a 512 byte sector
    virtual bool read(int unit, uint8_t *buf, uint8_t cylinder, uint8_t head, uint8_t sector) = 0;
};

class FloppyController final : public IODevice
{
public:
    FloppyController(System &sys);

    void setIOInterface(FloppyDiskIO *io);

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

    void updateForInterrupts() override {};

private:
    System &sys;

    uint8_t digitalOutput = 0;

    uint8_t status[4] = {0, 0, 0, 0};
    uint8_t presentCylinder[4];

    uint8_t command[9];
    uint8_t result[7];
    uint8_t commandLen = 0, resultLen = 0;
    uint8_t commandOff, resultOff;

    uint8_t readyChanged;

    FloppyDiskIO *io = nullptr;
};