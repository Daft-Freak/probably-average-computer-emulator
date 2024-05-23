#pragma once
#include "System.h"

class FloppyController final : public IODevice
{
public:
    // reads a 512 byte sector
    // TODO: this may end up being an IO interface class or something
    using ReadCallback = void(*)(uint8_t *, uint8_t, uint8_t, uint8_t, uint8_t);

    FloppyController(System &sys);

    void setReadCallback(ReadCallback cb);

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

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

    ReadCallback readCb = nullptr;
};