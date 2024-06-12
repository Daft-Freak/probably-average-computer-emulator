#pragma once
#include "FIFO.h"
#include "System.h"

class SerialMouse final : public IODevice
{
public:
    SerialMouse(System &sys);

    void addMotion(int x, int y);
    void setButton(int button, bool state);

    void sync();

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

private:
    System &sys;

    uint16_t divisor = 0;
    uint8_t interruptEnable = 0;
    uint8_t lineControl = 0;
    uint8_t modemControl = 0;

    // hardware doesn't have a queue, but we're doing it anyway
    FIFO<uint8_t, 8> rxQueue;

    uint8_t buttons = 0;
    uint8_t changedButtons = 0;
    int xMotion = 0, yMotion = 0;
};