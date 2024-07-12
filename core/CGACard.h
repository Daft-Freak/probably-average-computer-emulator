#pragma once
#include "System.h"

class CGACard final : public IODevice
{
public:
    using ScanlineCallback = void(*)(const uint8_t *data, int line, int w);

    CGACard(System &sys);

    bool isInVBlank() const;

    void setScanlineCallback(ScanlineCallback cb);

    void update();

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

    void updateForInterrupts() override {};
    int getCyclesToNextInterrupt(uint32_t cycleCount) override {return 0;}

private:
    void draw(int start, int end);

    System &sys;

    // 6845 registers
    uint8_t regSelect;
    uint8_t regs[18];

    uint8_t mode = 0;
    uint8_t colSelect;
    uint8_t status = 0;

    uint32_t lastUpdateCycle = 0;
    uint16_t scanline = 0;
    uint16_t scanlineCycle = 0;
    uint16_t curAddr = 0;
    uint16_t frame = 0;
    uint8_t scanlineBuf[320];

    uint8_t ram[16 * 1024]; // at B8000

    ScanlineCallback scanCb;
};