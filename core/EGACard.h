#pragma once
#include "System.h"

class EGACard final : public IODevice
{
public:
    using ScanlineCallback = void(*)(const uint8_t *data, int line, int w);

    EGACard(System &sys);

    void remove();

    bool isInVBlank() const;

    void setScanlineCallback(ScanlineCallback cb);

    void update();

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

    void updateForInterrupts() override {};
    int getCyclesToNextInterrupt(uint32_t cycleCount) override {return 0;}

private:
    void draw(int start, int end);

    void setupMemory();

    uint8_t readMem(uint32_t addr);
    void writeMem(uint32_t addr, uint8_t data);

    static uint8_t readMem(uint32_t addr, void *userData)
    {
        return reinterpret_cast<EGACard *>(userData)->readMem(addr);
    }
    static void writeMem(uint32_t addr, uint8_t data, void *userData)
    {
        reinterpret_cast<EGACard *>(userData)->writeMem(addr, data);
    }

    System &sys;

    // sequencer
    uint8_t seqAddr = 0;
    // reset
    uint8_t seqClockMode = 0;
    uint8_t seqMapMask = 0;
    // char map
    uint8_t seqMemMode = 0;

    // CRTC registers
    uint8_t regSelect;
    uint8_t regs[24];

    // FIXME: CGA leftovers
    uint8_t mode = 0;
    uint8_t colSelect;

    // graphics controller
    uint8_t gfxAddr = 0;
    // set/res, set/res en, compare, rotate
    uint8_t gfxReadSel = 0;
    // mode
    uint8_t gfxMisc = 0;
    // don't care
    // mask

    // attribute controller
    uint8_t attribCtrlAddr = 0;
    // palette, mode, overscan
    uint8_t attribColourPlaneEnable = 0;
    // horiz pan

    // external
    uint8_t miscOutput = 0;
    uint8_t featureControl = 0;
    uint8_t status = 0; // status1

    uint32_t lastUpdateCycle = 0;
    uint16_t scanline = 0;
    uint16_t scanlineCycle = 0;
    uint16_t curAddr = 0;
    uint16_t frame = 0;
    bool inCursor = false;
    uint8_t lastOutData = 0;

    uint8_t scanlineBuf[320];

    uint8_t ram[64 * 1024]; // minimum

    ScanlineCallback scanCb;
};