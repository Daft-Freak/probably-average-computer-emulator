#pragma once
#include "System.h"

class AboveBoard final : public IODevice
{
public:
    AboveBoard(System &sys);

    uint8_t read(uint16_t addr) override;
    void write(uint16_t addr, uint8_t data) override;

    void updateForInterrupts() override {}

private:
    System &sys;

    uint8_t detectF;

    uint8_t pageMask = 0;

    uint16_t eeprom[64] {
        0x000D, 0x0C00, 0xFFFF, 0x07FF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        0xBB44, 0x9E61, 0x9966, 0x8B74, 0xAD52, 0xBE41, 0xB24D, 0xCE31,
        0xCD32, 0xCC33, 0xCB34, 0x5CBA, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xAA01,
    };
    uint16_t eepromData;

    uint8_t pageMapping[16];

    uint8_t ram[2 * 1024 * 1024];
};