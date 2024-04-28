#pragma once
#include <cstdint>
#include <functional>
#include <list>

class CPU;
class MemoryBus
{
public:
    MemoryBus(CPU &cpu);
    void reset();

    void setBIOSROM(const uint8_t *rom);

    uint8_t read(uint32_t addr) const;
    void write(uint32_t addr, uint8_t data);

    const uint8_t *mapAddress(uint32_t addr) const;

    uint8_t readIOPort(uint16_t addr);
    void writeIOPort(uint16_t addr, uint8_t data);

private:
    void updatePIT();

    uint8_t ram[64 * 1024];

    const uint8_t *biosROM = nullptr; // at 0xFE000

    struct PIT
    {
        uint8_t control[3]{0, 0, 0};
        uint8_t active = 0;

        uint16_t counter[3];
        uint16_t reload[3];
        uint16_t latch[3];

        uint8_t latched = 0;
        uint8_t highByte = 0; // lo/hi access mode

        uint32_t lastUpdateCycle = 0;
    };

    PIT pit;

    CPU &cpu;
};