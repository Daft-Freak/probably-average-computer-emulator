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

private:
    uint8_t ram[64 * 1024];

    const uint8_t *biosROM = nullptr; // at 0xFE000

    CPU &cpu;
};