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

    uint8_t read(uint16_t addr) const;
    void write(uint16_t addr, uint8_t data);

    const uint8_t *mapAddress(uint16_t addr) const;

private:

    CPU &cpu;
};