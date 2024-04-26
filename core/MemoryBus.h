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

    uint8_t read(uint32_t addr) const;
    void write(uint32_t addr, uint8_t data);

    const uint8_t *mapAddress(uint32_t addr) const;

private:

    CPU &cpu;
};