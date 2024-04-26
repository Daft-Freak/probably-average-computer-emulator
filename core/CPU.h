#pragma once
#include <cstdint>

#include "MemoryBus.h"

class CPU final
{
public:

    CPU();

    void reset();

    void run(int ms);

    MemoryBus &getMem() {return mem;}
private:
    void executeInstruction();

    void cycleExecuted();

    static const uint32_t clockSpeed = 4772726;

    // internal state
    int cyclesToRun = 0;
    uint32_t cycleCount = 0;

    // registers

    // RAM
    MemoryBus mem;
};
