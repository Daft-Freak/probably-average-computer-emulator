#include <cstdio>
#include <cstring>

#include "CPU.h"
#include "MemoryBus.h"


CPU::CPU() : mem(*this)
{}

void CPU::reset()
{
    mem.reset();
}


void CPU::run(int ms)
{
    int cycles = (clockSpeed * ms) / 1000;

    cyclesToRun += cycles;

    while(cyclesToRun > 0)
    {
        executeInstruction();
    }
}


void CPU::executeInstruction()
{
    cycleExecuted();
}

void CPU::cycleExecuted()
{
    cyclesToRun -= 1;
    cycleCount += 1;
}