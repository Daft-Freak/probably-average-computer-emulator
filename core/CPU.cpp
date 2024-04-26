#include <cstdio>
#include <cstring>

#include "CPU.h"
#include "MemoryBus.h"


CPU::CPU() : mem(*this)
{}

void CPU::reset()
{
    reg(Reg16::CS) = 0xFFFF;
    reg(Reg16::DS) = reg(Reg16::ES) = reg(Reg16::SS) = 0;

    reg(Reg16::IP) = 0;

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
    auto addr = (reg(Reg16::CS) << 4) + (reg(Reg16::IP)++);

    auto opcode = mem.read(addr);
    cycleExecuted();

    switch(opcode)
    {
        default:
            printf("op %x @%05x\n", (int)opcode, addr);
            exit(1);
            break;
    }
}

void CPU::cycleExecuted()
{
    cyclesToRun -= 1;
    cycleCount += 1;
}