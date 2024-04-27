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

    // 7x
    auto jump8 = [this, addr](int cond)
    {
        auto off = static_cast<int8_t>(mem.read(addr + 1));
        bool condVal = false;

        switch(cond)
        {
            case 0x0: // JO
                condVal = flags & Flag_O;
                break;
            case 0x1: // JNO
                condVal = !(flags & Flag_O);
                break;
            case 0x2: // JB/JNAE
                condVal = flags & Flag_C;
                break;
            case 0x3: // JAE/JNB
                condVal = !(flags & Flag_C);
                break;
            case 0x4: // JE/JZ
                condVal = flags & Flag_Z;
                break;
            case 0x5: // JNE/JNZ
                condVal = !(flags & Flag_Z);
                break;
            case 0x6: // JBE/JNA
                condVal = flags & (Flag_C | Flag_Z);
                break;
            // JNBE/JA
            case 0x8: // JS
                condVal = flags & Flag_S;
                break;
            case 0x9: // JNS
                condVal = !(flags & Flag_S);
                break;
            case 0xA: // JP/JPE
                condVal = flags & Flag_P;
                break;
            case 0xB: // JNP/JPO
                condVal = !(flags & Flag_P);
                break;
            // JL/JNGE
            // JNL/JGE
            // JLE/JNG
            // JNLE/JG
        }

        if(condVal)
        {
            reg(Reg16::IP) = reg(Reg16::IP) + 1 + off;
            cyclesExecuted(16);
        }
        else
        {
            reg(Reg16::IP)++;
            cyclesExecuted(4);
        }
    };

    switch(opcode)
    {
        case 0x70: // JO
        case 0x71: // JNO
        case 0x72: // JB/JNAE
        case 0x73: // JAE/JNB
        case 0x74: // JE/JZ
        case 0x75: // JNE/JNZ
        case 0x76: // JBE/JNA
        case 0x78: // JS
        case 0x79: // JNS
        case 0x7A: // JP/JPE
        case 0x7B: // JNP/JPO
        {
            jump8(opcode & 0xF);
            break;
        }

        case 0xEA: // JMP far
        {
            auto newIP = mem.read(addr + 1) | mem.read(addr + 2) << 8;
            auto newCS = mem.read(addr + 3) | mem.read(addr + 4) << 8;
            reg(Reg16::IP) = newIP;
            reg(Reg16::CS) = newCS;

            cyclesExecuted(15);
            break;
        }
        case 0xEB: // JMP short
        {
            auto off = static_cast<int8_t>(mem.read(addr + 1));

            reg(Reg16::IP) = reg(Reg16::IP) + 1 + off;
            cyclesExecuted(15);
            break;
        }
        default:
            printf("op %x @%05x\n", (int)opcode, addr);
            exit(1);
            break;
    }
}

void CPU::cyclesExecuted(int cycles)
{
    cyclesToRun -= cycles;
    cycleCount += cycles;
}