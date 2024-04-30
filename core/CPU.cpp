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
        if(mem.hasInterrupt() && (flags & Flag_I))
            serviceInterrupt(mem.acknowledgeInterrupt());

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
        case 0x06: // PUSH seg
        case 0x0E:
        case 0x16:
        case 0x1E:
        {
            auto r = static_cast<Reg16>(((opcode >> 3) & 7) + static_cast<int>(Reg16::ES));

            reg(Reg16::SP) -= 2;
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            mem.write(stackAddr, reg(r) & 0xFF);
            mem.write(stackAddr + 1, reg(r) >> 8);

            cyclesExecuted(10 + 4);
            break;
        }

        case 0x07: // POP seg
        // 0x0F (CS) illegal
        case 0x17:
        case 0x1F:
        {
            auto r = static_cast<Reg16>(((opcode >> 3) & 7) + static_cast<int>(Reg16::ES));

            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            reg(r) = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;
            reg(Reg16::SP) += 2;

            cyclesExecuted(8 + 4);
            break;
        }

        case 0x50: // PUSH
        case 0x51:
        case 0x52:
        case 0x53:
        case 0x54:
        case 0x55:
        case 0x56:
        case 0x57:
        {
            auto r = static_cast<Reg16>(opcode & 7);

            reg(Reg16::SP) -= 2;
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            mem.write(stackAddr, reg(r) & 0xFF);
            mem.write(stackAddr + 1, reg(r) >> 8);

            cyclesExecuted(11 + 4);
            break;
        }

        case 0x58: // POP
        case 0x59:
        case 0x5A:
        case 0x5B:
        case 0x5C:
        case 0x5D:
        case 0x5E:
        case 0x5F:
        {
            auto r = static_cast<Reg16>(opcode & 7);

            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            reg(r) = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;
            reg(Reg16::SP) += 2;

            cyclesExecuted(8 + 4);
            break;
        }

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

        case 0x90: // NOP
        {
            cyclesExecuted(3);
            break;
        }

        case 0x9C: // PUSHF
        {
            reg(Reg16::SP) -= 2;
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            mem.write(stackAddr, flags & 0xFF);
            mem.write(stackAddr + 1, flags >> 8);

            cyclesExecuted(10 + 4);
            break;
        }
        case 0x9D: // POPF
        {
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            flags = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;
            reg(Reg16::SP) += 2;

            cyclesExecuted(8 + 4);
            break;
        }

        case 0x9E: // SAHF
        {
            auto mask = Flag_C | Flag_P | Flag_A | Flag_Z | Flag_S;
            flags = (flags & ~mask) | (reg(Reg8::AH) & mask);
            cyclesExecuted(4);
            break;
        }

        case 0x9F: // LAHF
        {
            reg(Reg8::AH) = flags;
            cyclesExecuted(4);
            break;
        }

        case 0xB0: // MOV imm -> reg8
        case 0xB1:
        case 0xB2:
        case 0xB3:
        case 0xB4:
        case 0xB5:
        case 0xB6:
        case 0xB7:
        {
            auto r = static_cast<Reg8>(opcode & 7);
            reg(r) = mem.read(addr + 1);
            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }

        case 0xB8: // MOV imm -> reg16
        case 0xB9:
        case 0xBA:
        case 0xBB:
        case 0xBC:
        case 0xBD:
        case 0xBE:
        case 0xBF:
        {
            auto r = static_cast<Reg16>(opcode & 7);
            reg(r) = mem.read(addr + 1) | mem.read(addr + 2) << 8;
            reg(Reg16::IP) += 2;
            cyclesExecuted(4);
            break;
        }

        case 0xC3: // RET near
        {
            // pop from stack
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);
            reg(Reg16::SP) += 2;
            auto newIP = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;

            reg(Reg16::IP) = newIP;
            cyclesExecuted(16 + 4);
            break;
        }


        case 0xE2: // LOOP
        {
            auto off = static_cast<int8_t>(mem.read(addr + 1));

            uint16_t count = --reg(Reg16::CX);

            if(count == 0)
            {
                // done
                reg(Reg16::IP)++;
                cyclesExecuted(5);
            }
            else
            {
                reg(Reg16::IP) = reg(Reg16::IP) + 1 + off;
                cyclesExecuted(17);
            }
            break;
        }

        case 0xE4: // IN AL from imm8
        {
            auto port = mem.read(addr + 1);
            reg(Reg8::AL) = mem.readIOPort(port);

            reg(Reg16::IP)++;
            cyclesExecuted(10);
            break;
        }

        case 0xE6: // OUT AL to imm8
        {
            auto port = mem.read(addr + 1);
            auto data = reg(Reg8::AL);

            mem.writeIOPort(port, data);

            reg(Reg16::IP)++;
            cyclesExecuted(10);
            break;
        }

        case 0xE8: // CALL
        {
            auto off = mem.read(addr + 1) | mem.read(addr + 2) << 8;

            // push
            reg(Reg16::SP) -= 2;
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);

            auto retAddr = reg(Reg16::IP) + 2;

            mem.write(stackAddr, retAddr & 0xFF);
            mem.write(stackAddr + 1, retAddr >> 8);

            reg(Reg16::IP) = reg(Reg16::IP) + 2 + off;
            cyclesExecuted(19);
            break;
        }

        case 0xE9: // JMP near
        {
            auto off = mem.read(addr + 1) | mem.read(addr + 2) << 8;

            reg(Reg16::IP) = reg(Reg16::IP) + 2 + off;
            cyclesExecuted(15);
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

        case 0xEC: // IN AL from DX
        {
            auto port = reg(Reg16::DX);

            reg(Reg8::AL) = mem.readIOPort(port);

            cyclesExecuted(8);
            break;
        }

        case 0xEE: // OUT AL to DX
        {
            auto port = reg(Reg16::DX);
            auto data = reg(Reg8::AL);

            mem.writeIOPort(port, data);

            cyclesExecuted(8);
            break;
        }

        case 0xF8: // CLC
        {
            flags &= ~Flag_C;
            cyclesExecuted(2);
            break;
        }
        case 0xF9: // STC
        {
            flags |= Flag_C;
            cyclesExecuted(2);
            break;
        }
        case 0xFA: // CLI
        {
            flags &= ~Flag_I;
            cyclesExecuted(2);
            break;
        }
        case 0xFB: // STI
        {
            flags |= Flag_I;
            cyclesExecuted(2);
            break;
        }
        case 0xFC: // CLD
        {
            flags &= ~Flag_D;
            cyclesExecuted(2);
            break;
        }
        case 0xFD: // STD
        {
            flags |= Flag_D;
            cyclesExecuted(2);
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

void CPU::serviceInterrupt(uint8_t vector)
{
    auto addr = vector * 4;

    auto newIP = mem.read(addr) | mem.read(addr + 1) << 8;
    auto newCS = mem.read(addr + 2) | mem.read(addr + 3) << 8;

    auto ss = reg(Reg16::SS) << 4;

    // push flags
    reg(Reg16::SP) -= 2;
    auto stackAddr = ss + reg(Reg16::SP);
    mem.write(stackAddr, flags & 0xFF);
    mem.write(stackAddr + 1, flags >> 8);

    // clear I/T
    flags &= ~(Flag_T | Flag_I);

    // inter-segment indirect call

    // push CS
    reg(Reg16::SP) -= 2;
    stackAddr = ss + reg(Reg16::SP);
    mem.write(stackAddr, reg(Reg16::CS) & 0xFF);
    mem.write(stackAddr + 1, reg(Reg16::CS) >> 8);

    // push IP
    reg(Reg16::SP) -= 2;
    stackAddr = ss + reg(Reg16::SP);

    auto retAddr = reg(Reg16::IP);

    mem.write(stackAddr, retAddr & 0xFF);
    mem.write(stackAddr + 1, retAddr >> 8);

    reg(Reg16::CS) = newCS;
    reg(Reg16::IP) = newIP;
    cyclesExecuted(37 + 6 + 4 * 4); // maybe? roughly cycles for indirect call with disp
}
