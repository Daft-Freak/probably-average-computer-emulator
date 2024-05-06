#include <cstdio>
#include <cstring>

#include "CPU.h"
#include "MemoryBus.h"

enum Flags
{
    Flag_C = (1 << 0),
    Flag_P = (1 << 2),
    Flag_A = (1 << 4),
    Flag_Z = (1 << 6),
    Flag_S = (1 << 7),
    Flag_T = (1 << 8),
    Flag_I = (1 << 9),
    Flag_D = (1 << 10),
    Flag_O = (1 << 11),
};

// opcode helpers

static constexpr bool parity(uint8_t v)
{
    return ~(0x6996 >> ((v ^ (v >> 4)) & 0xF)) & 1;
};

template<class T>
static constexpr int signBit()
{
    return 1 << ((sizeof(T) * 8) - 1);
}

template<class T>
static T doAdd(T dest, T src, uint16_t &flags)
{
    T res = dest + src;

    bool overflow = ~(dest ^ src) & (src ^ res) & signBit<T>();

    flags = (flags & ~(Flag_C | Flag_P | Flag_A | Flag_Z | Flag_S | Flag_O))
          | (res < dest ? Flag_C : 0) 
          | (parity(res) ? Flag_P : 0)
          | ((res & 0xF) < (dest & 0xF) ? Flag_A : 0)
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (overflow ? Flag_O : 0);

    return res;
}

template<class T>
static T doAddWithCarry(T dest, T src, uint16_t &flags)
{
    int c = flags & Flag_C ? 1 : 0;
    T res = dest + src + c;

    bool carry = res < dest || (res == dest && c);
    bool overflow = ~(dest ^ src) & (src ^ res) & signBit<T>();

    flags = (flags & ~(Flag_C | Flag_P | Flag_A | Flag_Z | Flag_S | Flag_O))
          | (carry ? Flag_C : 0) 
          | (parity(res) ? Flag_P : 0)
          | ((res & 0xF) < (dest & 0xF) + c ? Flag_A : 0)
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (overflow ? Flag_O : 0);

    return res;
}

template<class T>
static T doAnd(T dest, T src, uint16_t &flags)
{
    T res = dest & src;

    // c/o cleared
    // szp set from res
    flags = (flags & ~(Flag_C | Flag_P | Flag_Z | Flag_S | Flag_O))
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (parity(res) ? Flag_P : 0);

    return res;
}

template<class T>
static T doDec(T dest, uint16_t &flags)
{
    T res = dest - 1;

    flags = (flags & ~(Flag_P | Flag_A | Flag_Z | Flag_S | Flag_O))
          | (parity(res) ? Flag_P : 0)
          | ((res & 0xF) == 0xF ? Flag_A : 0)
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (res == signBit<T>() - 1 ? Flag_O : 0);

    return res;
}

template<class T>
static T doInc(T dest, uint16_t &flags)
{
    T res = dest + 1;

    flags = (flags & ~(Flag_P | Flag_A | Flag_Z | Flag_S | Flag_O))
          | (parity(res) ? Flag_P : 0)
          | ((res & 0xF) == 0 ? Flag_A : 0)
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (res == signBit<T>() ? Flag_O : 0);

    return res;
}

template<class T>
static T doOr(T dest, T src, uint16_t &flags)
{
    T res = dest | src;

    // c/o cleared
    // szp set from res
    flags = (flags & ~(Flag_C | Flag_P | Flag_Z | Flag_S | Flag_O))
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (parity(res) ? Flag_P : 0);

    return res;
}

template<class T>
static T doSub(T dest, T src, uint16_t &flags)
{
    T res = dest - src;

    bool overflow = (dest ^ src) & (src ^ res) & signBit<T>();

    flags = (flags & ~(Flag_C | Flag_P | Flag_A | Flag_Z | Flag_S | Flag_O))
          | (src > dest ? Flag_C : 0) 
          | (parity(res) ? Flag_P : 0)
          | ((res & 0xF) > (dest & 0xF) ? Flag_A : 0)
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (overflow ? Flag_O : 0);

    return res;
}

template<class T>
static T doSubWithBorrow(T dest, T src, uint16_t &flags)
{
    int c = flags & Flag_C ? 1 : 0;
    T res = dest - src - 1;

    bool carry = src > dest || (src == dest && c);
    bool overflow = (dest ^ src) & (src ^ res) & signBit<T>();

    flags = (flags & ~(Flag_C | Flag_P | Flag_A | Flag_Z | Flag_S | Flag_O))
          | (carry ? Flag_C : 0) 
          | (parity(res) ? Flag_P : 0)
          | ((res & 0xF) > (dest & 0xF) - c ? Flag_A : 0)
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (overflow ? Flag_O : 0);

    return res;
}

template<class T>
static T doXor(T dest, T src, uint16_t &flags)
{
    T res = dest ^ src;

    // c/o cleared
    // szp set from res
    flags = (flags & ~(Flag_C | Flag_P | Flag_Z | Flag_S | Flag_O))
          | (res == 0 ? Flag_Z : 0)
          | (res & signBit<T>() ? Flag_S : 0)
          | (parity(res) ? Flag_P : 0);

    return res;
}

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
        mem.updateForInterrupts();

        if(mem.hasInterrupt() && (flags & Flag_I))
            serviceInterrupt(mem.acknowledgeInterrupt());

        executeInstruction();
    }
}

void CPU::executeInstruction()
{
    auto addr = (reg(Reg16::CS) << 4) + (reg(Reg16::IP)++);

    auto opcode = mem.read(addr);
    bool rep = false;
    Reg16 segmentOverride = Reg16::AX; // not a segment reg, also == 0

    // prefixes
    while(true)
    {
        if((opcode & 0xE7) == 0x26) // segment override (26 = ES, 2E = CS, 36 = SS, 3E = DS)
            segmentOverride = static_cast<Reg16>(static_cast<int>(Reg16::ES) + ((opcode >> 3) & 3)); // the middle two bits
        else if(opcode == 0xF3) // REP/REPE
            rep = true;
        else
            break;

        opcode = mem.read(++addr);
        reg(Reg16::IP)++;
    }

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
            case 0x7: // JNBE/JA
                condVal = !(flags & (Flag_C | Flag_Z));
                break;
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
            case 0xC: // JL/JNGE
                condVal = !!(flags & Flag_S) != !!(flags & Flag_O);
                break;
            case 0xD: // JNL/JGE
                condVal = !!(flags & Flag_S) == !!(flags & Flag_O);
                break;
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
        case 0x04: // ADD AL imm8
        {
            uint8_t src = mem.read(addr + 1);

            reg(Reg8::AL) = doAdd(reg(Reg8::AL), src, flags);

            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }
        case 0x05: // ADD AX imm16
        {
            uint16_t imm = mem.read(addr + 1) | mem.read(addr + 2) << 8;

            reg(Reg16::AX) = doAdd(reg(Reg16::AX), imm, flags);

            reg(Reg16::IP) += 2;
            cyclesExecuted(4);
            break;
        }

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

        case 0x0C: // OR AL imm8
        {
            auto imm = mem.read(addr + 1);

            reg(Reg8::AL) = doOr(reg(Reg8::AL), imm, flags);

            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }

        case 0x14: // ADC AL imm8
        {
            uint8_t src = mem.read(addr + 1);

            reg(Reg8::AL) = doAddWithCarry(reg(Reg8::AL), src, flags);

            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }

        case 0x24: // AND AL imm8
        {
            auto imm = mem.read(addr + 1);

            reg(Reg8::AL) = doAnd(reg(Reg8::AL), imm, flags);

            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }

        case 0x27: // DAA
        {
            int val = reg(Reg8::AL);
            bool c = flags & Flag_C;

            if((val & 0xF) > 9 || (flags & Flag_A))
            {
                val += 6;

                flags |= Flag_A;

                if(val > 0xFF)
                {
                    // carry
                    flags |= Flag_C;
                    val &= 0xFF;
                }
            }
            else
                flags &= ~(Flag_A | Flag_C);

            if(reg(Reg8::AL) > 0x99 || c)
            {
                val += 0x60;
                flags |= Flag_C;
            }
            else
                flags &= ~Flag_C;

            flags = (flags & ~(Flag_P | Flag_Z | Flag_S))
                  | (val == 0 ? Flag_Z : 0)
                  | (val & 0x80 ? Flag_S : 0)
                  | (parity(val) ? Flag_P : 0);

            reg(Reg8::AL) = val;

            cyclesExecuted(4);
            break;
        }

        case 0x2C: // SUB AL imm8
        {
            auto imm = mem.read(addr + 1);

            reg(Reg16::AX) = doSub(reg(Reg8::AL), imm, flags);

            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }
        case 0x2D: // SUB AX imm16
        {
            uint16_t imm = mem.read(addr + 1) | mem.read(addr + 2) << 8;

            reg(Reg16::AX) = doSub(reg(Reg16::AX), imm, flags);

            reg(Reg16::IP) += 2;
            cyclesExecuted(4);
            break;
        }

        case 0x35: // XOR AX imm16
        {
            uint16_t imm = mem.read(addr + 1) | mem.read(addr + 2) << 8;

            reg(Reg16::AX) = doXor(reg(Reg16::AX), imm, flags);

            reg(Reg16::IP) += 2;
            cyclesExecuted(4);
            break;
        }

        case 0x3C: // CMP AL imm
        {
            auto imm = mem.read(addr + 1);

            doSub(reg(Reg8::AL), imm, flags);

            reg(Reg16::IP) += 1;
            cyclesExecuted(4);
            break;
        }
        case 0x3D: // CMP AX imm
        {
            uint16_t imm = mem.read(addr + 1) | mem.read(addr + 2) << 8;

            doSub(reg(Reg16::AX), imm, flags);

            reg(Reg16::IP) += 2;
            cyclesExecuted(4);
            break;
        }

        case 0x40: // INC reg16
        case 0x41:
        case 0x42:
        case 0x43:
        case 0x44:
        case 0x45:
        case 0x46:
        case 0x47:
        {
            auto destReg = static_cast<Reg16>(opcode & 7);
            reg(destReg) = doInc(reg(destReg), flags);
            cyclesExecuted(3);
            break;
        }

        case 0x48: // DEC reg16
        case 0x49:
        case 0x4A:
        case 0x4B:
        case 0x4C:
        case 0x4D:
        case 0x4E:
        case 0x4F:
        {
            auto destReg = static_cast<Reg16>(opcode & 7);
            reg(destReg) = doDec(reg(destReg), flags);
            cyclesExecuted(3);
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
        case 0x77: // JNBE/JA
        case 0x78: // JS
        case 0x79: // JNS
        case 0x7A: // JP/JPE
        case 0x7B: // JNP/JPO
        case 0x7C: // JL/JNGE
        case 0x7D: // JNL/JGE
        {
            jump8(opcode & 0xF);
            break;
        }

        case 0x90: // NOP
        {
            cyclesExecuted(3);
            break;
        }

        case 0x98: // CBW
        {
            if(reg(Reg8::AL) & 0x80)
                reg(Reg8::AH) = 0xFF;
            else
                reg(Reg8::AH) = 0;

            cyclesExecuted(2);
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

        case 0xA0: // MOV off16 -> AL
        {
            auto memAddr = mem.read(addr + 1) | mem.read(addr + 2) << 8;
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            memAddr += reg(segment) << 4;

            reg(Reg8::AL) = mem.read(memAddr);

            reg(Reg16::IP) += 2;
            cyclesExecuted(10);
            break;
        }
        case 0xA1: // MOV off16 -> AX
        {
            auto memAddr = mem.read(addr + 1) | mem.read(addr + 2) << 8;
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            memAddr += reg(segment) << 4;

            reg(Reg16::AX) = mem.read(memAddr) | mem.read(memAddr + 1) << 8;

            reg(Reg16::IP) += 2;
            cyclesExecuted(10 + 4);
            break;
        }
        case 0xA2: // MOV AL -> off16
        {
            auto memAddr = mem.read(addr + 1) | mem.read(addr + 2) << 8;
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            memAddr += reg(segment) << 4;

            mem.write(memAddr, reg(Reg8::AL));

            reg(Reg16::IP) += 2;
            cyclesExecuted(10);
            break;
        }
        case 0xA3: // MOV AX -> off16
        {
            auto memAddr = mem.read(addr + 1) | mem.read(addr + 2) << 8;
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            memAddr += reg(segment) << 4;

            mem.write(memAddr, reg(Reg8::AL));
            mem.write(memAddr + 1, reg(Reg8::AH));

            reg(Reg16::IP) += 2;
            cyclesExecuted(10 + 4);
            break;
        }

        case 0xA4: // MOVS byte
        {
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX))
                {
                    // TODO: interrupt

                    auto srcAddr = (reg(segment) << 4) + reg(Reg16::SI);
                    auto destAddr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);

                    mem.write(destAddr, mem.read(srcAddr));

                    if(flags & Flag_D)
                    {
                        reg(Reg16::SI)--;
                        reg(Reg16::DI)--;
                    }
                    else
                    {
                        reg(Reg16::SI)++;
                        reg(Reg16::DI)++;
                    }

                    reg(Reg16::CX)--;
                    cyclesExecuted(17);
                }
            }
            else
            {
                auto srcAddr = (reg(segment) << 4) + reg(Reg16::SI);
                auto destAddr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);

                mem.write(destAddr, mem.read(srcAddr));

                if(flags & Flag_D)
                {
                    reg(Reg16::SI)--;
                    reg(Reg16::DI)--;
                }
                else
                {
                    reg(Reg16::SI)++;
                    reg(Reg16::DI)++;
                }

                cyclesExecuted(18);
            }
            break;
        }
        case 0xA5: // MOVS word
        {
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX))
                {
                    // TODO: interrupt

                    auto srcAddr = (reg(segment) << 4) + reg(Reg16::SI);
                    auto destAddr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);

                    mem.write(destAddr, mem.read(srcAddr));
                    mem.write(destAddr + 1,  mem.read(srcAddr + 1));

                    if(flags & Flag_D)
                    {
                        reg(Reg16::SI) -= 2;
                        reg(Reg16::DI) -= 2;
                    }
                    else
                    {
                        reg(Reg16::SI) += 2;
                        reg(Reg16::DI) += 2;
                    }

                    reg(Reg16::CX)--;
                    cyclesExecuted(17 + 2 * 4);
                }
            }
            else
            {
                auto srcAddr = (reg(segment) << 4) + reg(Reg16::SI);
                auto destAddr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);

                mem.write(destAddr, mem.read(srcAddr));
                mem.write(destAddr + 1,  mem.read(srcAddr + 1));

                if(flags & Flag_D)
                {
                    reg(Reg16::SI) -= 2;
                    reg(Reg16::DI) -= 2;
                }
                else
                {
                    reg(Reg16::SI) += 2;
                    reg(Reg16::DI) += 2;
                }

                cyclesExecuted(18 + 2 * 4);
            }
            break;
        }

        case 0xA8: // TEST AL imm8
        {
            auto imm = mem.read(addr + 1);

            doAnd(reg(Reg8::AL), imm, flags);

            reg(Reg16::IP)++;
            cyclesExecuted(4);
            break;
        }

        case 0xAA: // STOS byte
        {
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX))
                {
                    // TODO: interrupt

                    auto addr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);
                    mem.write(addr, reg(Reg8::AL));

                    if(flags & Flag_D)
                        reg(Reg16::DI)--;
                    else
                        reg(Reg16::DI)++;

                    reg(Reg16::CX)--;
                    cyclesExecuted(10);
                }
            }
            else
            {
                auto addr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);
                mem.write(addr, reg(Reg8::AL));

                if(flags & Flag_D)
                    reg(Reg16::DI)--;
                else
                    reg(Reg16::DI)++;

                cyclesExecuted(11);
            }
            break;
        }
        case 0xAB: // STOS word
        {
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX))
                {
                    // TODO: interrupt

                    auto addr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);
                    mem.write(addr, reg(Reg8::AL));
                    mem.write(addr + 1, reg(Reg8::AH));

                    if(flags & Flag_D)
                        reg(Reg16::DI) -= 2;
                    else
                        reg(Reg16::DI) += 2;

                    reg(Reg16::CX)--;
                    cyclesExecuted(10 + 4);
                }
            }
            else
            {
                auto addr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);
                mem.write(addr, reg(Reg8::AL));
                mem.write(addr + 1, reg(Reg8::AH));

                if(flags & Flag_D)
                    reg(Reg16::DI) -= 2;
                else
                    reg(Reg16::DI) += 2;

                cyclesExecuted(11 + 4);
            }
            break;
        }
        case 0xAC: // LODS byte
        {
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX))
                {
                    // TODO: interrupt

                    auto addr = (reg(segment) << 4) + reg(Reg16::SI);
                    reg(Reg8::AL) = mem.read(addr);

                    if(flags & Flag_D)
                        reg(Reg16::SI)--;
                    else
                        reg(Reg16::SI)++;

                    reg(Reg16::CX)--;
                    cyclesExecuted(13);
                }
            }
            else
            {
                auto addr = (reg(segment) << 4) + reg(Reg16::SI);
                reg(Reg8::AL) = mem.read(addr);

                if(flags & Flag_D)
                    reg(Reg16::SI)--;
                else
                    reg(Reg16::SI)++;

                cyclesExecuted(12);
            }
            break;
        }
        case 0xAD: // LODS word
        {
            auto segment = segmentOverride == Reg16::AX ? Reg16::DS : segmentOverride;
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX))
                {
                    // TODO: interrupt

                    auto addr = (reg(segment) << 4) + reg(Reg16::SI);
                    reg(Reg16::AX) = mem.read(addr) | mem.read(addr + 1) << 8;

                    if(flags & Flag_D)
                        reg(Reg16::SI) -= 2;
                    else
                        reg(Reg16::SI) += 2;

                    reg(Reg16::CX)--;
                    cyclesExecuted(13 + 4);
                }
            }
            else
            {
                auto addr = (reg(segment) << 4) + reg(Reg16::SI);
                reg(Reg16::AX) = mem.read(addr) | mem.read(addr + 1) << 8;

                if(flags & Flag_D)
                    reg(Reg16::SI) -= 2;
                else
                    reg(Reg16::SI) += 2;

                cyclesExecuted(12 + 4);
            }
            break;
        }

        case 0xAF: // SCAS word
        {
            if(rep)
            {
                cyclesExecuted(2 + 9);

                while(reg(Reg16::CX) && (flags & Flag_Z))
                {
                    // TODO: interrupt

                    auto addr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);

                    uint16_t rSrc = mem.read(addr) | mem.read(addr + 1) << 8;

                    doSub(reg(Reg16::AX), rSrc, flags);

                    if(flags & Flag_D)
                        reg(Reg16::DI) -= 2;
                    else
                        reg(Reg16::DI) += 2;

                    reg(Reg16::CX)--;
                    cyclesExecuted(15 + 4);
                }
            }
            else
            {
                auto addr = (reg(Reg16::ES) << 4) + reg(Reg16::DI);

                uint16_t rSrc = mem.read(addr) | mem.read(addr + 1) << 8;

                doSub(reg(Reg16::AX), rSrc, flags);

                if(flags & Flag_D)
                    reg(Reg16::DI) -= 2;
                else
                    reg(Reg16::DI) += 2;

                reg(Reg16::CX)--;

                cyclesExecuted(15 + 4);
            }
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

        case 0xCD: // INT
        {
            auto imm = mem.read(addr + 1);
            reg(Reg16::IP)++;
            serviceInterrupt(imm);
            break;
        }

        case 0xCF: // IRET
        {
            // pop IP
            auto stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);
            reg(Reg16::SP) += 2;
            auto newIP = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;

            // pop CS
            stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);
            reg(Reg16::SP) += 2;
            auto newCS = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;

            // pop flags
            stackAddr = (reg(Reg16::SS) << 4) + reg(Reg16::SP);
            reg(Reg16::SP) += 2;
            flags = mem.read(stackAddr) | mem.read(stackAddr + 1) << 8;

            reg(Reg16::CS) = newCS;
            reg(Reg16::IP) = newIP;
            cyclesExecuted(32 + 3 * 4);
            break;
        }

        case 0xD7: // XLAT
        {
            auto addr = (reg(Reg16::BX) + reg(Reg8::AL)) & 0xFFFF;
            if(segmentOverride != Reg16::AX)
                addr += reg(segmentOverride) << 4;
            else
                addr += reg(Reg16::DS) << 4;

            reg(Reg8::AL) = mem.read(addr);
            cyclesExecuted(11);
            break;
        }

        case 0xE0: // LOOPNE/LOOPNZ
        {
            auto off = static_cast<int8_t>(mem.read(addr + 1));

            uint16_t count = --reg(Reg16::CX);

            if(count == 0 && !(flags & Flag_Z))
            {
                // done
                reg(Reg16::IP)++;
                cyclesExecuted(5);
            }
            else
            {
                reg(Reg16::IP) = reg(Reg16::IP) + 1 + off;
                cyclesExecuted(19);
            }
            break;
        }
        case 0xE1: // LOOPE/LOOPZ
        {
            auto off = static_cast<int8_t>(mem.read(addr + 1));

            uint16_t count = --reg(Reg16::CX);

            if(count == 0 && (flags & Flag_Z))
            {
                // done
                reg(Reg16::IP)++;
                cyclesExecuted(6);
            }
            else
            {
                reg(Reg16::IP) = reg(Reg16::IP) + 1 + off;
                cyclesExecuted(18);
            }
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
        case 0xE3: // JCXZ
        {
            auto off = static_cast<int8_t>(mem.read(addr + 1));
            if(reg(Reg16::CX) == 0)
            {
                reg(Reg16::IP) = reg(Reg16::IP) + 1 + off;
                cyclesExecuted(18);
            }
            else
            {
                reg(Reg16::IP)++;
                cyclesExecuted(6);
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

        case 0xF5: // CMC
        {
            flags ^= Flag_C;
            cyclesExecuted(2);
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
    cyclesExecuted(51 + 5 * 4); // timing for INT
}
