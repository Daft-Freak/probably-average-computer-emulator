#pragma once
#include <cstdint>

class System;

class CPU final
{
public:

    CPU(System &sys);

    void reset();

    void run(int ms);

    uint32_t getCycleCount() const {return cycleCount;}

    enum class Reg8
    {
        AL = 0,
        CL,
        DL,
        BL,
        AH,
        CH,
        DH,
        BH,
    };

    enum class Reg16
    {
        AX = 0,
        CX,
        DX,
        BX,

        // index registers
        SP,
        BP,
        SI,
        DI,

        // program counter
        IP,

        // segment registers
        ES,
        CS,
        SS,
        DS,
    };

    uint8_t reg(Reg8 r) const {return reinterpret_cast<const uint8_t *>(regs)[((static_cast<int>(r) & 3) << 1) + (static_cast<int>(r) >> 2)];}
    uint8_t &reg(Reg8 r) {return reinterpret_cast<uint8_t *>(regs)[((static_cast<int>(r) & 3) << 1) + (static_cast<int>(r) >> 2)];}
    uint16_t reg(Reg16 r) const {return regs[static_cast<int>(r)];}
    uint16_t &reg(Reg16 r) {return regs[static_cast<int>(r)];}

    void executeInstruction();

private:
    uint16_t readMem16(uint16_t offset, uint32_t segment);
    void writeMem16(uint16_t offset, uint32_t segment, uint16_t data);

    void cyclesExecuted(int cycles);

    void serviceInterrupt(uint8_t vector);

    static const uint32_t clockSpeed = 4772726;

    // internal state
    int cyclesToRun = 0;
    uint32_t cycleCount = 0;

    // registers
    uint16_t regs[13];
    uint16_t flags;

    // enabling interrupts happens one opcode later
    bool delayInterrupt = false;

    // RAM
    System &sys;
};
