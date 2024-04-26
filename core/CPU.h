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
    enum class Reg8
    {
        AL = 0,
        AH,
        BL,
        BH,
        CL,
        CH,
        DL,
        DH,
    };

    enum class Reg16
    {
        AX = 0,
        BX,
        CX,
        DX,

        // index registers
        SI,
        DI,
        BP,
        SP,

        // program counter
        IP,

        // segment registers
        CS,
        DS,
        ES,
        SS,
    };

    uint8_t reg(Reg8 r) const {return reinterpret_cast<const uint8_t *>(regs)[static_cast<int>(r)];}
    uint8_t &reg(Reg8 r) {return reinterpret_cast<uint8_t *>(regs)[static_cast<int>(r)];}
    uint16_t reg(Reg16 r) const {return regs[static_cast<int>(r)];}
    uint16_t &reg(Reg16 r) {return regs[static_cast<int>(r)];}

    void executeInstruction();

    void cycleExecuted();

    static const uint32_t clockSpeed = 4772726;

    // internal state
    int cyclesToRun = 0;
    uint32_t cycleCount = 0;

    // registers
    uint16_t regs[13];
    uint16_t status;

    // RAM
    MemoryBus mem;
};
