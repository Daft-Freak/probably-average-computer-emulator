#pragma once
#include <cstdint>
#include <tuple>

class System;

class CPU final
{
public:

    CPU(System &sys);

    void reset();

    void run(int ms);

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

    uint16_t getFlags() const {return flags;}
    void setFlags(uint16_t flags) {this->flags = flags;}

    void executeInstruction();

private:
    uint16_t readMem16(uint16_t offset, uint32_t segment);
    void writeMem16(uint16_t offset, uint32_t segment, uint16_t data);

    std::tuple<uint16_t, uint32_t> getEffectiveAddress(int mod, int rm, int &cycles, bool rw, uint32_t addr);

    // R/M helpers

    uint8_t readRM8(uint8_t modRM, int &cycles, uint32_t addr);
    uint16_t readRM16(uint8_t modRM, int &cycles, uint32_t addr);

    void writeRM8(uint8_t modRM, uint8_t v, int &cycles, uint32_t addr, bool rw = false);
    void writeRM16(uint8_t modRM, uint16_t v, int &cycles, uint32_t addr, bool rw = false);

    // ALU helpers
    using ALUOp8 = uint8_t(*)(uint8_t, uint8_t, uint16_t &);
    using ALUOp16 = uint16_t(*)(uint16_t, uint16_t, uint16_t &);

    template<ALUOp8 op, bool d, int regCycles, int memCycles>
    void doALU8(uint32_t addr);
    template<ALUOp16 op, bool d, int regCycles, int memCycles>
    void doALU16(uint32_t addr);

    template<ALUOp8 op>
    void doALU8AImm(uint32_t addr);
    template<ALUOp16 op>
    void doALU16AImm(uint32_t addr);

    void cyclesExecuted(int cycles);

    void serviceInterrupt(uint8_t vector);

    // internal state
    int cyclesToRun = 0;

    // registers
    uint16_t regs[13];
    uint16_t flags;

    // enabling interrupts happens one opcode later
    bool delayInterrupt = false;

    Reg16 segmentOverride;

    // RAM
    System &sys;
};
