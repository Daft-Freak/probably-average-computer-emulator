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

    void setBIOSROM(const uint8_t *rom);

    uint8_t read(uint32_t addr) const;
    void write(uint32_t addr, uint8_t data);

    const uint8_t *mapAddress(uint32_t addr) const;

    uint8_t readIOPort(uint16_t addr);
    void writeIOPort(uint16_t addr, uint8_t data);

    void updateForInterrupts();

    bool hasInterrupt() const {return pic.request & ~pic.mask;}
    uint8_t acknowledgeInterrupt();

private:
    void flagPICInterrupt(int index);
    void updatePIT();

    void updateCGA();

    uint8_t ram[64 * 1024];

    const uint8_t *biosROM = nullptr; // at 0xFE000

    struct DMA
    {
        uint16_t baseAddress[4];
        uint16_t baseWordCount[4];
        uint16_t currentAddress[4];
        uint16_t currentWordCount[4];

        uint16_t tempAddress;
        uint16_t tempWordCount;
        uint8_t tempData;

        uint8_t status;
        uint8_t command;
        uint8_t request;

        uint8_t mode[4];

        uint8_t mask = 0xF;

        bool flipFlop = false;
    };

    struct PIC
    {
        uint8_t initCommand[4];
        int nextInit = 0;

        uint8_t request = 0;
        uint8_t service = 0;
        uint8_t mask = 0;
    };

    struct PIT
    {
        uint8_t control[3]{0, 0, 0};
        uint8_t active = 0;

        uint16_t counter[3];
        uint16_t reload[3];
        uint16_t latch[3];

        uint8_t latched = 0;
        uint8_t highByte = 0; // lo/hi access mode

        uint8_t outState = 0;

        uint32_t lastUpdateCycle = 0;
    };

    struct PPI
    {
        uint8_t mode = 0;
        uint8_t output[3];
    };

    // an entire separate card...
    struct CGA
    {
        // 6845 registers
        uint8_t regSelect;
        uint8_t regs[18];

        uint8_t mode = 0;
        uint8_t colSelect;
        uint8_t status = 0;

        uint32_t lastUpdateCycle = 0;
        uint8_t scanline = 0;
        uint16_t scanlineCycle = 0;
    };

    DMA dma;

    PIC pic;

    PIT pit;

    PPI ppi;

    CGA cga;

    CPU &cpu;
};