#pragma once
#include <cstdint>
#include <functional>
#include <list>

#include "CPU.h"
#include "FIFO.h"

class IODevice
{
public:
    virtual uint8_t read(uint16_t addr) = 0;
    virtual void write(uint16_t addr, uint8_t data) = 0;
};

class System
{
public:
    using ScanlineCallback = void(*)(const uint8_t *data, int line, int w);

    // reads a 512 byte sector
    // TODO: this may end up being an IO interface class or something
    using FloppyReadCallback = void(*)(uint8_t *, uint8_t, uint8_t, uint8_t, uint8_t);

    System();
    void reset();

    CPU &getCPU() {return cpu;}

    void addMemory(uint32_t base, uint32_t size, uint8_t *ptr);
    // TODO: a const version somehow?

    void addIODevice(uint16_t min, uint16_t max, IODevice *dev);

    uint8_t readMem(uint32_t addr) const;
    void writeMem(uint32_t addr, uint8_t data);

    const uint8_t *mapAddress(uint32_t addr) const;

    uint8_t readIOPort(uint16_t addr);
    void writeIOPort(uint16_t addr, uint8_t data);

    void flagPICInterrupt(int index);

    void updateForInterrupts();
    void updateForDisplay();

    bool hasInterrupt() const {return pic.request & ~pic.mask;}
    uint8_t acknowledgeInterrupt();

    void setCGAScanlineCallback(ScanlineCallback cb);

    void setFloppyReadCallback(FloppyReadCallback cb);

    void sendKey(uint8_t scancode);

    bool hasSpeakerSample() const;
    int8_t getSpeakerSample();

private:
    struct IORange
    {
        uint16_t min, max;
        IODevice *dev;
    };

    void updatePIT();
    void updateSpeaker(uint32_t target);

    void updateCGA();

    CPU cpu;

    static const int maxAddress = 1 << 20;
    static const int blockSize = 16 * 1024;

    uint8_t *memMap[maxAddress / blockSize];

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

        uint8_t highAddr[4];
    };

    struct PIC
    {
        uint8_t initCommand[4];
        int nextInit = 0;

        uint8_t request = 0;
        uint8_t service = 0;
        uint8_t mask = 0;

        uint8_t statusRead = 0;
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
        uint16_t scanline = 0;
        uint16_t scanlineCycle = 0;
        uint16_t curAddr = 0;
        uint16_t frame = 0;
        uint8_t scanlineBuf[320];

        uint8_t ram[16 * 1024]; // at B8000

        ScanlineCallback scanCb;
    };

    // also a card
    struct FDC
    {
        uint8_t digitalOutput = 0;

        uint8_t status[4] = {0, 0, 0, 0};
        uint8_t presentCylinder[4];

        uint8_t command[9];
        uint8_t result[7];
        uint8_t commandLen = 0, resultLen = 0;
        uint8_t commandOff, resultOff;

        uint8_t readyChanged;

        FloppyReadCallback readCb = nullptr;
    };

    DMA dma;

    PIC pic;

    PIT pit;

    PPI ppi;

    CGA cga;

    FDC fdc;

    std::vector<IORange> ioDevices;

    FIFO<uint8_t, 8> keyboardQueue;
    uint32_t keyboardClockLowCycle = 0;
    uint32_t keyboardTestReplyCycle = 0;
    int keyboardTestDelay = 0; // need to delay sending back test result

    uint32_t lastSpeakerUpdateCycle = 0;
    uint32_t speakerSampleTimer = 0;
    FIFO<int8_t, 1024> speakerQueue; // somewhat unsafe
};