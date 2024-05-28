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


    DMA dma;

    PIC pic;

    PIT pit;

    PPI ppi;

    std::vector<IORange> ioDevices;

    FIFO<uint8_t, 8> keyboardQueue;
    uint32_t keyboardClockLowCycle = 0;
    uint32_t keyboardTestReplyCycle = 0;
    int keyboardTestDelay = 0; // need to delay sending back test result

    uint32_t lastSpeakerUpdateCycle = 0;
    uint32_t speakerSampleTimer = 0;
    FIFO<int8_t, 1024> speakerQueue; // somewhat unsafe

    // because this is a giant pile of hacks, it needs to poke around in the DMA controller
    // FIXME: real DMA, remove this
    friend class FloppyController;
};