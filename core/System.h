#pragma once
#include <cstdint>
#include <functional>
#include <list>

#include "CPU.h"
#include "FIFO.h"
#include "Scancode.h"

class IODevice
{
public:
    virtual uint8_t read(uint16_t addr) = 0;
    virtual void write(uint16_t addr, uint8_t data) = 0;

    virtual void updateForInterrupts() = 0;
    virtual int getCyclesToNextInterrupt(uint32_t cycleCount) = 0;
};

class System
{
public:
    using MemRequestCallback = uint8_t *(*)(unsigned int block);
    using SpeakerAudioCallback = void(*)(int8_t sample);

    using MemReadCallback = uint8_t(*)(uint32_t addr, void *);
    using MemWriteCallback = void(*)(uint32_t addr, uint8_t data, void *);

    enum class GraphicsConfig
    {
        MDA = 0,
        CGA_80Col,
        CGA_40Col,
        Other
    };

    System();
    void reset();

    CPU &getCPU() {return cpu;}

    uint32_t getCycleCount() const {return cycleCount;}

    void addMemory(uint32_t base, uint32_t size, uint8_t *ptr);
    void addReadOnlyMemory(uint32_t base, uint32_t size, const uint8_t *ptr);

    void removeMemory(unsigned int block);

    uint32_t *getMemoryDirtyMask();
    bool getMemoryBlockDirty(unsigned int block) const;
    void setMemoryBlockDirty(unsigned int block);
    void clearMemoryBlockDirty(unsigned int block);

    void setMemoryRequestCallback(MemRequestCallback cb);
    MemRequestCallback getMemoryRequestCallback() const;

    void setMemAccessCallbacks(uint32_t baseAddr, uint32_t size, MemReadCallback readCb, MemWriteCallback writeCb, void *userData = nullptr);

    void addIODevice(uint16_t mask, uint16_t value, uint8_t picMask, IODevice *dev);
    void removeIODevice(IODevice *dev);

    void setGraphicsConfig(GraphicsConfig config);

    uint8_t readMem(uint32_t addr);
    void writeMem(uint32_t addr, uint8_t data);

    const uint8_t *mapAddress(uint32_t addr) const;

    uint8_t readIOPort(uint16_t addr);
    void writeIOPort(uint16_t addr, uint8_t data);

    void flagPICInterrupt(int index);

    void addCPUCycles(int cycles)
    {
        cycleCount += cycles * cpuClkDiv;
    }

    void updateForInterrupts();
    void updateForDisplay();

    void calculateNextInterruptCycle(uint32_t cycleCount);
    uint32_t getNextInterruptCycle() const {return nextInterruptCycle;}

    bool hasInterrupt() const {return pic.request & ~pic.mask;}
    uint8_t acknowledgeInterrupt();

    void sendKey(XTScancode scancode, bool down);

    void setSpeakerAudioCallback(SpeakerAudioCallback cb);

    static constexpr int getClockSpeed() {return systemClock;}
    static constexpr int getCPUClockSpeed() {return systemClock / cpuClkDiv;}

    static constexpr int getMemoryBlockSize() {return blockSize;}
    static constexpr int getNumMemoryBlocks() {return maxAddress / blockSize;}

private:
    struct IORange
    {
        uint16_t ioMask, ioValue;
        uint8_t picMask;
        IODevice *dev;
    };

    void updatePIT();
    void calculateNextPITUpdate();
    void updateSpeaker(uint32_t target);

    // clocks
    static constexpr int systemClock = 14318180;
    static constexpr int cpuClkDiv = 3; // 4.7727MHz
    static constexpr int periphClkDiv = 6; // 2.38637MHz
    static constexpr int pitClkDiv = periphClkDiv * 2; // 1.19318MHz

    CPU cpu;

    uint32_t cycleCount = 0;

    static const int maxAddress = 1 << 20;
    static const int blockSize = 16 * 1024;

    uint8_t *memMap[maxAddress / blockSize];
    uint32_t memDirty[maxAddress / blockSize / 32];
    uint32_t memReadOnly[maxAddress / blockSize / 32];

    MemRequestCallback memReqCb = nullptr;

    uint32_t memAccessCbBase, memAccessCbEnd;
    MemReadCallback memReadCb = nullptr;
    MemWriteCallback memWriteCb = nullptr;
    void *memAccessUserData;

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
        uint8_t reloadNextCycle = 0;

        uint32_t lastUpdateCycle = 0;
        uint32_t nextUpdateCycle = 0;
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

    uint32_t nextInterruptCycle = 0;

    FIFO<uint8_t, 8> keyboardQueue;
    uint32_t keyboardClockLowCycle = 0;
    uint32_t keyboardTestReplyCycle = 0;
    int keyboardTestDelay = 0; // need to delay sending back test result

    uint32_t lastSpeakerUpdateCycle = 0;
    uint32_t speakerSampleTimer = 0;
    SpeakerAudioCallback speakerCb = nullptr;

    GraphicsConfig graphicsConfig = GraphicsConfig::CGA_80Col;

    // because this is a giant pile of hacks, it needs to poke around in the DMA controller
    // FIXME: real DMA, remove this
    friend class FloppyController;
    friend class FixedDiskAdapter;
};