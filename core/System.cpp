#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

#include "System.h"

System::System() : cpu(*this)
{
}

void System::reset()
{
    memset(memDirty, 0, sizeof(memDirty));
    memset(memReadOnly, 0, sizeof(memReadOnly));
    cpu.reset();
}

void System::addMemory(uint32_t base, uint32_t size, uint8_t *ptr)
{
    assert(size % blockSize == 0);
    assert(base % blockSize == 0);
    assert(base + size <= maxAddress);

    auto block = base / blockSize;
    int numBlocks = size / blockSize;

    for(int i = 0; i < numBlocks; i++)
        memMap[block + i] = ptr - base;
}

void System::addReadOnlyMemory(uint32_t base, uint32_t size, const uint8_t *ptr)
{
    assert(size % blockSize == 0);
    assert(base % blockSize == 0);
    assert(base + size <= maxAddress);

    auto block = base / blockSize;
    int numBlocks = size / blockSize;

    for(int i = 0; i < numBlocks; i++)
    {
        memMap[block + i] = const_cast<uint8_t *>(ptr) - base;
    
        memReadOnly[(block + i) / 32] |= 1 << ((block + i) % 32);
    }
}

void System::removeMemory(unsigned int block)
{
    assert(block < maxAddress / blockSize);
    memMap[block] = nullptr;
}

uint32_t *System::getMemoryDirtyMask()
{
    return memDirty;
}

bool System::getMemoryBlockDirty(unsigned int block) const
{
    return memDirty[block / 32] & (1 << (block % 32));
}

void System::clearMemoryBlockDirty(unsigned int block)
{
    memDirty[block / 32] &= ~(1 << (block % 32));
}

void System::setMemoryRequestCallback(MemRequestCallback cb)
{
    memReqCb = cb;
}

void System::addIODevice(uint16_t mask, uint16_t value, uint8_t picMask, IODevice *dev)
{
    ioDevices.emplace_back(IORange{mask, value, picMask, dev});
}

uint8_t System::readMem(uint32_t addr)
{
    addr &= (maxAddress - 1);

    auto block = addr / blockSize;
    
    auto ptr = memMap[block];

    // request more memory
    if(!ptr && memReqCb)
    {
        ptr = memReqCb(block);
        if(ptr)
            ptr = memMap[block] = ptr - block * blockSize;
    }

    if(ptr)
        return ptr[addr];

    return 0xFF;
}

void System::writeMem(uint32_t addr, uint8_t data)
{
    addr &= (maxAddress - 1);

    auto block = addr / blockSize;

    auto ptr = memMap[block];

    // request more memory
    if(!ptr && memReqCb)
    {
        ptr = memReqCb(block);
        if(ptr)
            ptr = memMap[block] = ptr - block * blockSize;
    }

    // no writing the ROM
    if(memReadOnly[block / 32] & (1 << (block % 32)))
        return;

    if(ptr)
        ptr[addr] = data;

    memDirty[block / 32] |= (1 << (block % 32));
}

const uint8_t *System::mapAddress(uint32_t addr) const
{
    return nullptr;
}

uint8_t System::readIOPort(uint16_t addr)
{
    if(addr < 0x100)
    {
        switch(addr)
        {
            case 0x00: // DMA channel 0 addr
            case 0x02: // DMA channel 1 addr
            case 0x04: // DMA channel 2 addr
            case 0x06: // DMA channel 3 addr
            {
                int channel = addr / 2;

                uint8_t ret;
                if(dma.flipFlop)
                    ret = dma.currentAddress[channel] >> 8;
                else
                    ret = dma.currentAddress[channel] & 0xFF;

                dma.flipFlop = !dma.flipFlop;

                // xt boot hack
                if(channel == 0 && !dma.flipFlop && !(dma.mask & 1))
                    dma.currentAddress[channel]++;
                //

                return ret;
            }
            case 0x01: // DMA channel 0 word count
            case 0x03: // DMA channel 1 word count
            case 0x05: // DMA channel 2 word count
            case 0x07: // DMA channel 3 word count
            {
                int channel = addr / 2;

                uint8_t ret;
                if(dma.flipFlop)
                    ret = dma.currentWordCount[channel] >> 8;
                else
                    ret = dma.currentWordCount[channel] & 0xFF;

                dma.flipFlop = !dma.flipFlop;

                return ret;
            }

            case 0x08: // DMA status
                return dma.status;

            case 0x20: // PIC request/service (OCW3)
                return pic.statusRead & 1 ? pic.service : pic.request;
            case 0x21: // PIC mask (OCW1)
                return pic.mask;
        
            case 0x40: // PIT counter 0
            case 0x41: // PIT counter 1
            case 0x42: // PIT counter 2
            {
                updatePIT();

                int channel = addr & 3;

                if(pit.control[channel])
                {
                    int access = (pit.control[channel] >> 4) & 3;

                    auto value = pit.latched & (1 << channel) ? pit.latch[channel] : pit.counter[channel];

                    uint8_t ret;

                    if(access == 1 || (access == 3 && !(pit.highByte & (1 << channel))))
                        ret = value & 0xFF;
                    else // access == 2 || (access == 3 && high byte)
                        ret = value >> 8;

                    // clear latch status if fully read
                    if(access != 3 || (pit.highByte & (1 << channel)))
                        pit.latched &= ~(1 << channel);

                    // flip hi/lo
                    if(access == 3)
                        pit.highByte ^= (1 << channel);

                    return ret;
                }
                break;
            }

            case 0x60: // PPI port A
            {
                if(ppi.mode & (1 << 4)) // input
                    return keyboardQueue.peek();
                else
                    return ppi.output[0];

                break;
            }
            case 0x61: // PPI port B
            {
                if(ppi.mode & (1 << 1)) // input
                {
                    printf("PPI B input\n");
                }
                else
                    return ppi.output[1];

                break;
            }
            case 0x62: // PPI port C
            {
                uint8_t ret = 0;

                if(ppi.mode & (1 << 0)) // input (lower)
                {
                    if(ppi.output[1] & (1 << 3))
                    {
                        // SW1 5-8
                        ret = 2 | 1 << 2; // 80-col CGA, two floppies
                    }
                    else
                    {
                        // SW1 1-4
                        ret = 1 << 0  // not test mode
                            | 0 << 1  // no co-processor
                            | 0 << 2; // 1 RAM bank
                    }
                }
                else
                    ret = ppi.output[2] & 0xF;

                if(ppi.mode & (1 << 3)) // input (upper)
                {
                    // cassette data, timer chan 2 and RAM errors
                }
                else
                    ret |= ppi.output[2] & 0xF0;

                return ret;
            }

            default:
                printf("IO R %04X\n", addr);
        }
    }
    else
    {
        for(auto & dev : ioDevices)
        {
            if((addr & dev.ioMask) == dev.ioValue)
                return dev.dev->read(addr);
        }
        printf("IO R %04X\n", addr);
    }

    return 0xFF;
}

void System::writeIOPort(uint16_t addr, uint8_t data)
{
    if(addr < 0x100)
    {
        switch(addr)
        {
            case 0x00: // DMA channel 0 addr
            case 0x02: // DMA channel 1 addr
            case 0x04: // DMA channel 2 addr
            case 0x06: // DMA channel 3 addr
            {
                int channel = addr / 2;

                if(dma.flipFlop)
                    dma.currentAddress[channel] = dma.baseAddress[channel] = (dma.baseAddress[channel] & 0xFF) | data << 8;
                else
                    dma.baseAddress[channel] = (dma.baseAddress[channel] & 0xFF00) | data;

                dma.flipFlop = !dma.flipFlop;
                break;
            }

            case 0x01: // DMA channel 0 word count
            case 0x03: // DMA channel 1 word count
            case 0x05: // DMA channel 2 word count
            case 0x07: // DMA channel 3 word count
            {
                int channel = addr / 2;

                if(dma.flipFlop)
                    dma.currentWordCount[channel] = dma.baseWordCount[channel] = (dma.baseWordCount[channel] & 0xFF) | data << 8;
                else
                    dma.baseWordCount[channel] = (dma.baseWordCount[channel] & 0xFF00) | data;

                dma.flipFlop = !dma.flipFlop;
                break;
            }

            case 0x08: // DMA command
                dma.command = data;
                break;
            case 0x09: // DMA request
            {
                int channel = data & 3;
                if(data & (1 << 2))
                    dma.request |= 1 << channel;
                else
                    dma.request &= ~(1 << channel);
                break;
            }
            case 0x0A: // DMA mask
            {
                int channel = data & 3;
                if(data & (1 << 2))
                    dma.mask |= 1 << channel;
                else
                {
                    dma.mask &= ~(1 << channel);
                
                    if(channel == 0)
                        dma.status |= 1; // report TC0 for XT BIOS
                }
                break;
            }
            case 0x0B: // DMA mode
            {
                int channel = data & 3;
                int dir = (data >> 2) & 3;
                bool autoInit = data & (1 << 4);
                bool dec = data & (1 << 5);
                int mode = data >> 6;

                static const char *dirStr[]{"verify", "write", "read", "ILLEGAL"};
                static const char *modeStr[]{"demand", "single", "block", "cascade"};

                printf("DMA ch%i %s%s %s %s\n", channel, autoInit ? "auto-init ": "", modeStr[mode], dirStr[dir], dec ? "decrement" : "increment");

                dma.mode[channel] = data;
                break;
            }
            case 0x0C: // DMA reset flip-flop
            {
                dma.flipFlop = false;
                break;
            }

            case 0x0D: // DMA master clear
            {
                dma.command = 0;
                dma.status = 0;
                dma.request = 0;
                dma.tempData = 0;
                dma.flipFlop = false;
                dma.mask = 0xF;
                break;
            }

            case 0x20: // PIC ICW1, OCW 2/3
            {
                if(data & (1 << 4)) // ICW1
                {
                    assert(data & (1 << 0)); // ICW4 needed
                    assert(data & (1 << 1)); // single
                    assert(!(data & (1 << 3))); // not level triggered

                    pic.initCommand[0] = data;
                    pic.nextInit = 1;

                    pic.mask = 0;
                    pic.statusRead = 2; // read IRR
                }
                else if(data & (1 << 3)) // OCW3
                {
                    assert(!(data & (1 << 7)));

                    if(data & 0x64)
                        printf("PIC OCW3 %02X\n", data);

                    if(data & 2)
                        pic.statusRead = (pic.statusRead & 0xFC) | (data & 3);
                }
                else // OCW2
                {
                    auto command = data >> 5;

                    switch(command)
                    {
                        case 1: // non-specific EOI
                        {
                            for(int i = 0; i < 8; i++)
                            {
                                if(pic.service & (1 << i))
                                {
                                    pic.service &= ~(1 << i);
                                    break;
                                }
                            }
                            break;
                        }
                        default:
                            printf("PIC OCW2 %02X\n", data);
                    }
                }

                break;
            }

            case 0x21: // PIC
            {
                if(pic.nextInit == 1) // ICW2
                {
                    pic.initCommand[1] = data;

                    if(!(pic.initCommand[0] & (1 << 1)))
                        pic.nextInit = 2; // ICW3 needed
                    else
                        pic.nextInit = 3; // ICW4 (assuming needed)
                }
                else if(pic.nextInit == 3) // ICW4
                {
                    assert(data & (1 << 0)); // 8086/88 mode
                    assert(!(data & (1 << 1))); // not auto EOI
                    assert(!(data & (1 << 2))); // slave
                    assert(data & (1 << 3)); // buffered mode
                    assert(!(data & (1 << 4))); // not special fully nested mode

                    pic.initCommand[3] = data;
                    pic.nextInit = 0;
                }
                else // mask
                {
                    auto enabled = pic.mask & ~data;

                    if(enabled)
                    {
                        // sync devices that are getting their IRQ unmasked
                        if(enabled & 1)
                            updatePIT();

                        for(auto &dev : ioDevices)
                        {
                            if(dev.picMask & enabled)
                                dev.dev->updateForInterrupts();
                        }
                    }

                    pic.mask = data;
                    calculateNextInterruptCycle(cpu.getCycleCount());
                }
                break;
            }

            case 0x40: // PIT counter 0
            case 0x41: // PIT counter 1
            case 0x42: // PIT counter 2
            {
                updatePIT();

                int channel = addr & 3;

                if(pit.control[channel])
                {
                    int access = (pit.control[channel] >> 4) & 3;

                    if(access == 1 || (access == 3 && !(pit.highByte & (1 << channel))))
                        pit.reload[channel] = (pit.reload[channel] & 0xFF00) | data;
                    else // access == 2 || (access == 3 && high byte)
                        pit.reload[channel] = (pit.reload[channel] & 0xFF) | data << 8;

                    // not active, wrote value
                    if(!(pit.active & (1 << channel)) && (access != 3 || (pit.highByte & (1 << channel))))
                    {
                        int mode = (pit.control[channel] >> 1) & 7;
                        // modes 1 and 5 start on gate input instead
                        if(mode != 1 && mode != 5)
                        {
                            if(channel == 2)
                                updateSpeaker(cpu.getCycleCount());

                            pit.active |= (1 << channel);
                            pit.counter[channel] = pit.reload[channel];

                            // round down odd count for mode 3
                            if(mode == 3 && (pit.reload[channel] & 1))
                                pit.counter[channel]--;

                            if(mode == 0) // mode 0 goes low immediately
                                pit.outState &= ~(1 << channel);
                            else // mode 2/3/4 start high
                                pit.outState |= (1 << channel);

                            calculateNextPITUpdate();
                            calculateNextInterruptCycle(cpu.getCycleCount());
                        }
                    }

                    // flip hi/lo
                    if(access == 3)
                        pit.highByte ^= (1 << channel);
                }
                break;
            }
            
            case 0x43: // PIT control
            {
                updatePIT();

                int channel = data >> 6;
                int access = (data >> 4) & 3;
                int mode = (data >> 1) & 7;

                if(channel == 3) // readback
                {
                    printf("PIT readback!\n");
                    return;
                }

                if(access == 0) // latch
                {
                    pit.latch[channel] = pit.counter[channel];
                    pit.latched |= (1 << channel);
                }
                else // set mode
                {
                    pit.control[channel] = data;
                    
                    // reset
                    pit.counter[channel] = pit.reload[channel] = 0;
                    pit.active &= ~(1 << channel);
                    pit.latched &= ~(1 << channel);
                    pit.highByte &= ~(1 << channel);

                    printf("PIT ch%i access %i mode %i\n", channel, access, mode);

                    calculateNextPITUpdate();
                    calculateNextInterruptCycle(cpu.getCycleCount());
                }

                break;
            }

            case 0x60: // PPI port A
            case 0x61: // PPI port B
            case 0x62: // PPI port C
            {
                int port = addr & 3;

                auto changed = ppi.output[port] ^ data;

                // timer gate or speaker output
                if(port == 1 && (changed & 3))
                {
                    updatePIT();
                    updateSpeaker(cpu.getCycleCount());
                }

                if(port == 1 && (changed & (1 << 7)))
                {
                    // keyboard data/irq clear
                    if(data & (1 << 7))
                        keyboardQueue.pop();
                    else if(!keyboardQueue.empty())
                        flagPICInterrupt(1);
                }

                if(port == 1 && (changed & (1 << 6)))
                {
                    // check for B6 going high (end of soft reset/self-test pulse)
                    if(data & (1 << 6))
                    {
                        // needs to be a long pulse (BIOS is going for 20ms)
                        if(cpu.getCycleCount() - keyboardClockLowCycle > 100000)
                        {
                            // send reply a little later
                            keyboardTestReplyCycle = cpu.getCycleCount();
                            keyboardTestDelay = 1000;
                            calculateNextInterruptCycle(cpu.getCycleCount());
                        }
                    }
                    else
                        keyboardClockLowCycle = cpu.getCycleCount();
                }

                ppi.output[port] = data;

                break;
            }
            case 0x63: // PPI control
            {
                if(data & 0x80) // mode set
                {
                    [[maybe_unused]] auto modeA = (data >> 5) & 3;
                    [[maybe_unused]] auto modeB = (data >> 2) & 1;
                    assert(!modeA);
                    assert(!modeB);

                    ppi.mode = data;
                }
                else // bit set/clear
                    printf("PPI control %02X\n", data);
                break;
            }

            case 0x81: // DMA channel 2 high addr
                dma.highAddr[2] = data;
                break;
            case 0x82: // DMA channel 3 high addr
                dma.highAddr[3] = data;
                break;
            case 0x83: // DMA channel 1 high addr
                dma.highAddr[1] = data;
                break;

            default:
                printf("IO W %04X = %02X\n", addr, data);
        }
    }
    else
    {
        for(auto & dev : ioDevices)
        {
            if((addr & dev.ioMask) == dev.ioValue)
                return dev.dev->write(addr, data);
        }
        printf("IO W %04X = %02X\n", addr, data);
    }
}

void System::flagPICInterrupt(int index)
{
    pic.request |= 1 << index;
}

void System::updateForInterrupts()
{
    // response from keyboard self-test
    if(keyboardTestDelay)
    {
        keyboardTestDelay -= cpu.getCycleCount() - keyboardTestReplyCycle;

        if(keyboardTestDelay <= 0)
        {
            keyboardTestDelay = 0;
            keyboardQueue.push(0xAA);
            flagPICInterrupt(1);
        }
        else
            keyboardTestReplyCycle = cpu.getCycleCount();
    }

    // timer
    if(!(pic.mask & 1))
    {
        auto passed = cpu.getCycleCount() - pit.lastUpdateCycle;
        if(passed >= pit.nextUpdateCycle - pit.lastUpdateCycle)
            updatePIT();
    }

    for(auto &dev : ioDevices)
    {
        if(dev.picMask & ~pic.mask)
            dev.dev->updateForInterrupts();
    }

    calculateNextInterruptCycle(cpu.getCycleCount());
}

void System::updateForDisplay()
{
    // PIT may update speaker, so we need to run that first
    updatePIT();
    updateSpeaker(cpu.getCycleCount());
}

void System::calculateNextInterruptCycle(uint32_t cycleCount)
{
    int toUpdate = std::numeric_limits<int>::max();

    // keyboard self-test
    if(keyboardTestDelay)
        toUpdate = std::min(toUpdate, static_cast<int>(keyboardTestDelay - (cycleCount - keyboardTestReplyCycle)));

    // timer
    if(!(pic.mask & 1))
        toUpdate = std::min(toUpdate, static_cast<int>(pit.nextUpdateCycle - cycleCount));

    for(auto &dev : ioDevices)
    {
        if(dev.picMask & ~pic.mask)
            toUpdate = std::min(toUpdate, dev.dev->getCyclesToNextInterrupt(cycleCount));
    }

    assert(toUpdate >= 0 || nextInterruptCycle == cycleCount + toUpdate);

    nextInterruptCycle = cycleCount + toUpdate;
}

uint8_t System::acknowledgeInterrupt()
{
    auto serviceable = pic.request & ~pic.mask;

    int serviceIndex;

    for(serviceIndex = 0; serviceIndex < 8; serviceIndex++)
    {
        if(serviceable & (1 << serviceIndex))
            break;
    }

    pic.service |= 1 << serviceIndex;
    pic.request &= ~(1 << serviceIndex);

    return serviceIndex | (pic.initCommand[1] & 0xF8);
}

void System::sendKey(uint8_t scancode)
{
    keyboardQueue.push(scancode);
    flagPICInterrupt(1);
}

bool System::hasSpeakerSample() const
{
    return !speakerQueue.empty();
}

int8_t System::getSpeakerSample()
{
    return speakerQueue.pop();
}

void System::updatePIT()
{
    auto elapsed = cpu.getCycleCount() - pit.lastUpdateCycle;

    elapsed /= 4; // PIT clock is four times slower than CPU clock

    while(elapsed)
    {
        int step = std::min(elapsed, (pit.nextUpdateCycle - pit.lastUpdateCycle) / 4);

        for(int i = 0; i < 3; i++)
        {
            if(pit.reloadNextCycle & (1 << i))
                pit.counter[i] = pit.reload[i]; // reload after reaching 1 on the last cycle
        }

        for(int i = 0; i < 3; i++)
        {
            if(!(pit.active & (1 << i)))
                continue;

            // ch2 gate
            if(i == 2 && !(ppi.output[1] & 1))
                continue;

            int mode = (pit.control[i] >> 1) & 7;

            if(mode == 3) // mode 3 decrements twice
                pit.counter[i] -= step * 2;
            else if(!(pit.reloadNextCycle & (1 << i)))
                pit.counter[i] -= step;

            if(mode == 0 && pit.counter[i] == 0 && !(pit.outState & (1 << i)))
            {
                // go high
                pit.outState |= 1 << i;
            
                // ch0 should trigger interrupt here
                if(i == 0)
                    flagPICInterrupt(0);
            }
            else if(mode == 2 && pit.counter[i] == 1)
                pit.reloadNextCycle |= 1 << i;
            else if(mode == 3 && pit.counter[i] == 0)
            {
                if(i == 2)
                    updateSpeaker(pit.lastUpdateCycle + (step - 1) * 4);

                // toggle out and reload
                // TODO: should delay low by one cycle if odd count
                pit.outState ^= 1 << i;
                pit.counter[i] = pit.reload[i] & ~1;

                if(i == 0 && (pit.outState & 1))
                    flagPICInterrupt(0);
            }
        }

        pit.lastUpdateCycle += step * 4;
        elapsed -= step;

        // recalculate next
        if(pit.lastUpdateCycle == pit.nextUpdateCycle || pit.reloadNextCycle)
            calculateNextPITUpdate();

        pit.reloadNextCycle = 0;
    }
}

void System::calculateNextPITUpdate()
{
    // find first channel to trigger
    int step = 0xFFFF;
    for(int i = 0; i < 3; i++)
    {
        if(!(pit.active & (1 << i)))
            continue;

        int mode = (pit.control[i] >> 1) & 7;

        int remaining = pit.counter[i];

        if(mode == 2)
            remaining--; // count to 1
        else if(mode == 3)
            remaining /= 2; // double-decrement

        if(remaining > 0 && remaining < step)
            step = remaining;
    }

    pit.nextUpdateCycle = pit.lastUpdateCycle + step * 4;
}

void System::updateSpeaker(uint32_t target)
{
    static const int fracBits = 8;
    static const int sampleRate = 22050;
    static const int divider = (4772726 << fracBits) / sampleRate;

    target &= ~3; // avoid getting ahead of PIT

    auto elapsed = target - lastSpeakerUpdateCycle;
    lastSpeakerUpdateCycle = target;

    assert(elapsed < 0xFFFFFF);

    speakerSampleTimer += elapsed << fracBits;

    bool gate = ppi.output[1] & (1 << 0);
    bool ppiData = ppi.output[1] & (1 << 1);
    bool pitData = pit.outState & (1 << 2);

    // gate low makes timer output high
    // FIXME: that should be handled in the PIT... and gate should also stop timer counting
    bool value = !((pitData || !gate) && ppiData);

    while(speakerSampleTimer >= divider)
    {
        speakerSampleTimer -= divider;

        while(speakerQueue.full()); // wait

        speakerQueue.push(value ? 127 : -128);
    }
}
