#include <cassert>
#include <cstdio>
#include <cstring>

#include "MemoryBus.h"
#include "CPU.h"
#include "CGAFont.h"

MemoryBus::MemoryBus(CPU &cpu) : cpu(cpu)
{
}

void MemoryBus::reset()
{

}

void MemoryBus::setBIOSROM(const uint8_t *rom)
{
    biosROM = rom;
}

uint8_t MemoryBus::read(uint32_t addr) const
{
    if(addr < 0x10000)
        return ram[addr];

    if(addr >= 0xB8000 && addr < 0xC0000)
        return cga.ram[addr & 0x3FFF];

    if(addr >= 0xF0000)
        return biosROM[addr & 0xFFFF];

    return 0xFF;
}

void MemoryBus::write(uint32_t addr, uint8_t data)
{
    if(addr < 0x10000)
        ram[addr] = data;
    else if(addr >= 0xB8000 && addr < 0xC0000)
        cga.ram[addr & 0x3FFF] = data;
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    return nullptr;
}

uint8_t MemoryBus::readIOPort(uint16_t addr)
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
            if(channel == 0 && !dma.flipFlop)
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
                    ret = 2 | 0 << 2; // 80-col CGA, one floppy
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

        case 0x3DA: // CGA status
        {
            updateCGA();
            return cga.status;
        }

        case 0x3F4: // floppy main status
        {
            // no drives busy, dma-mode
            return 0 |
                   (fdc.resultLen ? 1 << 4 : 0) | // fdc busy if there's a result to read
                   (fdc.resultLen ? 1 << 6 : 0) | // fdc->cpu if we have result data else cpu->fdc
                   1 << 7; // ready
        }
        case 0x3F5: // floppy command/data
        {
            if(fdc.resultLen)
            {
                auto ret = fdc.result[fdc.resultOff++];

                // end of result
                if(fdc.resultOff == fdc.resultLen)
                    fdc.resultLen = 0;

                return ret;
            }

            return 0xFF;
        }

        default:
            printf("IO R %04X\n", addr);
    }
    return 0xFF;
}

void MemoryBus::writeIOPort(uint16_t addr, uint8_t data)
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
        {
            dma.command = data;
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
            dma.status = 1; // report TC0 for XT BIOS
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
                pic.mask = data;
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
        
        case 0x3D4: // CGA reg select
        {
            cga.regSelect = data;
            break;
        }
        case 0x3D5: // CGA reg
        {
            updateCGA();
            cga.regs[cga.regSelect] = data;
            break;
        }

        case 0x3D8: // CGA mode
        {
            updateCGA();
            cga.mode = data;
            break;
        }
        case 0x3D9: // CGA colour select
        {
            updateCGA();
            cga.colSelect = data;
            break;
        }

        case 0x3F2: // floppy digital output
        {
            auto changed = fdc.digitalOutput ^ data;

            if((changed & (1 << 2)) && (data & (1 << 2)))
            {
                // leaving reset
                // because RDY is high, this generates an interrupt
                if(data & (1 << 3))
                    flagPICInterrupt(6);

                fdc.readyChanged = 0xF; // all of them
            }

            fdc.digitalOutput = data;
            break;
        }

        case 0x3F5: // floppy command/data
        {
            if(fdc.commandLen == 0)
            {
                fdc.command[0] = data;

                if(data == 0x03) // specify
                    fdc.commandLen = 3;
                else if(data == 0x04) // sense drive status
                    fdc.commandLen = 2;
                else if((data & 0x1F) == 0x06) // read
                    fdc.commandLen = 9;
                else if(data == 0x07) // recalibrate
                    fdc.commandLen = 2;
                else if(data == 0x08) // sense interrupt status
                    fdc.commandLen = 1;
                else if(data == 0x0F)
                    fdc.commandLen = 3;
                else
                    printf("FCD = %02X\n", data);

                if(fdc.commandLen)
                    fdc.commandOff = 1;
            }
            else
                fdc.command[fdc.commandOff++] = data;

            if(fdc.commandLen && fdc.commandOff == fdc.commandLen)
            {
                // got full command
                if(fdc.command[0] == 0x03) // specify
                {
                    // auto stepRateTime = fdc.command[1] >> 4;
                    // auto headUnloadTime = fdc.command[1] & 0xF;
                    // auto headLoadTime = fdc.command[2] >> 1;
                    // bool nonDMA = fdc.command[2] & 1;
                }
                else if(fdc.command[0] == 0x04) // sense drive status
                {
                    int unit = fdc.command[1] & 3;
                    int head = (fdc.command[1] >> 2) & 1;

                    bool track0 = fdc.presentCylinder[unit] == 0;

                    fdc.resultLen = 1;
                    fdc.result[0] = (track0 ? 1 << 4 : 0) | 1 << 5 /*ready*/;
                }
                else if((fdc.command[0] & 0x1F) == 0x06) // read
                {
                    // multitrack mfm skip
                    [[maybe_unused]] bool multiTrack = fdc.command[0] & (1 << 7);
                    [[maybe_unused]] bool mfm = fdc.command[0] & (1 << 6);
                    // bool skipDeleted = fdc.command[0] & (1 << 5);

                    int unit = fdc.command[1] & 3;
                    int head = (fdc.command[1] >> 2) & 1;

                    auto cylinder = fdc.command[2];
                    [[maybe_unused]] auto headAgain = fdc.command[3];
                    auto record = fdc.command[4];
                    auto number = fdc.command[5];
                    auto endOfTrack = fdc.command[6];
                    //auto gapLength = fdc.command[7];
                    //auto dataLength = fdc.command[8];

                    assert(head == headAgain);
                    assert(number == 2);
                    assert(multiTrack);
                    assert(mfm);

                    auto sectorSize = 128 << number;

                    // transfers data through DMA...
                    // super-hack
                    auto dmaSize = dma.currentWordCount[2] + 1;
                    auto destAddr = dma.currentAddress[2]; // + high byte from port 0x81!
                    while(dmaSize)
                    {
                        uint8_t buf[512];

                        if(fdc.readCb)
                            fdc.readCb(buf, cylinder, head, record, endOfTrack);
                        // should probably fail the read otherwise...

                        for(int i = 0; i < sectorSize; i++)
                            write(destAddr + i, buf[i]);

                        dmaSize -= sectorSize;
                        destAddr += sectorSize;

                        // update offset
                        record++;
                        if(record > endOfTrack)
                        {
                            record = 1;
                            if(head == 0)
                                head = 1;
                            else
                            {
                                head = 0;
                                cylinder++;
                            }
                        }
                    }

                    // FIXME: if !auto else reload
                    dma.currentAddress[2] = destAddr;
                    dma.currentWordCount[2] = 0;
                    // resets every time anyway...

                    fdc.status[0] = unit | head << 2;

                    fdc.resultLen = 7;
                    fdc.result[0] = fdc.status[0];
                    fdc.result[1] = fdc.status[1];
                    fdc.result[2] = fdc.status[2];
                    fdc.result[3] = cylinder;
                    fdc.result[4] = head;
                    fdc.result[5] = record;
                    fdc.result[6] = number;

                    if(fdc.digitalOutput & (1 << 3))
                        flagPICInterrupt(6);
                }
                else if(fdc.command[0] == 0x07) // recalibrate
                {
                    int unit = fdc.command[1] & 3;

                    fdc.status[0] = unit;

                    if(!fdc.readCb || unit != 0)
                        fdc.status[0] |= 1 << 6 | 1 << 4; // abnormal termination/equipment check
                    else
                    {
                        fdc.presentCylinder[unit] = 0;
                        fdc.status[0] |= 1 << 5; // set seek end
                    }

                    if(fdc.digitalOutput & (1 << 3))
                        flagPICInterrupt(6);
                }
                else if(fdc.command[0] == 0x08) // sense interrupt status
                {
                    if(fdc.readyChanged)
                    {
                        int unit = 0;
                        while(!(fdc.readyChanged & 1 << unit))
                            unit++;

                        fdc.readyChanged &= ~(1 << unit);

                        fdc.status[0] |= 0xC0 | unit;
                    }

                    fdc.result[0] = fdc.status[0];
                    fdc.result[1] = fdc.presentCylinder[fdc.status[0] & 3];
                    fdc.resultLen = 2;

                    fdc.status[0] = 0; // clear status
                }
                else if(fdc.command[0] == 0x0F) // seek
                {
                    int unit = fdc.command[1] & 3;
                    // int head = (fdc.command[1] >> 2) & 1;
                    auto cylinder = fdc.command[2];

                    fdc.presentCylinder[unit] = cylinder;

                    // set seek end
                    fdc.status[0] = 1 << 5 | unit;

                    if(fdc.digitalOutput & (1 << 3))
                        flagPICInterrupt(6);
                }

                fdc.commandLen = 0;
                fdc.resultOff = 0;
            }

            break;
        }

        default:
            printf("IO W %04X = %02X\n", addr, data);
    }
}

void MemoryBus::updateForInterrupts()
{
    // TODO: usual target for optimisation...

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
        updatePIT();
}

void MemoryBus::updateForDisplay()
{
    updateCGA();

    // PIT may update speaker, so we need to run that first
    updatePIT();
    updateSpeaker(cpu.getCycleCount());
}

uint8_t MemoryBus::acknowledgeInterrupt()
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

void MemoryBus::setCGAScanlineCallback(ScanlineCallback cb)
{
    cga.scanCb = cb;
}

void MemoryBus::setFloppyReadCallback(FloppyReadCallback cb)
{
    fdc.readCb = cb;
}

void MemoryBus::sendKey(uint8_t scancode)
{
    keyboardQueue.push(scancode);
    flagPICInterrupt(1);
}

bool MemoryBus::hasSpeakerSample() const
{
    return !speakerQueue.empty();
}

int8_t MemoryBus::getSpeakerSample()
{
    return speakerQueue.pop();
}

void MemoryBus::flagPICInterrupt(int index)
{
    pic.request |= 1 << index;
}

void MemoryBus::updatePIT()
{
    auto elapsed = cpu.getCycleCount() - pit.lastUpdateCycle;

    elapsed /= 4; // PIT clock is four times slower than CPU clock

    while(elapsed--)
    {
        for(int i = 0; i < 3; i++)
        {
            if(!(pit.active & (1 << i)))
                continue;

            // TODO: ch2 gate

            int mode = (pit.control[i] >> 1) & 7;

            if(mode == 2 && pit.counter[i] == 1)
                pit.counter[i] = pit.reload[i]; // reload after reaching 1 on the last cycle
            else if(mode == 3) // mode 3 decrements twice
                pit.counter[i] -= 2;
            else
                pit.counter[i]--;

            if(mode == 0 && pit.counter[i] == 0 && !(pit.outState & (1 << i)))
            {
                // go high
                pit.outState |= 1 << i;
            
                // ch0 should trigger interrupt here
                if(i == 0)
                    flagPICInterrupt(0);
            }
            else if(mode == 3 && pit.counter[i] == 0)
            {
                if(i == 2)
                    updateSpeaker(pit.lastUpdateCycle);

                // toggle out and reload
                // TODO: should delay low by one cycle if odd count
                pit.outState ^= 1 << i;
                pit.counter[i] = pit.reload[i] & ~1;

                if(i == 0 && (pit.outState & 1))
                    flagPICInterrupt(0);
            }
        }

        pit.lastUpdateCycle += 4;
    }
}

void MemoryBus::updateSpeaker(uint32_t target)
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

void MemoryBus::updateCGA()
{
    auto elapsed = cpu.getCycleCount() - cga.lastUpdateCycle;

    elapsed *= 3; // system clock

    // 80-col mode uses full system clock, other modes use half
    if(!(cga.mode & (1 << 0)))
        elapsed /= 2;
    
    // FIXME: this loses a cycle sometimes in 40-col mode
    cga.lastUpdateCycle = cpu.getCycleCount();

    int lineClocks = (cga.regs[0/*h total*/] + 1) * 8;
    int hDisplayed = cga.regs[1/* h disp*/] * 8;
    int hBlankStart = cga.regs[2/*h sync*/] * 8;

    int charHeight = cga.regs[9/*max char scan*/] + 1;
    int totalLines = (cga.regs[4/* v total*/] + 1) * charHeight + cga.regs[5 /*v adjust*/];
    int vDisplayed = cga.regs[6/*v displayed*/] * charHeight;
    int vBlankStart = cga.regs[7/*v sync*/] * charHeight;

    uint16_t cursorAddr = cga.regs[14] << 8 | cga.regs[15];

    while(elapsed--)
    {
        cga.scanlineCycle++;

        if(cga.scanlineCycle >= lineClocks)
        {
            // display line
            if(cga.scanline < vDisplayed && cga.scanCb)
                cga.scanCb(cga.scanlineBuf, cga.scanline, hDisplayed);

            cga.scanlineCycle = 0;
            cga.scanline++;

            if((cga.scanline % charHeight) == 0)
                cga.curAddr += cga.regs[1/* h disp*/] * 2;

            cga.status &= ~(1 << 0); // clear accessible

            // check new scanline
            if(cga.scanline >= totalLines)
            {
                cga.scanline = 0;
                cga.frame++;
                cga.curAddr = cga.regs[12] << 8 | cga.regs[13];
                cga.status &= ~(1 << 0 | 1 << 3); // clear accessible / vblank
            }
            else if(cga.scanline >= vBlankStart)
                cga.status |= 1 << 0 | 1 << 3; // accessible / vblank
        }
        else if(cga.scanlineCycle >= hBlankStart)
            cga.status |= 1 << 0; // accessible

        if(cga.scanlineCycle < hDisplayed && cga.scanline < vDisplayed)
        {
            // in visible area

            if(!(cga.mode & (1 << 3))) // check enabled
                cga.scanlineBuf[cga.scanlineCycle / 2] = 0; // black
            else if(cga.mode & (1 << 1))
            {
                // graphics mode
                if(cga.mode & (1 << 4))
                {
                    // hi-res
                }
                else
                {
                    int palIndex = (cga.colSelect >> 5) & 1;
                    bool bright = (cga.colSelect & (1 << 4));
                    auto bg = cga.colSelect & 0xF;

                    auto charAddr = cga.curAddr + (cga.scanlineCycle / 4);
                    if(cga.scanline & 1)
                        charAddr += 0x2000;

                    auto data = cga.ram[charAddr];
                    auto col = (data << ((cga.scanlineCycle & 3) * 2) >> 6) & 3;

                    if(col == 0)
                        col = bg;
                    else
                    {
                        // palette mapping is just shifting up 1 bit, palette select is the low bit
                        // TODO: mixed palette if b/w bit set
                        col = (col << 1) | palIndex | (bright ? 8 : 0);
                    }

                    if(cga.scanlineCycle & 1)
                        cga.scanlineBuf[cga.scanlineCycle / 2] |= col << 4;
                    else
                        cga.scanlineBuf[cga.scanlineCycle / 2] = col;
                }
            }
            else
            {
                // text mode
                // assuming 8x8 chars...
                auto charAddr = cga.curAddr + (cga.scanlineCycle / 8) * 2;
                auto ch = cga.ram[charAddr];
                auto attr = cga.ram[charAddr + 1];

                int charLine = cga.scanline & 7;

                // check if in cursor
                // for more accuracy, should toggle when reaching those lines (resulting in wrap around sometimes)
                bool cursor = charAddr == cursorAddr * 2 && charLine >= (cga.regs[10/*cursor start*/] & 0x1F) && charLine <= (cga.regs[11/*cursor end*/] & 0x1F);

                int col;

                // also check for blinking (handled outside 6845, 8/8 frames)
                if(cursor && (cga.frame & 8))
                    col = attr & 0xF;
                else
                {
                    // not cursor or cursor off
                    auto fontData = cgaFont[ch * 8 + charLine];
                    col = (fontData & 1 << (cga.scanlineCycle & 7)) ? attr & 0xF : (attr >> 4) & 7;

                    // blink character
                    if((attr & 0x80) && !(cga.frame & 16))
                        col = (attr >> 4) & 7;
                }

                if(cga.scanlineCycle & 1)
                    cga.scanlineBuf[cga.scanlineCycle / 2] |= col << 4;
                else
                    cga.scanlineBuf[cga.scanlineCycle / 2] = col;
            }
        }
    }
}
