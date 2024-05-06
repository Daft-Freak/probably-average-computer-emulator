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

void MemoryBus::setBASICROM(const uint8_t *rom)
{
    basicROM = rom;
}

uint8_t MemoryBus::read(uint32_t addr) const
{
    if(addr < 0x10000)
        return ram[addr];

    if(addr >= 0xB8000 && addr < 0xBC000)
        return cga.ram[addr & 0x3FFF];

    if(addr >= 0xF6000 && addr < 0xFE000 && basicROM)
        return basicROM[addr - 0xF6000];

    if(addr >= 0xFE000)
        return biosROM[addr & 0x1FFF];

    return 0xFF;
}

void MemoryBus::write(uint32_t addr, uint8_t data)
{
    if(addr < 0x10000)
        ram[addr] = data;
    else if(addr >= 0xB8000 && addr < 0xBC000)
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
            {
                if(ppi.output[1] & 0x80)
                {
                    // switches
                    return 0 << 0 | // no floppy drives
                           0 << 1 | // no co-processor
                           3 << 2 | // 4 memory banks
                           1 << 4 | // 40-col CGA
                           0 << 6;  // still no floppy drives
                }
                else
                    printf("PPI A keyboard\n");
            }
            else
                return ppi.output[0];

            break;
        }

        case 0x62: // PPI port C
        {
            uint8_t ret = 0;

            if(ppi.mode & (1 << 0)) // input (lower)
            {
                if(ppi.output[1] & (1 << 2))
                {
                    // SW2 1-4
                }
                else
                {
                    // SW2 5, other bits read as high
                    ret = 0xE;
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
                        pit.active |= (1 << channel);
                        pit.counter[channel] = pit.reload[channel];

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
            ppi.output[port] = data;
            break;
        }
        case 0x63: // PPI control
        {
            if(data & 0x80) // mode set
            {
                auto modeA = (data >> 5) & 3;
                auto modeB = (data >> 2) & 1;
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

        default:
            printf("IO W %04X = %02X\n", addr, data);
    }
}

void MemoryBus::updateForInterrupts()
{
    // TODO: usual target for optimisation...

    // timer
    if(!(pic.mask & 1))
        updatePIT();
}

void MemoryBus::updateForDisplay()
{
    updateCGA();
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

            int mode = (pit.control[i] >> 1) & 7;

            if(mode == 2 && pit.counter[i] == 1)
                pit.counter[i] = pit.reload[i]; // reload after reaching 1 on the last cycle
            else
                pit.counter[i]--; // mode 3 decrements twice

            if(mode == 0 && pit.counter[i] == 0 && !(pit.outState & (1 << i)))
            {
                // go high
                pit.outState |= 1 << i;
            
                // ch0 should trigger interrupt here
                if(i == 0)
                    flagPICInterrupt(0);
            }
        }

        pit.lastUpdateCycle += 4;
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
                cga.curAddr = cga.regs[12] | cga.regs[13] << 8;
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
            }
            else
            {
                // text mode
                // assuming 8x8 chars...
                auto charAddr = cga.curAddr + (cga.scanlineCycle / 8) * 2;
                auto ch = cga.ram[charAddr];
                auto attr = cga.ram[charAddr + 1];

                auto fontData = cgaFont[ch * 8 + (cga.scanline & 7)];
                auto col = (fontData & 1 << (cga.scanlineCycle & 7)) ? attr & 0xF : (attr >> 4) & 7;

                if(cga.scanlineCycle & 1)
                    cga.scanlineBuf[cga.scanlineCycle / 2] |= col << 4;
                else
                    cga.scanlineBuf[cga.scanlineCycle / 2] = col;
            }
        }
    }
}
