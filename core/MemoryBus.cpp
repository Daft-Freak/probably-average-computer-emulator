#include <cstdio>
#include <cstring>

#include "MemoryBus.h"
#include "CPU.h"

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

    if(addr >= 0xFE000)
        return biosROM[addr & 0x1FFF];

    return 0;
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    return nullptr;
}

uint8_t MemoryBus::readIOPort(uint16_t addr)
{
    switch(addr)
    {
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
        default:
            printf("IO R %04X\n", addr);
    }
    return 0;
}

void MemoryBus::writeIOPort(uint16_t addr, uint8_t data)
{
    switch(addr)
    {
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
                    if(mode == 2 || mode == 3 || mode == 4)
                    {
                        pit.active |= (1 << channel);
                        pit.counter[channel] = pit.reload[channel];
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

        default:
            printf("IO W %04X = %02X\n", addr, data);
    }
}

void MemoryBus::updatePIT()
{
    auto elapsed = cpu.getCycleCount() - pit.lastUpdateCycle;

    elapsed /= 4; // PIT clock is four times slower than CPU clock

    while(elapsed--)
    {
        for(int i = 0; i < 2; i++)
        {
            if(!(pit.active & (1 << i)))
                continue;

            int mode = (pit.control[i] >> 1) & 7;

            if(mode == 2 && pit.counter[i] == 1)
                pit.counter[i] = pit.reload[i]; // reload after reaching 1 on the last cycle
            else
                pit.counter[i]--; // mode 3 decrements twice
        }

        pit.lastUpdateCycle += 4;
    }
}

void MemoryBus::write(uint32_t addr, uint8_t data)
{
    if(addr < 0x10000)
        ram[addr] = data;
}