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

void MemoryBus::write(uint32_t addr, uint8_t data)
{
    if(addr < 0x10000)
        ram[addr] = data;
}