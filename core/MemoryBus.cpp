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

uint8_t MemoryBus::read(uint32_t addr) const
{
    return 0;
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    return nullptr;
}

void MemoryBus::write(uint32_t addr, uint8_t data)
{
}