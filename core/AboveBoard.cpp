#include <cstdio>

#include "AboveBoard.h"

// EEPROM data

// word 0 is related to bus/timings?, serial port(i, s) and parallel port(p)
// ?????iis sspp????
//
// ii 0-3 map to IRQ2,5,4,3
//
// sss = 000 = none
// sss = 001 = 2E8
// sss = 011 = 2F8
// sss = 101 = 3E8
// sss = 111 = 3F8
//
// pp = 00 = none
// pp = 01 = 378
// pp = 10 = 278

// word 1 seems to be base addr(B), backfill(b) and extended ram(e,E)
// bb?BBBee eeeEEEEE
// base addr 0-7 map to 208,218,248,258,???,2A8,2B8,2E8
// bb = 00 = none
// bb = 11 = 512-640

// word 2 gets set to FFFF a lot

// word 3 only gets set if 8-bit bus?

// word 6 is usually FFFF

// words 8-18 are the serial number with the high byte as the inverse of the low byte
// word 19 is a checksum of the serial number
// set to "DaftRAM1234" here

// word 63 needs to be AAxx, xx is possibly related to bus settings

AboveBoard::AboveBoard(System &sys) : sys(sys)
{
    // "base address" can be one of 208, 218, 248, 258, 2A8, 2B8 or 2E8
    // the page mapping registers seem to be at xx0-xx7 though
    // this is configured in the EEPROM
    sys.addIODevice(0x3F0, 0x250, this);
}

uint8_t AboveBoard::read(uint16_t addr)
{
    if((addr & 0x300F) <= 8)
    {
        // mapping readback
        auto page = ((addr & 0xF) + 6) << 16 | (addr & 0xF000);
        int index = (page - 0xC0000) / 0x4000;

        if(index >= 0 && index < 8)
            return pageMapping[index];
    }

    switch(addr)
    {
        case 0x259:
            // part of board detection
            // | 0x18 is "Matched Memory Classic" (if we return | 0x18 for 025F below)
            // | 0x20 seems to mean the board is configured for a 16-bit bus
            return 0x18;

        case 0x25F:
            // software expects bottom three bits to read back as +3 from what was written
            // some of the other bits determine board type
            // | 0x18 is "Above Board Plus 8" or "Matched Memory Classic"
            // | 0x10 is "Above Board Plus"
            // ! 0x08 is invalid? (EMM.SYS fails)
            // | 0x00 is also "Above Board Plus"?
            return (detectF & 0xE0) | ((detectF + 3) & 7) | 0x18;

        // EEPROM access
        case 0xC25C:
            return eepromData & 0xFF;
        case 0xC25D:
            return eepromData >> 8;

        default:
            printf("AB R %04X @~%04X\n", addr, sys.getCPU().reg(CPU::Reg16::IP));
    }
    return 0xFF;
}

void AboveBoard::write(uint16_t addr, uint8_t data)
{
    if((addr & 0xF) <= 8)
    {
        // page mapping registers
        auto page = ((addr & 0xF) + 6) << 16 | (addr & 0xC000);

        // don't map (or more importantly, UNMAP) pages we don't control
        if(page < 0xC0000)
            return;

        int bit = (page - 0xC0000) / 0x4000;

        pageMapping[bit] = data;

        // register is too small for the last 4, assume enabled?
        if(bit < 8 && !(pageMask & (1 << bit)))
            return;

        if(data & 0x80)
        {
            // the upper bits are in the address
            auto index = (data & 0x7F) | (addr & 0x3000) >> 5;
            
            size_t offset = index * 16 * 1024;

            printf("AB map %04X = %02X\n", page, index);

            // unmap on out-of-bounds page
            if(offset >= sizeof(ram))
                sys.removeMemory(page / System::getMemoryBlockSize());
            else
                sys.addMemory(page, 16 * 1024, ram + offset);
        }
        else
        {
            printf("AB unmap %04X\n", page);
            sys.removeMemory(page / System::getMemoryBlockSize());
        }

        return;
    }

    switch(addr)
    {
        case 0x25F:
            detectF = data;
            break;

        case 0x125F:
            // looks like an enable mask for pages C000-DC00?
            printf("AB page mask %02X\n", data);
            pageMask = data;
            break;

        // EEPROM access
        case 0xC25C:
            eepromData = (eepromData & 0xFF00) | data;
            break;
        case 0xC25D:
            eepromData = (eepromData & 0xFF) | data << 8;
            break;
        case 0xC25E:
        {
            int cmd = data >> 6;
            int addr = data & 0x3F;

            if(cmd == 2) // read
                eepromData = eeprom[addr];
            else if(cmd == 3) // write
                eeprom[addr] = eepromData;
            else
                printf("AB EEPROM %i? %02X\n", cmd, addr);
            
            break;
        }

        default:
            printf("AB W %04X = %02X @~%04X\n", addr, data, sys.getCPU().reg(CPU::Reg16::IP));
    }
}
