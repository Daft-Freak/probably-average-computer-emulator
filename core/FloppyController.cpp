#include <cassert>
#include <cstdio>

#include "FloppyController.h"

FloppyController::FloppyController(System &sys) : sys(sys)
{
    // technically generates IRQ6, but not in a way that requires updateForInterrupts (yet?)
    sys.addIODevice(0x3F8, 0x3F0, 0, this);
}

void FloppyController::setIOInterface(FloppyDiskIO *io)
{
    this->io = io;
}

uint8_t FloppyController::read(uint16_t addr)
{
    switch(addr)
    {
        case 0x3F4: // floppy main status
        {
            // no drives busy, dma-mode
            return 0 |
                   (resultLen ? 1 << 4 : 0) | // fdc busy if there's a result to read
                   (resultLen ? 1 << 6 : 0) | // fdc->cpu if we have result data else cpu->fdc
                   1 << 7; // ready
        }
        case 0x3F5: // floppy command/data
        {
            if(resultLen)
            {
                auto ret = result[resultOff++];

                // end of result
                if(resultOff == resultLen)
                    resultLen = 0;

                return ret;
            }

            return 0xFF;
        }

        case 0x3F7: // floppy digital input
            return 0; // TODO, but this makes 8088_bios boot

    }
    return 0xFF;
}

void FloppyController::write(uint16_t addr, uint8_t data)
{
    switch(addr)
    {
        case 0x3F2: // floppy digital output
        {
            auto changed = digitalOutput ^ data;

            if((changed & (1 << 2)) && (data & (1 << 2)))
            {
                // leaving reset
                // because RDY is high, this generates an interrupt
                if(data & (1 << 3))
                    sys.flagPICInterrupt(6);

                readyChanged = 0xF; // all of them
            }

            digitalOutput = data;
            break;
        }

        case 0x3F5: // floppy command/data
        {
            if(commandLen == 0)
            {
                command[0] = data;

                if(data == 0x03) // specify
                    commandLen = 3;
                else if(data == 0x04) // sense drive status
                    commandLen = 2;
                else if((data & 0x1F) == 0x06) // read
                    commandLen = 9;
                else if(data == 0x07) // recalibrate
                    commandLen = 2;
                else if(data == 0x08) // sense interrupt status
                    commandLen = 1;
                else if((data & 0x1F) == 0x0a) // read id
                    commandLen = 2;
                else if(data == 0x0F)
                    commandLen = 3;
                else
                    printf("FCD = %02X\n", data);

                if(commandLen)
                    commandOff = 1;
            }
            else
                command[commandOff++] = data;

            if(commandLen && commandOff == commandLen)
            {
                // got full command
                if(command[0] == 0x03) // specify
                {
                    // auto stepRateTime = command[1] >> 4;
                    // auto headUnloadTime = command[1] & 0xF;
                    // auto headLoadTime = command[2] >> 1;
                    // bool nonDMA = command[2] & 1;
                }
                else if(command[0] == 0x04) // sense drive status
                {
                    int unit = command[1] & 3;
                    // int head = (command[1] >> 2) & 1;

                    bool track0 = presentCylinder[unit] == 0;

                    resultLen = 1;
                    result[0] = (track0 ? 1 << 4 : 0) | 1 << 5 /*ready*/;
                }
                else if((command[0] & 0x1F) == 0x06) // read
                {
                    // multitrack mfm skip
                    [[maybe_unused]] bool multiTrack = command[0] & (1 << 7);
                    [[maybe_unused]] bool mfm = command[0] & (1 << 6);
                    // bool skipDeleted = command[0] & (1 << 5);

                    int unit = command[1] & 3;
                    int head = (command[1] >> 2) & 1;

                    auto cylinder = command[2];
                    [[maybe_unused]] auto headAgain = command[3];
                    auto record = command[4];
                    auto number = command[5];
                    auto endOfTrack = command[6];
                    //auto gapLength = command[7];
                    //auto dataLength = command[8];

                    assert(head == headAgain);
                    assert(number == 2);
                    assert(multiTrack);
                    assert(mfm);

                    auto sectorSize = 128 << number;

                    // transfers data through DMA...
                    // super-hack
                    bool failed = false;
                    auto &dma = sys.dma;
                    auto dmaSize = dma.currentWordCount[2] + 1;
                    auto destAddr = dma.currentAddress[2];
                    auto destHigh = dma.highAddr[2] << 16;
                    while(dmaSize)
                    {
                        uint8_t buf[512];

                        if(!io || !io->read(unit, buf, cylinder, head, record))
                        {
                            failed = true;
                            break;
                        }

                        for(int i = 0; i < sectorSize; i++)
                            sys.writeMem(destHigh + destAddr + i, buf[i]);

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

                    status[0] = unit | head << 2;

                    if(failed)
                        status[0] |= 1 << 6;

                    resultLen = 7;
                    result[0] = status[0];
                    result[1] = status[1];
                    result[2] = status[2];
                    result[3] = cylinder;
                    result[4] = head;
                    result[5] = record;
                    result[6] = number;

                    if(digitalOutput & (1 << 3))
                        sys.flagPICInterrupt(6);
                }
                else if(command[0] == 0x07) // recalibrate
                {
                    int unit = command[1] & 3;

                    status[0] = unit;

                    if(!io || !io->isPresent(unit))
                        status[0] |= 1 << 6 | 1 << 4; // abnormal termination/equipment check
                    else
                    {
                        presentCylinder[unit] = 0;
                        status[0] |= 1 << 5; // set seek end
                    }

                    if(digitalOutput & (1 << 3))
                        sys.flagPICInterrupt(6);
                }
                else if(command[0] == 0x08) // sense interrupt status
                {
                    if(readyChanged)
                    {
                        int unit = 0;
                        while(!(readyChanged & 1 << unit))
                            unit++;

                        readyChanged &= ~(1 << unit);

                        status[0] = 0xC0 | unit;
                    }

                    result[0] = status[0];
                    result[1] = presentCylinder[status[0] & 3];
                    resultLen = 2;

                    status[0] = 0; // clear status
                }
                else if((command[0] & 0x1F) == 0x0a) // read id
                {
                    [[maybe_unused]] bool mfm = command[0] & (1 << 6);

                    int unit = command[1] & 3;
                    int head = (command[1] >> 2) & 1;

                    assert(mfm);

                    status[0] = unit | head << 2;

                    resultLen = 7;
                    result[0] = status[0];
                    result[1] = status[1];
                    result[2] = status[2];
                    result[3] = presentCylinder[unit];
                    result[4] = head;
                    result[5] = 1;
                    result[6] = 2; // ?

                    if(digitalOutput & (1 << 3))
                        sys.flagPICInterrupt(6);
                }
                else if(command[0] == 0x0F) // seek
                {
                    int unit = command[1] & 3;
                    // int head = (command[1] >> 2) & 1;
                    auto cylinder = command[2];

                    status[0] = unit;

                    if(!io || !io->isPresent(unit))
                        status[0] |= 1 << 6 | 1 << 4; // abnormal termination/equipment check
                    else
                    {
                        presentCylinder[unit] = cylinder;

                        // set seek end
                        status[0] |= 1 << 5;
                    }

                    if(digitalOutput & (1 << 3))
                        sys.flagPICInterrupt(6);
                }

                commandLen = 0;
                resultOff = 0;
            }

            break;
        }
    }
}
