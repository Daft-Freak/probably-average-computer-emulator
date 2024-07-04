#include <cassert>
#include <cstdio>

#include "FixedDiskAdapter.h"

FixedDiskAdapter::FixedDiskAdapter(System &sys) : sys(sys)
{
    sys.addIODevice(0x320, 0x32F, this);
}

void FixedDiskAdapter::setIOInterface(FixedDiskIO *io)
{
    this->io = io;
}

uint8_t FixedDiskAdapter::read(uint16_t addr)
{
    switch(addr)
    {
        case 0x320: // read
        {
            auto ret = responseLen ? data[responseOffset++] : 0xFF;

            // when all bytes read
            if(responseOffset == responseLen)
            {
                responseOffset = responseLen = 0;

                // clear interrupt
                status &= ~(1 << 5);

                status &= ~(1 << 1 | 1 << 3); // clear IO mode, busy
                // and bus?
            }
            return ret;
        }

        case 0x321: // status
        {
            auto ret = status;

            // this is a hack so that the "init characteristics" command works
            if(commandDataOffset < commandDataLen)
                status |= (1 << 0); // set request
            return ret;
        }

        case 0x322: // drive settings?
            break;

        default:
            printf("FXD R %04X @~%04X\n", addr, sys.getCPU().reg(CPU::Reg16::IP));
    }
    return 0xFF;
}

void FixedDiskAdapter::write(uint16_t addr, uint8_t data)
{
    switch(addr)
    {
        case 0x320: // write
        {
            if(commandDataOffset < commandDataLen)
                this->data[commandDataOffset++] = data;
            else if(controlBlockOffset < 6)
                controlBlock[controlBlockOffset++] = data;
            
            if(controlBlockOffset == 6 && commandDataOffset == commandDataLen)
            {
                // always clear this
                // it needs to be set again before the BIOS will send the rest of the "init characteristics" data though...
                status &= ~(1 << 0); // clear request

                if(!commandDataLen)
                {
                    if(controlBlock[0] == 0x0C/*init characteristics*/)
                    {
                        commandDataLen = 8; // has an extra 8 bytes
                        break;
                    }
                }

                // now we really have all the data

                status |= (1 << 5); // interrupt?
                if(dmaIntrMask & 2)
                    sys.flagPICInterrupt(5);

                int drive = (controlBlock[1] >> 5) & 1;
                int head = controlBlock[1] & 0x1F;
                int sector = controlBlock[2] & 0x3F;
                int cylinder = controlBlock[3] | (controlBlock[2] & 0xC0) << 2;
    
                const int sectorsPerTrack = 17;
                const int sectorSize = 512;
                bool failed = false;

                auto outData = this->data;
                responseLen = 1;

                 // don't clear sense if reading it
                if(controlBlock[0] != 0x03)
                    sense[0] = 0;

                if(controlBlock[0] == 0x00) // test drive ready
                {}
                else if(controlBlock[0] == 0x01) // recalibrate
                {
                    if(!io || !io->isPresent(drive))
                    {
                        failed = true;
                        sense[0] |= 6; // didn't reach track 0
                    }
                }
                else if(controlBlock[0] == 0x03) // request sense
                {
                    // copy sense data
                    for(auto &b : sense)
                        *outData++ = b;

                    responseLen = 5;
                }
                else if(controlBlock[0] == 0x08) // read
                {
                    // transfers data through DMA...
                    // super-hack
                    
                    auto &dma = sys.dma;
                    auto dmaSize = dma.currentWordCount[3] + 1;
                    auto destAddr = dma.currentAddress[3];
                    auto destHigh = dma.highAddr[3] << 16;

                    auto lba = ((cylinder * numHeads[drive] + head) * sectorsPerTrack) + sector;

                    while(dmaSize && (dmaIntrMask & 1))
                    {
                        uint8_t buf[512];

                        if(!io || !io->read(drive, buf, lba))
                        {
                            failed = true;
                            break;
                        }

                        for(int i = 0; i < sectorSize; i++)
                            sys.writeMem(destHigh + destAddr + i, buf[i]);

                        dmaSize -= sectorSize;
                        destAddr += sectorSize;
                        lba++;
                    }

                    sense[0] |= 1 << 7; // address valid
                    
                    // set an error if failed
                    // TODO: set different errors for no drive and read fail
                    if(failed)
                        sense[0] |= 4; // not ready

                }
                else if(controlBlock[0] == 0x0A) // write
                {
                    // hack the second
                    auto &dma = sys.dma;
                    auto dmaSize = dma.currentWordCount[3] + 1;
                    auto srcAddr = dma.currentAddress[3];
                    auto destHigh = dma.highAddr[3] << 16;

                    auto lba = ((cylinder * numHeads[drive] + head) * sectorsPerTrack) + sector;

                    while(dmaSize && (dmaIntrMask & 1))
                    {
                        uint8_t buf[512];

                        for(int i = 0; i < sectorSize; i++)
                            buf[i] = sys.readMem(destHigh + srcAddr + i);

                        if(!io || !io->write(drive, buf, lba))
                        {
                            failed = true;
                            break;
                        }

                        dmaSize -= sectorSize;
                        srcAddr += sectorSize;
                        lba++;
                    }

                    sense[0] |= 1 << 7; // address valid
                    
                    // set an error if failed
                    // TODO: set different errors for no drive and read fail
                    if(failed)
                        sense[0] |= 4; // not ready
                }
                else if(controlBlock[0] == 0x0C) // init characteristics
                {
                    // manual says byte 1 is ignored, BIOS suggests otherwise...
                    numCylinders[drive] = this->data[0] << 8 | this->data[1];
                    numHeads[drive] = this->data[2];
                }
                else if(controlBlock[0] == 0x0F) // write to sector buffer
                {} // TODO?
                else if(controlBlock[0] == 0xE0) // RAM diagnostic
                {}
                else if(controlBlock[0] == 0xE4) // controller diagnostic
                {}
                else
                {
                    failed = true;
                    sense[0] |= 0x20; // invalid command
                    printf("FXD %02X (+%i)\n", controlBlock[0], commandDataOffset);
                }

                // needs to be set after "init characteristics"
                status |= (1 << 1); // IO mode

                status &= ~(1 << 0); // clear request

                controlBlockOffset = 0;
                commandDataOffset = commandDataLen = 0;

                responseOffset = 0;
                *outData = (controlBlock[1] & (1 << 5)) // drive number for status
                         | (failed ? (1 << 1) : 0);

                // copy data for sense
                sense[1] = controlBlock[1];
                sense[2] = controlBlock[2];
                sense[3] = controlBlock[3];
            }
            break;
        }
        case 0x321: // reset
        {
            controlBlockOffset = commandDataOffset = responseOffset = 0;
            commandDataLen = responseLen = 0;
            status = 0;
            dmaIntrMask = 0;
            break;
        }

        case 0x322: //controller select
        {
            // BIOS expects these bits
            status = 1 << 3/*busy*/ | 1 << 2/*bus*/ | 1 << 0/*req*/;
            break;
        }

        case 0x323: // dma/interrupt mask
        {
            dmaIntrMask = data;
            break;
        }

        // these don't seem to do anything, but the BIOS likes to write to them
        case 0x327:
        case 0x32B:
        case 0x32F:
            break;
        
        default:
            printf("FXD W %04X = %02X @~%04X\n", addr, data, sys.getCPU().reg(CPU::Reg16::IP));
    }
}
