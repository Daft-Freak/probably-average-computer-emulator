#include "CGACard.h"

#include "CGAFont.h"

CGACard::CGACard(System &sys) : sys(sys)
{
    sys.addMemory(0xB8000, sizeof(ram), ram);
    sys.addMemory(0xBC000, sizeof(ram), ram); // mirror
    sys.addIODevice(0x3D4, 0x3DA, this);
}

void CGACard::setScanlineCallback(ScanlineCallback cb)
{
    scanCb = cb;
}

void CGACard::update()
{
    auto elapsed = sys.getCPU().getCycleCount() - lastUpdateCycle;

    elapsed *= 3; // system clock

    // 80-col mode uses full system clock, other modes use half
    if(!(mode & (1 << 0)))
        elapsed /= 2;
    
    // FIXME: this loses a cycle sometimes in 40-col mode
    lastUpdateCycle = sys.getCPU().getCycleCount();

    int lineClocks = (regs[0/*h total*/] + 1) * 8;
    int hDisplayed = regs[1/* h disp*/] * 8;

    int charHeight = regs[9/*max char scan*/] + 1;
    int totalLines = (regs[4/* v total*/] + 1) * charHeight + regs[5 /*v adjust*/];
    int vDisplayed = regs[6/*v displayed*/] * charHeight;
    int vBlankStart = regs[7/*v sync*/] * charHeight;

    uint16_t cursorAddr = regs[14] << 8 | regs[15];

    while(elapsed--)
    {
        scanlineCycle++;

        if(scanlineCycle >= lineClocks)
        {
            // display line
            if(scanline < vDisplayed && scanCb)
                scanCb(scanlineBuf, scanline, hDisplayed);

            scanlineCycle = 0;
            scanline++;

            if((scanline % charHeight) == 0)
                curAddr += regs[1/* h disp*/] * 2;

            // check new scanline
            if(scanline >= totalLines)
            {
                scanline = 0;
                frame++;
                curAddr = regs[12] << 8 | regs[13];
                status &= ~(1 << 3); // clear vblank
            }
            else if(scanline >= vBlankStart)
                status |=  1 << 3; // vblank
        }

        if(scanlineCycle < hDisplayed && scanline < vDisplayed)
        {
            // in visible area

            if(!(mode & (1 << 3))) // check enabled
                scanlineBuf[scanlineCycle / 2] = 0; // black
            else if(mode & (1 << 1))
            {
                // graphics mode
                if(mode & (1 << 4))
                {
                    // hi-res
                }
                else
                {
                    int palIndex = (colSelect >> 5) & 1;
                    bool bright = (colSelect & (1 << 4));
                    auto bg = colSelect & 0xF;

                    auto charAddr = curAddr + (scanlineCycle / 4);
                    if(scanline & 1)
                        charAddr += 0x2000;

                    auto data = ram[charAddr];
                    auto col = (data << ((scanlineCycle & 3) * 2) >> 6) & 3;

                    if(col == 0)
                        col = bg;
                    else
                    {
                        // palette mapping is just shifting up 1 bit, palette select is the low bit
                        // TODO: mixed palette if b/w bit set
                        col = (col << 1) | palIndex | (bright ? 8 : 0);
                    }

                    if(scanlineCycle & 1)
                        scanlineBuf[scanlineCycle / 2] |= col << 4;
                    else
                        scanlineBuf[scanlineCycle / 2] = col;
                }
            }
            else
            {
                // text mode
                // assuming 8x8 chars...
                auto charAddr = curAddr + (scanlineCycle / 8) * 2;
                auto ch = ram[charAddr];
                auto attr = ram[charAddr + 1];

                int charLine = scanline & 7;

                // check if in cursor
                // for more accuracy, should toggle when reaching those lines (resulting in wrap around sometimes)
                bool cursor = charAddr == cursorAddr * 2 && charLine >= (regs[10/*cursor start*/] & 0x1F) && charLine <= (regs[11/*cursor end*/] & 0x1F);

                int col;

                // also check for blinking (handled outside 6845, 8/8 frames)
                if(cursor && (frame & 8))
                    col = attr & 0xF;
                else
                {
                    // not cursor or cursor off
                    auto fontData = cgaFont[ch * 8 + charLine];
                    col = (fontData & 1 << (scanlineCycle & 7)) ? attr & 0xF : (attr >> 4) & 7;

                    // blink character
                    if((attr & 0x80) && !(frame & 16))
                        col = (attr >> 4) & 7;
                }

                if(scanlineCycle & 1)
                    scanlineBuf[scanlineCycle / 2] |= col << 4;
                else
                    scanlineBuf[scanlineCycle / 2] = col;
            }
        }
    }
}

uint8_t CGACard::read(uint16_t addr)
{
    switch(addr)
    {
        case 0x3DA: // status
        {
            update();

            int hDisplayed = regs[1/* h disp*/] * 8;

            int charHeight = regs[9/*max char scan*/] + 1;
            int vDisplayed = regs[6/*v displayed*/] * charHeight;

            // the low "accessible" bit is the inverse of DE
            bool de = scanlineCycle < hDisplayed && scanline < vDisplayed;

            return status | (de ? 0 : 1);
        }
    }
    return 0xFF;
}

void CGACard::write(uint16_t addr, uint8_t data)
{
    switch(addr)
    {
        case 0x3D4: // reg select
        {
            regSelect = data;
            break;
        }
        case 0x3D5: // reg
        {
            update();
            regs[regSelect] = data;
            break;
        }

        case 0x3D8: // mode
        {
            update();
            mode = data;
            break;
        }
        case 0x3D9: // colour select
        {
            update();
            colSelect = data;
            break;
        }
    }
}
