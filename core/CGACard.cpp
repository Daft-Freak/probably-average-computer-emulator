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

    while(elapsed)
    {
        auto startCycle = scanlineCycle;
        auto step = std::max(UINT32_C(1), std::min(elapsed, static_cast<uint32_t>(lineClocks) - scanlineCycle));

        scanlineCycle += step;
        elapsed -= step;

        if(startCycle < hDisplayed && scanline < vDisplayed)
        {
            // in visible area, draw
            auto endCycle = scanlineCycle < hDisplayed ? scanlineCycle : hDisplayed;
            draw(startCycle, endCycle);
        }

        if(scanlineCycle >= lineClocks)
        {
            // display line
            if(scanline < vDisplayed && scanCb)
            {
                // hi-res gfx needs 2x width?
                int w = (mode & (1 << 4)) ? hDisplayed * 2 : hDisplayed;

                scanCb(scanlineBuf, scanline, w);
            }

            scanlineCycle = 0;
            scanline++;

            if((scanline % charHeight) == 0)
                curAddr += regs[1/* h disp*/] * 2;

            // check new scanline
            if(scanline >= totalLines)
            {
                scanline = 0;
                frame++;
                curAddr = (regs[12] << 8 | regs[13]) * 2;
                status &= ~(1 << 3); // clear vblank
            }
            else if(scanline >= vBlankStart)
                status |=  1 << 3; // vblank
        }
    }
}

uint8_t CGACard::read(uint16_t addr)
{
    switch(addr)
    {
        case 0x3D5: // reg
        {
            // write only
            if(regSelect < 12) // datasheet suggests this should be 14, but that breaks HWiNFO?
                return 0xFF;

            if(regSelect < 18)
                return regs[regSelect];

            return 0xFF;
        }

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
            regSelect = data & 0x1F;
            break;
        }
        case 0x3D5: // reg
        {
            update();
            if(regSelect == 12 || regSelect == 14)
                data &= 0x3F;

            if(regSelect < 18)
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

void CGACard::draw(int start, int end)
{
    if(!(mode & (1 << 3))) // check enabled
    {
        for(int cycle = start; cycle < end; cycle++)
            scanlineBuf[cycle / 2] = 0; // black
    }
    else if(mode & (1 << 1))
    {
        // graphics mode
        if(mode & (1 << 4))
        {
            // hi-res
            auto addr = curAddr + ((scanline & 1) ? 0x2000 : 0);

            auto fg = colSelect & 0xF;

            for(int cycle = start; cycle < end; cycle++)
            {
                auto charAddr = addr + (cycle / 4);

                auto data = ram[charAddr];
                auto col = (data << ((cycle & 3) * 2) >> 6) & 3;

                auto col0 = 0, col1 = 0;
                
                if(col & 2)
                    col0 = fg;
                
                if(col & 1)
                    col1 = fg;

                scanlineBuf[cycle] = col0 | col1 << 4;
            }
        }
        else
        {
            int palIndex = (colSelect >> 5) & 1;
            bool bright = (colSelect & (1 << 4));
            auto bg = colSelect & 0xF;

            auto addr = curAddr + ((scanline & 1) ? 0x2000 : 0);

            for(int cycle = start; cycle < end; cycle++)
            {
                auto charAddr = addr + (cycle / 4);

                auto data = ram[charAddr];
                auto col = (data << ((cycle & 3) * 2) >> 6) & 3;

                if(col == 0)
                    col = bg;
                else
                {
                    // palette mapping is just shifting up 1 bit, palette select is the low bit
                    // TODO: mixed palette if b/w bit set
                    col = (col << 1) | palIndex | (bright ? 8 : 0);
                }

                if(cycle & 1)
                    scanlineBuf[cycle / 2] |= col << 4;
                else
                    scanlineBuf[cycle / 2] = col;
            }
        }
    }
    else
    {
        // text mode
        // assuming 8x8 chars...

        int charLine = scanline & 7;

        // check if line in cursor
        // for more accuracy, should toggle when reaching those lines (resulting in wrap around sometimes)
        // also check for blinking (handled outside 6845, 8/8 frames)
        bool cursorLine = (frame & 8) && charLine >= (regs[10/*cursor start*/] & 0x1F) && charLine <= (regs[11/*cursor end*/] & 0x1F);
        uint8_t *cursorPtr = nullptr;

        if(cursorLine)
        {
            uint16_t cursorAddr = regs[14] << 8 | regs[15];

            // +2 because we check after incrementing
            // set to null if not cursor line
            cursorPtr = ram + cursorAddr * 2 + 2;
        }

        int cycle = start;

        auto out = scanlineBuf + cycle / 2;
        auto in = ram + curAddr + (cycle / 8) * 2;

        auto doSingle = [this, &out](bool cursor, uint8_t attr, uint8_t fontData, int cx)
        {
            int col;
            
            if(cursor) 
                col = attr & 0xF;
            // not cursor or cursor off
            // blink character
            else if((attr & 0x80) && !(frame & 16))
                col = (attr >> 4) & 7;
            else
                col = (fontData & 1 << cx) ? attr & 0xF : (attr >> 4) & 7;

            if(cx & 1)
                *out++ |= col << 4;
            else
                *out = col;
        };

        auto lineFont = cgaFont + charLine;
    
        // round to char size
        if(cycle & 7)
        {
            auto ch = *in++;
            auto attr = *in++;
            auto fontData = lineFont[ch * 8];

            // check if char in cursor
            bool cursor = in == cursorPtr;

            for(; cycle & 7 && cycle < end; cycle++)
                doSingle(cursor, attr, fontData, cycle & 7);

            if(cycle == end)
                return;
        }

        // full chars
        auto charCount = (end - cycle) / 8;
        while(charCount--)
        {
            auto ch = *in++;
            auto attr = *in++;

            auto fontData = lineFont[ch * 8];

            // check if char in cursor (fg fill)
            if(in == cursorPtr)
            {
                out[0] = out[1] = out[2] = out[3] = (attr & 0xF) | attr << 4;
                out += 4;
            }
            // blink character (bg fill)
            // also check if char is blank and do the same
            else if(!fontData || ((attr & 0x80) && !(frame & 16)))
            {
                out[0] = out[1] = out[2] = out[3] = ((attr >> 4) & 7) | (attr & 0x70);
                out += 4;
            }
            else
            {
                for(int i = 0; i < 4; i++, fontData >>= 2)
                {
                    int col0 = (fontData & 1) ? attr & 0xF : (attr >> 4) & 7;
                    int col1 = (fontData & 2) ? attr & 0xF : (attr >> 4) & 7;

                    *out++ = col0 | col1 << 4;
                }
            }
        }

        // remainder
        if(end & 7)
        {
            auto ch = *in++;
            auto attr = *in;
            auto fontData = lineFont[ch * 8];

            // check if char in cursor
            bool cursor = in == cursorPtr;

            for(int i = 0; i < (end & 7); i++)
                doSingle(cursor, attr, fontData, i);
        }
    }
}
