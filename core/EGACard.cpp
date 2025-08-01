#include <cstdio>

#include "EGACard.h"

#include "CGAFont.h"

EGACard::EGACard(System &sys) : sys(sys)
{
    //sys.addMemory(0xB8000, sizeof(ram), ram);
    sys.addIODevice(0x3F0, 0x3B0, 0, this); // MDA compat
    sys.addIODevice(0x3F0, 0x3C0, 0, this); // EGA stuff
    sys.addIODevice(0x3F0, 0x3D0, 0, this); // CGA compat
}

void EGACard::remove()
{
    sys.removeIODevice(this);
}

bool EGACard::isInVBlank() const
{
    int vBlankStart = regs[21/*v sync*/] | ((regs[7 /*overflow*/] >> 3) & 1) << 8;
    return scanline >= vBlankStart;
}

void EGACard::setScanlineCallback(ScanlineCallback cb)
{
    scanCb = cb;
}

void EGACard::update()
{
    auto elapsed = sys.getCycleCount() - lastUpdateCycle;

    // check half clock flag
    // TODO: clock sel
    if(seqClockMode & (1 << 3))
    {
        elapsed /= 2;
        lastUpdateCycle += elapsed * 2;
    }
    else
        lastUpdateCycle += elapsed;

    int lineClocks = (regs[0/*h total*/] + 2) * 8;
    int hDisplayed = (regs[1/* h disp*/] + 1) * 8;

    int charHeight = regs[9/*max char scan*/] + 1;
    int totalLines = (regs[6/* v total*/])    | ( regs[7 /*overflow*/]       & 1) << 8;
    int vDisplayed = (regs[18/*v displayed*/] | ((regs[7 /*overflow*/] >> 1) & 1) << 8) + 1;
    int vBlankStart = regs[21/*v sync*/]      | ((regs[7 /*overflow*/] >> 3) & 1) << 8;

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
                curAddr += (regs[1/* h disp*/] + 1) * 2;

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

uint8_t EGACard::read(uint16_t addr)
{
    switch(addr)
    {
        case 0x3C2: // status 0
        {
            //TODO: bit 7 (1 if in display area)
            uint8_t switches = 0x7; // cga monitor, 80 col

            int clksel = (miscOutput >> 2) & 3;

            return ((switches >> (3 - clksel)) & 1) << 4;
        }

        case 0x3D5: // crtc reg
        {
            // write only
            if(regSelect < 12 || regSelect > 17) 
                return 0xFF;

            if(regSelect < 24)
                return regs[regSelect];

            return 0xFF;
        }

        case 0x3BA:
        case 0x3DA: // status 1
        {
            update();

            int hDisplayed = (regs[1/* h disp*/] + 1) * 8;

            int vDisplayed = (regs[18/*v displayed*/] | ((regs[7 /*overflow*/] >> 1) & 1) << 8) + 1;

            // the low "accessible" bit is the inverse of DE
            bool de = scanlineCycle < hDisplayed && scanline < vDisplayed;

            //printf("EGA R status 1 %02X (hd %i vd %i)\n", status | (de ? 0 : 1), hDisplayed, vDisplayed);

            int statusMux = (attribColourPlaneEnable >> 4) & 3;
            // FIXME: we're still IRGB internally
            // so this is returning I for all the secondary colours...
            int diag = 0;
            switch(statusMux)
            {
                case 0: // R/B
                    diag = (lastOutData & 1) | (lastOutData & 4) >> 1;
                    break;
                case 1: // sB/G
                    diag = (lastOutData & 2) >> 1 | ((lastOutData & 8) >> 2);
                    break;
                case 2: // sR/sG
                    diag = (lastOutData & 8) >> 3 | ((lastOutData & 8) >> 2);
                    break;
            }

            return 0xC0 | diag << 4 | status | (de ? 0 : 1);
        }

        default:
            printf("EGA R %03X\n", addr);
    }
    return 0xFF;
}

void EGACard::write(uint16_t addr, uint8_t data)
{
    switch(addr)
    {
        case 0x3C0: // attrib controller index/data
            attribCtrlAddr = data & 0x1F;
            break;
        // these are supposed to be the same reg
        // but the bios doesn't do that...
        case 0x3C1: // attrib controller regs
        {
            update();
            printf("EGA attrib %i = %02X\n", attribCtrlAddr, data);
            switch(attribCtrlAddr)
            {
                case 0x12: // colour plane enable
                    attribColourPlaneEnable = data;
                    break;

            }
            break;
        }
        case 0x3C2: // misc output
        {
            update();

            printf("EGA misc out = %02X\n", data);

            auto changed = miscOutput ^ data;
            miscOutput = data;

            if(changed & (1 << 1))
                setupMemory();
            break;
        }

        case 0x3C4: // sequencer index
            seqAddr = data & 0xF;
            break;
        case 0x3C5: // sequencer regs
        {
            update();
            printf("EGA seq %i = %02X\n", seqAddr, data);
            data &= 0xF; // high bits not connected
            switch(seqAddr)
            {
                case 1: // clock mode
                    seqClockMode = data;
                    break;
                case 2: // map mask
                    seqMapMask = data;
                    break;

                case 4: // memory mode
                    seqMemMode = data;
                    setupMemory();
                    break;
            }
            break;
        }

        // 3CA = gfx 2 pos
        
        // 3CC = gfx 1 pos

        case 0x3CE: // gfx ctrl index
            gfxAddr = data;
            break;
        case 0x3CF: // gfx ctrl regs
        {
            update();
            printf("EGA gfx %i = %02X\n", gfxAddr, data);
            switch(gfxAddr)
            {
                // 4 read sel
                case 4: // read sel
                    gfxReadSel = data;
                    break;
                
                case 6:
                    gfxMisc = data;
                    setupMemory();
                    break;
            }
            break;
        }


        case 0x3D4: // reg select
        {
            regSelect = data & 0x1F;
            break;
        }
        case 0x3D5: // reg
        {
            update();
            //printf("EGA CRTC W %i = %02X\n", regSelect, data);

            if(regSelect < 24)
                regs[regSelect] = data;
            break;
        }

        /*case 0x3D8: // mode
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
        }*/

        case 0x3BA: // feature control
        case 0x3DA:
            featureControl = data;
            break;

        default:
            printf("EGA W %03X = %02X\n", addr, data);
    }
}

void EGACard::draw(int start, int end)
{
    if(miscOutput & (1 << 4)/*disable output*/) // check enabled
    {
        for(int cycle = start; cycle < end; cycle++)
            scanlineBuf[cycle / 2] = 0; // black
    }
    else if(!(seqMemMode & 1)/*alpha mode*/)
    {
        // graphics mode
        if(mode & (1 << 4))
        {
            // hi-res
            auto addr = curAddr + ((scanline & 1) ? 0x2000 : 0);

            addr &= 0x3FFF;

            auto fg = colSelect & 0xF;

            int cycle = start;

            auto out = scanlineBuf + cycle;
            auto in = ram + addr + (cycle / 4);

            // round up
            if(cycle & 3)
            {
                auto data = *in++;
                data <<= (cycle & 3) * 2;
                for(; cycle & 3 && cycle < end; cycle++, data <<= 2)
                    *out++ = (data & 0x80 ? fg : 0) | (data & 0x40 ? fg << 4 : 0);

                if(cycle == end)
                    return;
            }

            // full bytes
            auto count = (end - cycle) / 4;
            while(count--)
            {
                auto data = *in++;

                *out++ = (data & 0x80 ? fg : 0) | (data & 0x40 ? fg << 4 : 0);
                *out++ = (data & 0x20 ? fg : 0) | (data & 0x10 ? fg << 4 : 0);
                *out++ = (data & 0x08 ? fg : 0) | (data & 0x04 ? fg << 4 : 0);
                *out++ = (data & 0x02 ? fg : 0) | (data & 0x01 ? fg << 4 : 0);
            }

            // remainder
            if(end & 3)
            {
                auto data = *in;
                for(int i = 0; i < (end & 7); i++, data <<= 2)
                    *out++ = (data & 0x80 ? fg : 0) | (data & 0x40 ? fg << 4 : 0);
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

                charAddr &= 0x3FFF;

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
        if(!inCursor && charLine > (regs[10/*cursor start*/] & 0x1F))
            inCursor = true;
        else if(inCursor && charLine == (regs[11/*cursor end*/] & 0x1F))
            inCursor = false;
        bool cursorLine = (frame & 8) && inCursor;
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
        auto inChars = ram + (curAddr & 0x3FFF) + (cycle / 8) * 2;
        auto inAttrib = ram + (curAddr & 0x3FFF) + (cycle / 8) * 2 + 0x4000;

        auto doSingle = [this, &out](bool cursor, uint8_t attr, uint8_t fontData, int cx)
        {
            int col;

            bool blinkEn = mode & (1 << 5);
            
            if(cursor) 
                col = attr & 0xF;
            // not cursor or cursor off
            else if(blinkEn)
            {
                // blink character
                if((attr & 0x80) && !(frame & 16))
                    col = (attr >> 4) & 7;
                else
                    col = (fontData & 1 << cx) ? attr & 0xF : (attr >> 4) & 7; // blink enabled so bg col is only three bits
            }
            else
                col = ((fontData & 1 << cx) ? attr : (attr >> 4)) & 0xF; // if blink is disabled we can use the high bit of the bg colour

            if(cx & 1)
                *out++ |= col << 4;
            else
                *out = col;
        };

        auto lineFont = cgaFont + charLine;
    
        // round to char size
        if(cycle & 7)
        {
            auto ch = *inChars;
            auto attr = *inAttrib;
            inChars += 2;
            inAttrib += 2;
            auto fontData = lineFont[ch * 8];

            // check if char in cursor
            bool cursor = inChars == cursorPtr;

            for(; cycle & 7 && cycle < end; cycle++)
                doSingle(cursor, attr, fontData, cycle & 7);

            if(cycle == end)
                return;
        }

        // full chars
        bool blinkEn = mode & (1 << 5);
        auto bgMask = blinkEn ? 7 : 0xF; // if blink is disabled, the blink bit is bg intensity

        auto charCount = (end - cycle) / 8;
        while(charCount--)
        {
            auto ch = *inChars;
            auto attr = *inAttrib;
            inChars += 2;
            inAttrib += 2;
            auto fontData = lineFont[ch * 8];

            // check if char in cursor (fg fill)
            if(inChars == cursorPtr)
            {
                out[0] = out[1] = out[2] = out[3] = (attr & 0xF) | attr << 4;
                out += 4;
            }
            // blink character (bg fill)
            // also check if char is blank and do the same
            else if(!fontData || (blinkEn && (attr & 0x80) && !(frame & 16)))
            {
                int bg = (attr >> 4) & bgMask;
                bg = bg | bg << 4;
                out[0] = out[1] = out[2] = out[3] = bg;
                out += 4;
            }
            else
            {
                for(int i = 0; i < 4; i++, fontData >>= 2)
                {
                    int col0 = (fontData & 1) ? attr & 0xF : (attr >> 4) & bgMask;
                    int col1 = (fontData & 2) ? attr & 0xF : (attr >> 4) & bgMask;

                    *out++ = col0 | col1 << 4;
                }
            }
        }

        lastOutData = out[-1] >> 4;

        // remainder
        if(end & 7)
        {
            auto ch = *inChars;
            auto attr = *inAttrib;
            inChars += 2;
            auto fontData = lineFont[ch * 8];

            // check if char in cursor
            bool cursor = inChars == cursorPtr;

            int i;
            for(i = 0; i < (end & 7); i++)
                doSingle(cursor, attr, fontData, i);

            if(i & 1)
                lastOutData = out[-1] >> 4;
            else
                lastOutData = out[0] & 0xF;
        }
    }
}

void EGACard::setupMemory()
{
    bool enabled = miscOutput & (1 << 1);
    bool chain = gfxMisc & (1 << 1);
    int map = (gfxMisc >> 2) & 3;
    bool oddEven = !(seqMemMode & (1 << 2));

    static const int mapAddrs[]
    {
        0xA0000,
        0xA0000,
        0xB0000,
        0xB8000
    };
    static const int mapSizes[]
    {
        128 * 1024,
        64 * 1024,
        32 * 1024,
        32 * 1024
    };

    if(!enabled)
    {
        printf("EGA RAM disabled\n");
        sys.setMemAccessCallbacks(0, 0, nullptr, nullptr);
    }
    else
    {
        printf("EGA RAM at %05X (%iK) chain %i odd/even %i\n", mapAddrs[map], mapSizes[map] / 1024, chain, oddEven);
        sys.setMemAccessCallbacks(mapAddrs[map], mapSizes[map], &EGACard::readMem, &EGACard::writeMem, this);
    }
}

uint8_t EGACard::readMem(uint32_t addr)
{
    bool chain = gfxMisc & (1 << 1);
    //bool oddEven = !(seqMemMode & (1 << 2)); // does odd/even affect this?

    int bank = 0; // TODO: > 64k
    int plane = gfxReadSel;
    int planeAddr = addr & 0x3FFF;

    auto mappedAddr = planeAddr;

    // remap low bit for chaining
    // TODO: only correct for 64k/no expansion ram
    if(chain)
        mappedAddr = (mappedAddr & ~1) | ((addr >> 14) & 1);
    else if((addr >> 14) & 1) // only 16k per plane
        return 0xFF;

    //printf("EGA R %05X (%04X, sel %i)\n", addr, mappedAddr, gfxReadSel);

    return ram[mappedAddr + plane * 0x4000 + bank * 0x10000];
}

void EGACard::writeMem(uint32_t addr, uint8_t data)
{
    bool chain = gfxMisc & (1 << 1);
    bool oddEven = !(seqMemMode & (1 << 2));

    int bank = 0; // TODO: > 64k
    int planeAddr = addr & 0x3FFF;

    auto mappedAddr = planeAddr;

    // remap low bit for chaining
    // TODO: only correct for 64k/no expansion ram
    if(chain)
        mappedAddr = (mappedAddr & ~1) | ((addr >> 14) & 1);
    else if((addr >> 14) & 1) // only 16k per plane
        return;

    //if(data)
    //    printf("EGA W %05X(%04X) = %02X\n", addr, mappedAddr, data);

    for(int i = 0; i < 4; i++)
    {
        if(!(seqMapMask & (1 << i)))
            continue;

        // odd/even selects bank based on (original) low bit
        // 0/1 and 2/3 should have the same masks in this case
        if(oddEven && (i & 1) != (planeAddr & 1))
            continue;

        // TODO: graphics controller stuff (set/reset, rotate, latches, and/or/xor, ...)
        ram[mappedAddr + i * 0x4000 + bank * 0x10000] = data;
    }
}