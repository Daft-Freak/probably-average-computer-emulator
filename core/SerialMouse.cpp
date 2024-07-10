#include <cassert>
#include <cstdio>

#include "SerialMouse.h"

// this is also a minimal serial card
// (as those aren't implemented yet)

SerialMouse::SerialMouse(System &sys) : sys(sys)
{
    sys.addIODevice(0x3F8, 0x2F8, 1 << 3, this);

    lineStatus = (1 << 5)/*tx empty*/ | (1 << 6)/*tx shift empty*/;
}

void SerialMouse::addMotion(int x, int y)
{
    xMotion += x;
    yMotion += y;
}

void SerialMouse::setButton(int button, bool state)
{
    auto newButton = (state ? 1 : 0) << button;

    if(buttons ^ newButton)
    {
        changedButtons |= newButton;
        buttons ^= (1 << button);
    }
}

void SerialMouse::sync()
{
    if(!changedButtons && !xMotion && ! yMotion)
        return;

    // make sure queue isn't full
    if(rxQueue.getCount() > 5)
        return;

    // TODO: clamp motion?

    rxQueue.push(0x40 | (buttons & 1) << 5 | (buttons & 2) << 3 | (yMotion & 0xC0) >> 4 | (xMotion & 0xC0) >> 6);
    rxQueue.push(xMotion & 0x3F);
    rxQueue.push(yMotion & 0x3F);

    changedButtons = 0;
    xMotion = 0;
    yMotion = 0;
}

void SerialMouse::update()
{
    auto elapsed = sys.getCPU().getCycleCount() - lastUpdateCycle;

    if(!cpuCyclesPerWord)
    {
        // skip
        lastUpdateCycle += elapsed;
        return;
    }

    while(elapsed)
    {
        auto step = std::min(elapsed, wordCycleCounter);

        wordCycleCounter -= step;

        if(wordCycleCounter == 0)
        {
            // received word
            if(!rxQueue.empty())
            {
                // TODO: if already set, set overrun
                lineStatus |= 1; // receive ready
                if(interruptEnable & 1) // data available
                    sys.flagPICInterrupt(3);
            }
        
            wordCycleCounter = cpuCyclesPerWord;
        }

        lastUpdateCycle += step;
        elapsed -= step;
    }
}

uint8_t SerialMouse::read(uint16_t addr)
{
    update();

    switch(addr)
    {
        case 0x2F8: // RX buffer or divisor LSB
        {
            if(lineControl & (1 << 7))
                return divisor;
            else
            {
                if(lineStatus & 1)
                {
                    lineStatus &= ~1; // clear receive ready
                    return rxQueue.pop();
                }

                break;
            }
        }
        case 0x2F9: // interrupt enable or divisor MSB
        {
            if(lineControl & (1 << 7))
                return divisor >> 8;
            else
                return interruptEnable;
        }

        case 0x2FB: // line control
            return lineControl;

        case 0x2FC: // modem control
            return modemControl;

        case 0x2FD: // line status:
            return lineStatus;

        case 0x2FE: // modem status
            return 0;

        default:
            printf("serial/mouse R %04X @~%04X\n", addr, sys.getCPU().reg(CPU::Reg16::IP));
    }
    return 0xFF;
}

void SerialMouse::write(uint16_t addr, uint8_t data)
{
    update();

    switch(addr)
    {
        case 0x2F8: // TX buffer or divisor LSB
        {
            if(lineControl & (1 << 7))
            {
                divisor = (divisor & 0xFF00) | data;
                updateTimings();
            }
            else
                printf("serial/mouse tx %X\n", data);
            break;
        }
        case 0x2F9: // interrupt enable or divisor MSB
        {
            if(lineControl & (1 << 7))
            {
                divisor = (divisor & 0xFF) | data << 8;
                updateTimings();
            }
            else
            {
                interruptEnable = data;
                if(data)
                    printf("serial/mouse interrupt enable %X\n", data);
            }
            break;
        }

        case 0x2FB: // line control
            lineControl = data;
            printf("serial/mouse line control %X\n", data);
            updateTimings();
            break;
        case 0x2FC: // modem control
        {
            auto changed = modemControl ^ data;
            modemControl = data;
            printf("serial/mouse modem control %X\n", data);

            if(changed & 1) // DTR
                rxQueue.push('M');

            if((changed & 2) && (data & 3) == 3)
            {
                // RTS went high while DTR on, end of reset (probably)

                while(!rxQueue.empty())
                    rxQueue.pop();

                rxQueue.push('M');
            }
            break;
        }

        default:
            printf("serial/mouse W %04X = %02X @~%04X\n", addr, data, sys.getCPU().reg(CPU::Reg16::IP));
    }
}

void SerialMouse::updateForInterrupts()
{
    if(!interruptEnable)
        return;

    auto elapsed = sys.getCPU().getCycleCount() - lastUpdateCycle;

    if(elapsed >= wordCycleCounter)
        update();
}

int SerialMouse::getCyclesToNextInterrupt(uint32_t cycleCount)
{
    if(!cpuCyclesPerWord)
        return 0x7FFFFFFF;

    auto passed = cycleCount - lastUpdateCycle;

    return wordCycleCounter - passed;
}

void SerialMouse::updateTimings()
{
    if(divisor == 0)
        return;

    int bits = 5 + (lineControl & 3); // data bits
    bits += 1 + ((lineControl >> 2) & 1); // stop bits
    bits += (lineControl >> 3) & 1; // parity bit
    bits++; // start bit

    auto baud = 1843200 / divisor / 16;

    // get rough number of cpu cycles per word
    // (this emulator does not yet support multiple clocks...)
    cpuCyclesPerWord = 4772726 * bits / baud;

    sys.calculateNextInterruptCycle(sys.getCPU().getCycleCount());
}
