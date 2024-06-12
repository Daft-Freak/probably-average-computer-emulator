#include <cassert>
#include <cstdio>

#include "SerialMouse.h"

// this is also a minimal serial card
// (as those aren't implemented yet)

SerialMouse::SerialMouse(System &sys) : sys(sys)
{
    sys.addIODevice(0x2F8, 0x2FF, this);
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

    // TODO: make sure queue isn't full
    // TODO: clamp motion?

    rxQueue.push(0x40 | (buttons & 1) << 5 | (buttons & 2) << 3 | (yMotion & 0xC0) >> 4 | (xMotion & 0xC0) >> 6);
    rxQueue.push(xMotion & 0x3F);
    rxQueue.push(yMotion & 0x3F);

    changedButtons = 0;
    xMotion = 0;
    yMotion = 0;

    if(interruptEnable & 1) // data available
        sys.flagPICInterrupt(3);
}

uint8_t SerialMouse::read(uint16_t addr)
{
    switch(addr)
    {
        case 0x2F8: // RX buffer or divisor LSB
        {
            if(lineControl & (1 << 7))
                return divisor;
            else
            {
                auto v = rxQueue.pop();

                // re-flag interrupt if we have more data
                if(!rxQueue.empty() && (interruptEnable & 1))
                    sys.flagPICInterrupt(3);

                return v;
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
            return (rxQueue.empty() ? 0 : 1) | (1 << 5)/*tx empty*/ | (1 << 6)/*tx shift empty*/;

        case 0x2FE: // modem status
            return 0;

        default:
            printf("serial/mouse R %04X @~%04X\n", addr, sys.getCPU().reg(CPU::Reg16::IP));
    }
    return 0xFF;
}

void SerialMouse::write(uint16_t addr, uint8_t data)
{
    switch(addr)
    {
        case 0x2F8: // TX buffer or divisor LSB
        {
            if(lineControl & (1 << 7))
                divisor = (divisor & 0xFF00) | data;
            else
                printf("serial/mouse tx %X\n", data);
            break;
        }
        case 0x2F9: // interrupt enable or divisor MSB
        {
            if(lineControl & (1 << 7))
                divisor = (divisor & 0xFF) | data << 8;
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
