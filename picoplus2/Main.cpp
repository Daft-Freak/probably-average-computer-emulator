#include <forward_list>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "tusb.h"

#include "dvhstx/dvhstx.hpp"

#include "BIOS.h"

#include "AboveBoard.h"
#include "CGACard.h"
#include "FixedDiskAdapter.h"
#include "FloppyController.h"
#include "Scancode.h"
#include "SerialMouse.h"
#include "System.h"

static System sys;

// static AboveBoard aboveBoard(sys);

static CGACard cga(sys);
static FloppyController fdc(sys);

#ifdef FIXED_DISK
static FixedDiskAdapter fixDisk(sys);
#endif

static SerialMouse mouse(sys);

static uint8_t ram[128 * 1024];

static pimoroni::DVHSTX dv;

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    // seems to be a bug sometimes when switching mode
    if(w > 640)
        w = 640;

    auto fb = dv.get_back_buffer();
    auto ptr = fb + line * 640;

    // convert
    for(int x = 0; x < w; x += 2)
    {
        int index = *data & 0xF;
        *ptr++ = index;
        if(w == 320) *ptr++ = index;

        index = *data++ >> 4;
        *ptr++ = index;
        if(w == 320) *ptr++ = index;
    }

    if(line == 0)
    {
        // sync
        dv.wait_for_flip();
        // set_display_size(w, 240); /*really 200*/
    }
    
    // write

    if(line == 199)
    {
        // end
        dv.flip_async();
    }
}

static void alarmCallback(uint alarmNum)
{
    // TODO: audio output
    while(sys.hasSpeakerSample())
        sys.getSpeakerSample();

    timer_hw->intr = 1 << alarmNum;
    hardware_alarm_set_target(alarmNum, make_timeout_time_ms(5));
}

void update_key_state(XTScancode code, bool state)
{
    sys.sendKey(code, state);
}

void update_mouse_state(int8_t x, int8_t y, bool left, bool right)
{
    mouse.addMotion(x, y);
    mouse.setButton(0, left);
    mouse.setButton(1, right);
    mouse.sync();
}

static void display_init()
{
    dv.init(640, 240, pimoroni::DVHSTX::MODE_PALETTE);

    static const uint8_t palette[16][4]
    {
        // B, G, R, X
        {0x00, 0x00, 0x00}, // black
        {0xAA, 0x00, 0x00}, // blue
        {0x00, 0xAA, 0x00}, // green
        {0xAA, 0xAA, 0x00}, // cyan
        {0x00, 0x00, 0xAA}, // red
        {0xAA, 0x00, 0xAA}, // magenta
        {0x00, 0x55, 0xAA}, // brown
        {0xAA, 0xAA, 0xAA}, // light grey

        {0x55, 0x55, 0x55}, // dark grey
        {0xFF, 0x55, 0x55}, // light blue
        {0x55, 0xFF, 0x55}, // light green
        {0xFF, 0xFF, 0x55}, // light cyan
        {0x55, 0x55, 0xFF}, // light red
        {0xFF, 0x55, 0xFF}, // light magenta
        {0x55, 0xFF, 0xFF}, // yellow
        {0xFF, 0xFF, 0xFF}, // white
    };

    for(int i = 0; i < 16; i++)
        dv.set_palette_colour(i, reinterpret_cast<const uint32_t *>(palette)[i]);
}

int main()
{
    tusb_init();

    stdio_init_all();

    // init storage/filesystem

    display_init();

    // emulator init
    sys.addMemory(0, sizeof(ram), ram);
    cga.setScanlineCallback(scanlineCallback);

    auto bios = _binary_bios_xt_rom_start;
    auto biosSize = _binary_bios_xt_rom_end - _binary_bios_xt_rom_start;
    auto biosBase = 0x100000 - biosSize;
    sys.addReadOnlyMemory(biosBase, biosSize, (const uint8_t *)bios);

#ifdef FIXED_DISK
    // size is wring, but mem mapping can't handle smaller
    sys.addReadOnlyMemory(0xC8000, 0x4000, (const uint8_t *)_binary_fixed_disk_bios_rom_start);

    // fixedIO.openDisk(0, "hd-win3.0.img");
    // fixDisk.setIOInterface(&fixedIO);
#endif

    sys.reset();

    auto time = get_absolute_time();

    // fake audio output
    // (since the core audio output is all over the place we can't just drain between updates)
    int alarmNum = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(alarmNum, alarmCallback);
    hardware_alarm_set_target(alarmNum, make_timeout_time_ms(5));
    irq_set_priority(TIMER0_IRQ_0 + alarmNum, PICO_LOWEST_IRQ_PRIORITY);
  
    while(true)
    {
        tuh_task();

        auto now = get_absolute_time();
        auto elapsed = absolute_time_diff_us(time, now) / 1000;

        if(elapsed)
        {
            if(elapsed > 10)
                elapsed = 10;

            sys.getCPU().run(elapsed);
            time = delayed_by_ms(time, elapsed);

            cga.update();
            sys.updateForDisplay();
        }
    }

    return 0;
}