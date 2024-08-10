#include <forward_list>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "tusb.h"

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

static uint8_t ram[256 * 1024];

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    // seems to be a bug sometimes when switching mode
    if(w > 640)
        w = 640;

    // convert

    if(line == 0)
    {
        // sync
        // set_display_size(w, 240); /*really 200*/

    }
    
    // write

    if(line == 199)
    {
        // end
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

int main()
{
    tusb_init();

    stdio_init_all();

    // init storage/filesystem
    // init display

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