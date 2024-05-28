#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "Display.h"

#include "CGACard.h"
#include "System.h"

extern char _binary_bios_xt_rom_start[];
extern char _binary_bios_xt_rom_end[];

static System sys;

static CGACard cga(sys);

static uint8_t ram[160 * 1024];

static uint8_t scanLineOutBuf[640];

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    // TODO: sync screen w

    auto ptr = scanLineOutBuf;

    for(int x = 0; x < w; x += 2)
    {
        int index = *data & 0xF;
        *ptr++ = index << 2;

        index = *data++ >> 4;
        *ptr++ = index << 2;
    }

    if(line == 0)
    {
        // sync
        while(!display_render_needed());

        set_display_size(w, 240); /*really 200*/
    }

    write_display(0, line, w, scanLineOutBuf);

    if(line == 199)
        update_display();
}

static void alarmCallback(uint alarmNum) {
    // TODO: audio output
    while(sys.hasSpeakerSample())
        sys.getSpeakerSample();

    timer_hw->intr = 1 << alarmNum;
    hardware_alarm_set_target(alarmNum, make_timeout_time_ms(5));
}

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(250000, false);
    
    init_display();

    // emulator init
    sys.addMemory(0, sizeof(ram), ram);
    cga.setScanlineCallback(scanlineCallback);

    auto bios = _binary_bios_xt_rom_start;
    auto biosSize = _binary_bios_xt_rom_end - _binary_bios_xt_rom_start;
    auto biosBase = 0x100000 - biosSize;
    sys.addMemory(biosBase, biosSize, (uint8_t *)bios);

    sys.reset();

    auto time = get_absolute_time();

    // fake audio output
    // (since the core audio output is all over the place we can't just drain between updates)
    int alarmNum = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(alarmNum, alarmCallback);
    hardware_alarm_set_target(alarmNum, make_timeout_time_ms(5));
    irq_set_priority(TIMER_IRQ_0 + alarmNum, PICO_LOWEST_IRQ_PRIORITY);
  
    while(true)
    {
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