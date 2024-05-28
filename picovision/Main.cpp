#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/time.h"

#include "Display.h"

#include "CGACard.h"
#include "System.h"

extern char _binary_bios_xt_rom_start[];
extern char _binary_bios_xt_rom_end[];

static System sys;

static CGACard cga(sys);

static uint8_t ram[160 * 1024];

static uint16_t scanLineOutBuf[640];

static constexpr uint16_t pack555(uint8_t r, uint8_t g, uint8_t b)
{
    return b | g << 5 | r << 10;
}

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    // RGB * 0xAA + I * 0x55
    // (except brown)
    static const uint16_t palette[]
    {
        pack555(0x00, 0x00, 0x00), // black
        pack555(0x00, 0x00, 0x15), // blue
        pack555(0x00, 0x15, 0x00), // green
        pack555(0x00, 0x15, 0x15), // cyan
        pack555(0x15, 0x00, 0x00), // red
        pack555(0x15, 0x00, 0x15), // magenta
        pack555(0x15, 0x0A, 0x00), // brown
        pack555(0x15, 0x15, 0x15), // light grey

        pack555(0x0A, 0x0A, 0x0A), // dark grey
        pack555(0x0A, 0x0A, 0x1F), // light blue
        pack555(0x0A, 0x1F, 0x0A), // light green
        pack555(0x0A, 0x1F, 0x1F), // light cyan
        pack555(0x1F, 0x0A, 0x0A), // light red
        pack555(0x1F, 0x0A, 0x1F), // light magenta
        pack555(0x1F, 0x1F, 0x0A), // yellow
        pack555(0x1F, 0x1F, 0x1F), // white
    };

    // TODO: sync screen w

    auto ptr = scanLineOutBuf;

    for(int x = 0; x < w; x += 2)
    {
        int index = *data & 0xF;

        *ptr++ = palette[index];
        index = *data++ >> 4;

        *ptr++ = palette[index];
    }

    if(line == 0)
    {
        // sync
        while(!display_render_needed());
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