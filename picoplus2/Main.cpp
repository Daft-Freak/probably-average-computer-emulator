#include <forward_list>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "tusb.h"

#include "dvhstx/dvhstx.hpp"

#include "AboveBoard.h"
#include "CGACard.h"
#include "FixedDiskAdapter.h"
#include "FloppyController.h"
#include "Scancode.h"
#include "SerialMouse.h"
#include "System.h"

extern char _binary_bios_xt_rom_start[];
extern char _binary_bios_xt_rom_end[];

extern char _binary_fixed_disk_bios_rom_start[];
extern char _binary_fixed_disk_bios_rom_end[];

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

static uint8_t lastKeys[6]{0, 0, 0, 0, 0};
static uint8_t lastKeyMod = 0;

static const XTScancode scancodeMap[]{
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,

    XTScancode::A,
    XTScancode::B,
    XTScancode::C,
    XTScancode::D,
    XTScancode::E,
    XTScancode::F,
    XTScancode::G,
    XTScancode::H,
    XTScancode::I,
    XTScancode::J,
    XTScancode::K,
    XTScancode::L,
    XTScancode::M,
    XTScancode::N,
    XTScancode::O,
    XTScancode::P,
    XTScancode::Q,
    XTScancode::R,
    XTScancode::S,
    XTScancode::T,
    XTScancode::U,
    XTScancode::V,
    XTScancode::W,
    XTScancode::X,
    XTScancode::Y,
    XTScancode::Z,
    
    XTScancode::_1,
    XTScancode::_2,
    XTScancode::_3,
    XTScancode::_4,
    XTScancode::_5,
    XTScancode::_6,
    XTScancode::_7,
    XTScancode::_8,
    XTScancode::_9,
    XTScancode::_0,

    XTScancode::Return,
    XTScancode::Escape,
    XTScancode::Backspace,
    XTScancode::Tab,
    XTScancode::Space,

    XTScancode::Minus,
    XTScancode::Equals,
    XTScancode::LeftBracket,
    XTScancode::RightBracket,
    XTScancode::Backslash,
    XTScancode::Backslash, // same key
    XTScancode::Semicolon,
    XTScancode::Apostrophe,
    XTScancode::Grave,
    XTScancode::Comma,
    XTScancode::Period,
    XTScancode::Slash,

    XTScancode::CapsLock,

    XTScancode::F1,
    XTScancode::F2,
    XTScancode::F3,
    XTScancode::F4,
    XTScancode::F5,
    XTScancode::F6,
    XTScancode::F7,
    XTScancode::F8,
    XTScancode::F9,
    XTScancode::F10,
    XTScancode::F11,
    XTScancode::F12,

    XTScancode::Invalid, // PrintScreen
    XTScancode::ScrollLock,
    XTScancode::Invalid, // Pause
    XTScancode::Insert,
    
    XTScancode::Home,
    XTScancode::PageUp,
    XTScancode::Delete,
    XTScancode::End,
    XTScancode::PageDown,
    XTScancode::Right,
    XTScancode::Left,
    XTScancode::Down,
    XTScancode::Up,

    XTScancode::NumLock,

    XTScancode::KPDivide,
    XTScancode::KPMultiply,
    XTScancode::KPMinus,
    XTScancode::KPPlus,
    XTScancode::KPEnter,
    XTScancode::KP1,
    XTScancode::KP2,
    XTScancode::KP3,
    XTScancode::KP4,
    XTScancode::KP5,
    XTScancode::KP6,
    XTScancode::KP7,
    XTScancode::KP8,
    XTScancode::KP9,
    XTScancode::KP0,
    XTScancode::KPPeriod,

    XTScancode::NonUSBackslash,

    XTScancode::Invalid, // Application
    XTScancode::Invalid, // Power

    XTScancode::KPEquals,

    // F13-F24
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,

    // no mapping
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,

    XTScancode::KPComma,
    XTScancode::Invalid,

    XTScancode::International1,
    XTScancode::International2,
    XTScancode::International3,
    XTScancode::International4,
    XTScancode::International5,
    XTScancode::International6,
    XTScancode::Invalid, // ...7
    XTScancode::Invalid, // ...8
    XTScancode::Invalid, // ...9
    XTScancode::Lang1,
    XTScancode::Lang2,
    XTScancode::Lang3,
    XTScancode::Lang4,
    XTScancode::Lang5,
};

static const XTScancode modMap[]
{
    XTScancode::LeftCtrl,
    XTScancode::LeftShift,
    XTScancode::LeftAlt,
    XTScancode::Invalid, // LeftGUI
    XTScancode::RightCtrl,
    XTScancode::RightShift,
    XTScancode::RightAlt,
    XTScancode::Invalid, // RightGUI
};

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

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
    // request report if it's a keyboard/mouse
    auto protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if(protocol == HID_ITF_PROTOCOL_KEYBOARD || protocol == HID_ITF_PROTOCOL_MOUSE)
        tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
    auto protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if(protocol == HID_ITF_PROTOCOL_KEYBOARD)
    {
        auto keyboardReport = (hid_keyboard_report_t const*) report;

        // check for new keys down
        for(int i = 0; i < 6 && keyboardReport->keycode[i]; i++)
        {
            auto key = keyboardReport->keycode[i];
            bool found = false;
            for(int j = 0; j < 6 && lastKeys[j] && !found; j++)
                found = lastKeys[j] == key;

            if(found)
                continue;

            if(key < std::size(scancodeMap) && scancodeMap[key] != XTScancode::Invalid)
                sys.sendKey(scancodeMap[key], true);
            else
                printf("key down %i %i\n", i, key);
        }

        // do the reverse and check for released keys
        for(int i = 0; i < 6 && lastKeys[i]; i++)
        {
            auto key = lastKeys[i];
            bool found = false;
            for(int j = 0; j < 6 && keyboardReport->keycode[j] && !found; j++)
                found = keyboardReport->keycode[j] == key;

            if(found)
                continue;

            if(key < std::size(scancodeMap) && scancodeMap[key] != XTScancode::Invalid)
                sys.sendKey(scancodeMap[key], false);
            else
                printf("key up %i %i\n", i, key);
        }

        // ...and mods
        auto changedMods = lastKeyMod ^ keyboardReport->modifier;
        auto pressedMods = changedMods & keyboardReport->modifier;
        auto releasedMods = changedMods ^ pressedMods;
        
        for(int i = 0; i < 8; i++)
        {
            if(modMap[i] == XTScancode::Invalid)
                continue;

            if(pressedMods & (1 << i))
                sys.sendKey(modMap[i], true);
            else if(releasedMods & (1 << i))
                sys.sendKey(modMap[i], false);
        }

        memcpy(lastKeys, keyboardReport->keycode, 6);
        lastKeyMod = keyboardReport->modifier;

        tuh_hid_receive_report(dev_addr, instance);
    }
    else if(protocol == HID_ITF_PROTOCOL_MOUSE)
    {
        auto mouseReport = (hid_mouse_report_t const*) report;

        mouse.addMotion(mouseReport->x, mouseReport->y);
        mouse.setButton(0, mouseReport->buttons & MOUSE_BUTTON_LEFT);
        mouse.setButton(1, mouseReport->buttons & MOUSE_BUTTON_RIGHT);
        mouse.sync();

        tuh_hid_receive_report(dev_addr, instance);
    }
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