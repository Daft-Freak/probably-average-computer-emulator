#include <array>

#include "tusb.h"

#include "Scancode.h"

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

static uint8_t lastKeys[6]{0, 0, 0, 0, 0};
static uint8_t lastKeyMod = 0;

void update_key_state(XTScancode code, bool state);
void update_mouse_state(int8_t x, int8_t y, bool left, bool right);

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
                update_key_state(scancodeMap[key], true);
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
                update_key_state(scancodeMap[key], false);
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
                update_key_state(modMap[i], true);
            else if(releasedMods & (1 << i))
                update_key_state(modMap[i], false);
        }

        memcpy(lastKeys, keyboardReport->keycode, 6);
        lastKeyMod = keyboardReport->modifier;

        tuh_hid_receive_report(dev_addr, instance);
    }
    else if(protocol == HID_ITF_PROTOCOL_MOUSE)
    {
        auto mouseReport = (hid_mouse_report_t const*) report;

        update_mouse_state(mouseReport->x, mouseReport->y, mouseReport->buttons & MOUSE_BUTTON_LEFT, mouseReport->buttons & MOUSE_BUTTON_RIGHT);

        tuh_hid_receive_report(dev_addr, instance);
    }
}