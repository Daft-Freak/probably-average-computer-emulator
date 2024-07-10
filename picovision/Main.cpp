#include <forward_list>

#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "tusb.h"

#include "fatfs/ff.h"

#include "aps6404.hpp"

#include "Display.h"

#include "CGACard.h"
#include "FixedDiskAdapter.h"
#include "FloppyController.h"
#include "Scancode.h"
#include "SerialMouse.h"
#include "System.h"

struct MemBlockMapping
{
    uint8_t cacheBlock;
    uint8_t memBlock;
};

extern char _binary_bios_xt_rom_start[];
extern char _binary_bios_xt_rom_end[];

extern char _binary_fixed_disk_bios_rom_start[];
extern char _binary_fixed_disk_bios_rom_end[];

static FATFS fs;

static System sys;

static CGACard cga(sys);
static FloppyController fdc(sys);

#ifdef FIXED_DISK
static FixedDiskAdapter fixDisk(sys);
#endif

static SerialMouse mouse(sys);

static const int cacheBlocks = 12;
static const int frameFlushThreshold = 4; // minimum dirty blocks before flushing at frame end
static const int maxFlushBlocks = 2; // maximum blocks to flush at the end of a frame

static uint8_t ramCache[cacheBlocks * System::getMemoryBlockSize()];
static std::forward_list<MemBlockMapping> ramCacheMap;
static std::forward_list<MemBlockMapping>::iterator ramCacheEnd;
static std::forward_list<MemBlockMapping>::iterator lastFlushedBlock;

static constexpr uint32_t psramBaseAddress = 0x100000;

static uint8_t scanLineOutBuf[640];
static int firstFrames = 2;
static bool discardFrame = false;

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
    XTScancode::Invalid, // Insert
    
    XTScancode::Invalid, // Home
    XTScancode::Invalid, // PageUp
    XTScancode::Invalid, // Delete
    XTScancode::Invalid, // End
    XTScancode::Invalid, // PageDown
    XTScancode::Invalid, // Right
    XTScancode::Invalid, // Left
    XTScancode::Invalid, // Down
    XTScancode::Invalid, // Up

    XTScancode::NumLock,

    XTScancode::Invalid, // KPDivide
    XTScancode::KPMultiply,
    XTScancode::KPMinus,
    XTScancode::KPPlus,
    XTScancode::Invalid, // KPEnter
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
    XTScancode::Invalid, // RightCtrl
    XTScancode::RightShift,
    XTScancode::Invalid, // RightAlt
    XTScancode::Invalid, // RightGUI
};

class FileFixedIO final : public FixedDiskIO
{
public:
    bool isPresent(int unit) override;
    bool read(int unit, uint8_t *buf, uint32_t lba) override;
    bool write(int unit, const uint8_t *buf, uint32_t lba) override;

    void openDisk(int unit, const char *path);

    static const int maxDrives = 1;

private:
    FIL file[maxDrives];
};

bool FileFixedIO::isPresent(int unit)
{
    return unit < maxDrives;// && file[unit];
}

bool FileFixedIO::read(int unit, uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    f_lseek(&file[unit], lba * 512);

    UINT read = 0;
    auto res = f_read(&file[unit], buf, 512, &read);

    return res == FR_OK && read == 512;
}

bool FileFixedIO::write(int unit, const uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    
    f_lseek(&file[unit], lba * 512);

    UINT written = 0;
    auto res = f_write(&file[unit], buf, 512, &written);

    return res == FR_OK && written == 512;
}

void FileFixedIO::openDisk(int unit, const char *path)
{
    if(unit >= maxDrives)
        return;

    auto res = f_open(&file[unit], path, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);

    if(res == FR_OK)
        printf("Loaded fixed-disk %i: %s\n", unit, path);
}

static FileFixedIO fixedIO;

static std::forward_list<MemBlockMapping>::iterator cacheFlush();

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    auto ptr = scanLineOutBuf;

    // seems to be a bug sometimes when switching mode
    if(w > 640)
        w = 640;

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
        while(!discardFrame && !display_render_needed());
        discardFrame = false;

        set_display_size(w, 240); /*really 200*/

        if(firstFrames)
        {
            // fill the bottom part of the screen of the first two frames
            for(int y = 200; y < 240; y++)
                write_display(0, y, 640, scanLineOutBuf);
            firstFrames--;
        }
    }
    
    if(!discardFrame)
        write_display(0, line, w, scanLineOutBuf);

    if(line == 199)
    {
        // attempt to flush some dirty blocks early
        auto firstDirty = cacheFlush();
        int psramBank = display_get_ram_bank();

        update_display();

        // if we're flushing wait for the next frame now
        // which won't cause a glitch since we're at the end of the frame anyway
        if(firstDirty != ramCacheMap.end())
        {
            display_wait_for_frame(psramBank);

            // finish writing blocks
            int count = maxFlushBlocks;

            for(auto it = firstDirty; it != ramCacheMap.end() && count; ++it)
            {
                if(!sys.getMemoryBlockDirty(it->memBlock))
                    continue;

                const auto blockSize = System::getMemoryBlockSize();
                display_get_ram().write(psramBaseAddress + it->memBlock * blockSize, (uint32_t *)(ramCache + it->cacheBlock * blockSize), blockSize);

                sys.clearMemoryBlockDirty(it->memBlock);

                count--;
            }
        }
    }
}

static uint8_t *requestMem(unsigned int block)
{
    const auto blockSize = System::getMemoryBlockSize();

    // don't map over 640k
    if(block * blockSize > 0xA0000)
        return nullptr;

    auto &psram = display_get_ram();

    // find non-dirty block
    auto it = ramCacheMap.begin();
    auto prevIt = ramCacheMap.before_begin();
    for(; it != ramCacheMap.end(); prevIt = it++)
    {
        // unused block
        if(it->memBlock == 0xFF)
            break;

        if(!sys.getMemoryBlockDirty(it->memBlock))
            break;
    }

    if(it == ramCacheMap.end())
    {
        // all cache dirty, force flush first
        it = ramCacheMap.begin();
        prevIt = ramCacheMap.before_begin();

        // we really don't want the bank switching here, so make sure the next frame has started
        while(!display_render_needed());

        int psramBank = display_get_ram_bank();

        // as we're going to block for an entire frame anyway, might as well use the time to flush all the blocks
        for(auto it2 = it; it2 != ramCacheMap.end(); ++it2)
            psram.write(psramBaseAddress + it2->memBlock * blockSize, (uint32_t *)(ramCache + it2->cacheBlock * blockSize), blockSize);

        psram.wait_for_finish_blocking();
        // swap buf, write again
        // this will cause a display glitch
        display_wait_for_frame(psramBank);

        for(auto it2 = it; it2 != ramCacheMap.end(); ++it2)
        {
            psram.write(psramBaseAddress + it2->memBlock * blockSize, (uint32_t *)(ramCache + it2->cacheBlock * blockSize), blockSize);
            sys.clearMemoryBlockDirty(it2->memBlock);
        }

        // drop the rest of this frame, one broken frame is better than two...
        discardFrame = true;
    }

    // got a cache block, fill it
    psram.wait_for_finish_blocking();
    psram.read_blocking(psramBaseAddress + block * blockSize, (uint32_t *)(ramCache + it->cacheBlock * blockSize), blockSize / 4);

    // remove old mapping
    if(it->memBlock != 0xFF)
        sys.removeMemory(it->memBlock);

    it->memBlock = block;

    if(it != ramCacheEnd)
    {
        // move to end of list
        ramCacheMap.splice_after(ramCacheEnd, ramCacheMap, prevIt);
        ramCacheEnd = it;
    }

    return ramCache + it->cacheBlock * blockSize;
}

static std::forward_list<MemBlockMapping>::iterator cacheFlush()
{
    // this should be true at this point
    // but if it isn't we could end up switching bank part way through
    if(!display_render_needed())
        return ramCacheMap.end();

    const auto blockSize = System::getMemoryBlockSize();

    // find dirty
    int dirtyBlocks = 0;
    auto firstDirty = lastFlushedBlock;

    // try to avoid blocks flushed last time
    for(; firstDirty != ramCacheMap.end(); ++firstDirty)
    {
        if(firstDirty != lastFlushedBlock && sys.getMemoryBlockDirty(firstDirty->memBlock))
            break;
    }

    // if there were no more dirty blocks, we'll search from the start while counting

    for(auto it = ramCacheMap.begin(); it != ramCacheMap.end(); ++it)
    {
        if(sys.getMemoryBlockDirty(it->memBlock))
        {
            dirtyBlocks++;
            // track first dirty block
            if(firstDirty == ramCacheMap.end())
                firstDirty = it;
        }
    }

    if(dirtyBlocks < frameFlushThreshold)
        return ramCacheMap.end();

    // now flush to the current PSRAM bank
    int count = maxFlushBlocks;

    for(auto it = firstDirty; it != ramCacheMap.end() && count; ++it)
    {
        if(!sys.getMemoryBlockDirty(it->memBlock))
            continue;

        display_get_ram().write(psramBaseAddress + it->memBlock * blockSize, (uint32_t *)(ramCache + it->cacheBlock * blockSize), blockSize);

        count--;
        lastFlushedBlock = it;
    }

    return firstDirty;
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
                sys.sendKey(static_cast<uint8_t>(scancodeMap[key]));
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
                sys.sendKey(0x80 | static_cast<uint8_t>(scancodeMap[key])); // break code
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
                sys.sendKey(static_cast<uint8_t>(modMap[i]));
            else if(releasedMods & (1 << i))
                sys.sendKey(0x80 | static_cast<uint8_t>(modMap[i])); // break code
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

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(250000, false);

    tusb_init();

    stdio_init_all();

    // init storage/filesystem
    auto res = f_mount(&fs, "", 1);

    if(res != FR_OK)
    {
        printf("Failed to mount filesystem! (%i)\n", res);
        while(true);
    }

    init_display();

    // int ram map
    ramCacheMap.emplace_front(MemBlockMapping{0, 0xFF});
    ramCacheEnd = ramCacheMap.begin();

    for(uint8_t i = 1; i < cacheBlocks; i++)
        ramCacheEnd = ramCacheMap.emplace_after(ramCacheEnd, MemBlockMapping{i, 0xFF});

    lastFlushedBlock = ramCacheMap.end();

    // emulator init
    sys.setMemoryRequestCallback(requestMem);
    cga.setScanlineCallback(scanlineCallback);

    auto bios = _binary_bios_xt_rom_start;
    auto biosSize = _binary_bios_xt_rom_end - _binary_bios_xt_rom_start;
    auto biosBase = 0x100000 - biosSize;
    sys.addReadOnlyMemory(biosBase, biosSize, (const uint8_t *)bios);

#ifdef FIXED_DISK
    // size is wring, but mem mapping can't handle smaller
    sys.addReadOnlyMemory(0xC8000, 0x4000, (const uint8_t *)_binary_fixed_disk_bios_rom_start);

    fixedIO.openDisk(0, "hd0.img");
    fixDisk.setIOInterface(&fixedIO);
#endif

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