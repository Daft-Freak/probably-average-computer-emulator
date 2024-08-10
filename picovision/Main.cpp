#include <forward_list>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "tusb.h"

#include "fatfs/ff.h"

#include "aps6404.hpp"

#include "BIOS.h"

#include "Display.h"

#include "AboveBoard.h"
#include "CGACard.h"
#include "FixedDiskAdapter.h"
#include "FloppyController.h"
#include "Scancode.h"
#include "SerialMouse.h"
#include "System.h"

struct MemBlockMapping
{
    uint8_t cacheBlock;
    uint8_t windowBlock; // where expanded memory is mapped to
    uint16_t memBlock;
};

static FATFS fs;

static System sys;

static AboveBoard aboveBoard(sys);

static CGACard cga(sys);
static FloppyController fdc(sys);

#ifdef FIXED_DISK
static FixedDiskAdapter fixDisk(sys);
#endif

static SerialMouse mouse(sys);

static const int cacheBlocks = 12;
static const int frameFlushThreshold = 4; // minimum dirty blocks before flushing at frame end
static const int maxFlushBlocks = 2; // maximum blocks to flush at the end of a frame
static const int totalPSRAMBlocks = 7 * 1024 * 1024 / System::getMemoryBlockSize(); // (1MB CPU address space + 6MB expanded)

static uint8_t ramCache[cacheBlocks * System::getMemoryBlockSize()];
static std::forward_list<MemBlockMapping> ramCacheMap;
static std::forward_list<MemBlockMapping>::iterator ramCacheEnd;
static std::forward_list<MemBlockMapping>::iterator lastFlushedBlock;
static uint32_t cacheUsedMem[totalPSRAMBlocks / 32];

static constexpr uint32_t psramBaseAddress = 0x100000;

static uint8_t scanLineOutBuf[640];
static int firstFrames = 2;
static bool discardFrame = false;

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
                if(!sys.getMemoryBlockDirty(it->windowBlock))
                    continue;

                const auto blockSize = System::getMemoryBlockSize();
                display_get_ram().write(psramBaseAddress + it->memBlock * blockSize, (uint32_t *)(ramCache + it->cacheBlock * blockSize), blockSize);

                sys.clearMemoryBlockDirty(it->windowBlock);

                count--;
            }
        }
    }
}

static uint8_t *requestMem(unsigned int block)
{
    const auto blockSize = System::getMemoryBlockSize();

    block = aboveBoard.remapMemoryBlockFromWindow(block);

    // don't map over 640k
    if(block * blockSize > 0xA0000 && block * blockSize < 0x100000)
        return nullptr;

    // limit to PSRAM (rounded down a bit)
    if(block * blockSize >= 7 * 1024 * 1024)
        return nullptr;

    auto &psram = display_get_ram();

    // find non-dirty block
    auto it = ramCacheMap.begin();
    auto prevIt = ramCacheMap.before_begin();
    for(; it != ramCacheMap.end(); prevIt = it++)
    {
        // unused block
        if(it->memBlock == 0xFFFF)
            break;

        if(!sys.getMemoryBlockDirty(it->windowBlock))
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
            sys.clearMemoryBlockDirty(it->windowBlock);
        }

        // drop the rest of this frame, one broken frame is better than two...
        if(!cga.isInVBlank())
            discardFrame = true;
    }

    // got a cache block, fill it
    bool used = (cacheUsedMem[block / 32] & (1 << (block % 32)));
    if(used)
    {
        // load if previously used
        psram.wait_for_finish_blocking();
        psram.read_blocking(psramBaseAddress + block * blockSize, (uint32_t *)(ramCache + it->cacheBlock * blockSize), blockSize / 4);
    }
    else // otherwise just clear it
        memset(ramCache + it->cacheBlock * blockSize, 0, blockSize);

    // remove old mapping
    if(it->memBlock != 0xFFFF)
        sys.removeMemory(it->windowBlock);

    it->memBlock = block;
    it->windowBlock = aboveBoard.remapMemoryBlockToWindow(block);

    // the first time we use a block, mark it as dirty
    if(!used)
    {
        cacheUsedMem[block / 32] |= (1 << (block % 32));
        sys.setMemoryBlockDirty(it->windowBlock);
    }

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
        if(firstDirty != lastFlushedBlock && sys.getMemoryBlockDirty(firstDirty->windowBlock))
            break;
    }

    // if there were no more dirty blocks, we'll search from the start while counting

    for(auto it = ramCacheMap.begin(); it != ramCacheMap.end(); ++it)
    {
        if(sys.getMemoryBlockDirty(it->windowBlock))
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
        if(!sys.getMemoryBlockDirty(it->windowBlock))
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
    ramCacheMap.emplace_front(MemBlockMapping{0, 0xFF, 0xFFFF});
    ramCacheEnd = ramCacheMap.begin();

    for(uint8_t i = 1; i < cacheBlocks; i++)
        ramCacheEnd = ramCacheMap.emplace_after(ramCacheEnd, MemBlockMapping{i, 0xFF, 0xFFFF});

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