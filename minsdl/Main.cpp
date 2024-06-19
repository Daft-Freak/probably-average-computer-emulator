#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <SDL.h>

#include "CGACard.h"
#include "FixedDiskAdapter.h"
#include "FloppyController.h"
#include "Scancode.h"
#include "SerialMouse.h"
#include "System.h"

class FileFloppyIO final : public FloppyDiskIO
{
public:
    bool isPresent(int unit) override;
    bool read(int unit, uint8_t *buf, uint8_t cylinder, uint8_t head, uint8_t sector) override;

    void openDisk(int unit, std::string path);

    static const int maxDrives = 4;

private:
    std::ifstream file[maxDrives];

    bool doubleSided[maxDrives];
    int sectorsPerTrack[maxDrives];
};

class FileFixedIO final : public FixedDiskIO
{
public:
    bool isPresent(int unit) override;
    bool read(int unit, uint8_t *buf, uint32_t lba) override;
    bool write(int unit, const uint8_t *buf, uint32_t lba) override;

    void openDisk(int unit, std::string path);

    static const int maxDrives = 2;

private:
    std::fstream file[maxDrives];

    bool doubleSided[maxDrives];
    int sectorsPerTrack[maxDrives];
};

static bool quit = false;
static bool turbo = false;

static System sys;

static CGACard cga(sys);
static FixedDiskAdapter fixDisk(sys);
static FloppyController fdc(sys);
static SerialMouse mouse(sys);

static uint8_t ram[640 * 1024];

static uint8_t screenData[640 * 200 * 4];
static int curScreenW = 0;

static uint8_t biosROM[0x10000];
static uint8_t fixedDiskBIOSROM[16 * 1024];

static FileFloppyIO floppyIO;
static FileFixedIO fixedIO;

static std::list<std::string> nextFloppyImage;

static XTScancode scancodeMap[SDL_NUM_SCANCODES]
{
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
    XTScancode::Invalid, // ...6
    XTScancode::Invalid, // ...7
    XTScancode::Invalid, // ...8
    XTScancode::Invalid, // ...9

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
    XTScancode::Invalid,
    XTScancode::Invalid,
    XTScancode::Invalid,

    XTScancode::LeftCtrl,
    XTScancode::LeftShift,
    XTScancode::LeftAlt,
    XTScancode::Invalid, // LeftGUI
    XTScancode::Invalid, // RightCtrl
    XTScancode::RightShift,
};

static void audioCallback(void *userdata, Uint8 *stream, int len)
{
    auto ptr = reinterpret_cast<int16_t *>(stream);
    for(int i = 0; i < len / 2; i++)
    {
        while(!quit && !sys.hasSpeakerSample())
            std::this_thread::yield();

        *ptr++ = sys.getSpeakerSample() << 4;
    }
}

static void pollEvents()
{
    const int escMod = KMOD_RCTRL | KMOD_RSHIFT;

    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_KEYDOWN:
            {
                if((event.key.keysym.mod & escMod) != escMod)
                {
                    auto code = scancodeMap[event.key.keysym.scancode];

                    if(code != XTScancode::Invalid)
                        sys.sendKey(static_cast<uint8_t>(code));
                }
                break;
            }
            case SDL_KEYUP:
            {
                if((event.key.keysym.mod & escMod) == escMod)
                {
                    // emulator shortcuts
                    switch(event.key.keysym.sym)
                    {
                        case SDLK_f:
                        {
                            // load next floppy
                            if(!nextFloppyImage.empty())
                            {
                                auto newPath = nextFloppyImage.front();
                                nextFloppyImage.splice(nextFloppyImage.end(), nextFloppyImage, nextFloppyImage.begin());

                                std::cout << "Swapping floppy 0 to " << newPath << "\n";
                                floppyIO.openDisk(0, newPath);
                            }
                            break;
                        }
                    }
                }
                else
                {
                    auto code = scancodeMap[event.key.keysym.scancode];

                    if(code != XTScancode::Invalid)
                        sys.sendKey(0x80 | static_cast<uint8_t>(code)); // break code
                }
                break;
            }

            case SDL_MOUSEMOTION:
                mouse.addMotion(event.motion.xrel, event.motion.yrel);
                break;

            case SDL_MOUSEBUTTONDOWN:
            case SDL_MOUSEBUTTONUP:
                if(event.button.button == SDL_BUTTON_LEFT)
                    mouse.setButton(0, event.button.state == SDL_PRESSED);
                else if(event.button.button == SDL_BUTTON_RIGHT)
                    mouse.setButton(1, event.button.state == SDL_PRESSED);
                break;

            case SDL_QUIT:
                quit = true;
                break;
        }
    }

    mouse.sync();
}

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    // RGB * 0xAA + I * 0x55
    // (except brown)
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

    auto ptr = screenData + line * 640 * 4;

    auto ptr32 = reinterpret_cast<uint32_t *>(ptr);
    auto pal32 = reinterpret_cast<const uint32_t *>(palette);

    for(int x = 0; x < w; x += 2)
    {
        auto in = *data++;
        int index = in & 0xF;
        *ptr32++ = pal32[index];

        index = in >> 4;
        *ptr32++ = pal32[index];
    }

    curScreenW = w;
}

bool FileFloppyIO::isPresent(int unit)
{
    return unit < maxDrives && file[unit];
}

bool FileFloppyIO::read(int unit, uint8_t *buf, uint8_t cylinder, uint8_t head, uint8_t sector)
{
    if(unit >= maxDrives)
        return false;

    int heads = doubleSided[unit] ? 2 : 1;
    auto lba = ((cylinder * heads + head) * sectorsPerTrack[unit]) + sector - 1;

    file[unit].clear();

    return file[unit].seekg(lba * 512).read(reinterpret_cast<char *>(buf), 512).gcount() == 512;
}

void FileFloppyIO::openDisk(int unit, std::string path)
{
    if(unit >= maxDrives)
        return;

    file[unit].close();

    file[unit].open(path);
    if(file[unit])
    {
        file[unit].seekg(0, std::ios::end);
        auto fdSize = file[unit].tellg();

        // try to work out geometry
        switch(fdSize / 1024)
        {
            case 160:
                doubleSided[unit] = false;
                sectorsPerTrack[unit] = 8;
                break;
            case 180:
                doubleSided[unit] = false;
                sectorsPerTrack[unit] = 9;
                break;
            case 360:
                doubleSided[unit] = true;
                sectorsPerTrack[unit] = 9;
                // could also be a single-sided 3.5-inch disk
                break;
            default:
                std::cerr << "unhandled floppy image size " << fdSize << "(" << fdSize / 1024 << "k)\n";
                // set... something
                doubleSided[unit] = false;
                sectorsPerTrack[unit] = 8;
                break;
        }

        std::cout << "using " << (doubleSided[unit] ? 2 : 1) << " head(s) " << sectorsPerTrack[unit] << " sectors/track for floppy image\n";
    }
}

bool FileFixedIO::isPresent(int unit)
{
    return unit < maxDrives && file[unit];
}

bool FileFixedIO::read(int unit, uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    file[unit].clear();

    return file[unit].seekg(lba * 512).read(reinterpret_cast<char *>(buf), 512).gcount() == 512;
}

bool FileFixedIO::write(int unit, const uint8_t *buf, uint32_t lba)
{
    if(unit >= maxDrives)
        return false;

    file[unit].clear();

    return file[unit].seekp(lba * 512).write(reinterpret_cast<const char *>(buf), 512).good();
}

void FileFixedIO::openDisk(int unit, std::string path)
{
    if(unit >= maxDrives)
        return;

    if(!std::filesystem::exists(path))
    {
        // new disk image
        file[unit].open(path, std::ios::in | std::ios::out | std::ios::trunc | std::ios::binary);

        // fill boot sector
        for(int i = 0; i < 512; i++)
            file[unit].put(0);
    }
    else
        file[unit].open(path, std::ios::in | std::ios::out | std::ios::binary);

    if(file[unit])
        std::cout << "Loaded fixed-disk " << unit << ": " << path << "\n";
}

int main(int argc, char *argv[])
{
    int screenWidth = 640;
    int screenHeight = 480;
    int textureHeight = 200;
    int screenScale = 2;

    uint32_t timeToRun = 0;
    bool timeLimit = false;

    std::string biosPath = "bios-xt.rom";
    std::string floppyPaths[FileFloppyIO::maxDrives];
    std::string fixedPaths[FileFixedIO::maxDrives];

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
        else if(arg == "--turbo")
            turbo = true;
        else if(arg == "--time" && i + 1 < argc)
        {
            timeLimit = true;
            timeToRun = std::stoi(argv[++i]) * 1000;
        }
        else if(arg == "--bios" && i + 1 < argc)
            biosPath = argv[++i];
        else if(arg.compare(0, 8, "--floppy") == 0 && arg.length() == 9 && i + 1 < argc)
        {
            int n = arg[8] - '0';
            if(n >= 0 && n < FileFloppyIO::maxDrives)
                floppyPaths[n] = argv[++i];
        }
        else if(arg == "--floppy-next" && i + 1 < argc)
        {
            // floppy image to load later
            nextFloppyImage.push_back(argv[++i]);
        }
        else if(arg.compare(0, 7, "--fixed") == 0 && arg.length() == 8 && i + 1 < argc)
        {
            int n = arg[7] - '0';
            if(n >= 0 && n < FileFixedIO::maxDrives)
                fixedPaths[n] = argv[++i];
        }
        else
            break;
    }

    // get base path
    std::string basePath;
    auto tmp = SDL_GetBasePath();
    if(tmp)
    {
        basePath = tmp;
        SDL_free(tmp);
    }

  
    // emu init
    auto &cpu = sys.getCPU();
    sys.addMemory(0, sizeof(ram), ram);

    std::ifstream biosFile(basePath + biosPath, std::ios::binary);

    if(biosFile)
    {
        biosFile.read(reinterpret_cast<char *>(biosROM), sizeof(biosROM));

        size_t readLen = biosFile.gcount();

        uint32_t biosBase = 0xF0000;
        // move shorter ROM to end (so reset vector is in the right place)
        if(readLen < sizeof(biosROM))
            biosBase += sizeof(biosROM) - readLen;

        sys.addMemory(biosBase, readLen, biosROM);
        biosFile.close();
    }
    else
    {
        std::cerr << biosPath << " not found in " << basePath << "\n";
        return 1;
    }

    // attempt to load BIOS rom for fixed-disk adapter
    biosFile.open(basePath + "fixed-disk.rom");
    if(biosFile)
    {
        std::cout << "loading fixed-disk adapter ROM at C8000\n";
        biosFile.read(reinterpret_cast<char *>(fixedDiskBIOSROM), sizeof(fixedDiskBIOSROM));
        sys.addMemory(0xC8000, sizeof(fixedDiskBIOSROM), fixedDiskBIOSROM);
    }

    // try to open floppy disk image(s)
    for(int i = 0; i < FileFloppyIO::maxDrives; i++)
    {
        if(!floppyPaths[i].empty())
        {
            floppyIO.openDisk(i, basePath + floppyPaths[i]);
        
            // add current image to end of floppy list so we can cycle
            if(i == 0 && !nextFloppyImage.empty())
                nextFloppyImage.push_back(floppyPaths[i]);
        }
    }

    // ... and fixed disks
    for(int i = 0; i < FileFixedIO::maxDrives; i++)
    {
        if(!fixedPaths[i].empty())
            fixedIO.openDisk(i, basePath + fixedPaths[i]);
    }

    for(auto &path : nextFloppyImage)
        path = basePath + path;
    
    fdc.setIOInterface(&floppyIO);
    fixDisk.setIOInterface(&fixedIO);

    cga.setScanlineCallback(scanlineCallback);

    sys.reset();

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        std::cerr << "Failed to init SDL!\n";
        return 1;
    }

    auto window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                   screenWidth * screenScale, screenHeight * screenScale,
                                   SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

    auto renderer = SDL_CreateRenderer(window, -1, turbo ? 0 : SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, screenWidth, textureHeight);

    // audio
    SDL_AudioSpec spec{};

    spec.freq = 22050;
    spec.format = AUDIO_S16;
    spec.channels = 1;
    spec.samples = 512;
    spec.callback = audioCallback;

    auto dev = SDL_OpenAudioDevice(nullptr, false, &spec, nullptr, 0);

    if(!dev)
    {
        std::cerr << "Failed to open audio: " << SDL_GetError() << "\n";
        quit = true;
    }

    if(!turbo)
        SDL_PauseAudioDevice(dev, 0);

    auto lastTick = SDL_GetTicks();
    auto startTime = SDL_GetTicks();

    auto checkTimeLimit = [timeLimit, &timeToRun]()
    {
        // fixed length benchmark
        if(timeLimit)
        {
            timeToRun -= 10;
            if(timeToRun == 0)
            {
                quit = true;
                return true;
            }
        }
        return false;
    };

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();
      
        if(turbo)
        {
            // push as fast as possible
            // avoid doing SDL stuff between updates
            while(now - lastTick < 10)
            {
                cpu.run(10);

                now = SDL_GetTicks();

                if(checkTimeLimit())
                    break;
            }
        }
        else
        {
            cpu.run(now - lastTick);

            cga.update();
            sys.updateForDisplay();
        }

        lastTick = now;

        // TODO: sync
        SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 4);
        SDL_RenderClear(renderer);
        SDL_Rect srcRect{0, 0, curScreenW, textureHeight};
        SDL_RenderCopy(renderer, texture, &srcRect, nullptr);
        SDL_RenderPresent(renderer);
    }

    if(timeLimit)
    {
        auto runTime = SDL_GetTicks() - startTime;
        printf("Ran for %ums\n", runTime);
    }

    SDL_CloseAudioDevice(dev);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
