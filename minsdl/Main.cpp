#include <fstream>
#include <iostream>
#include <string>

#include <SDL.h>

#include "CPU.h"

static bool quit = false;
static bool turbo = false;

static CPU cpu;

static uint8_t screenData[640 * 200 * 4];
static int curScreenW = 0;

static uint8_t biosROM[0x2000];

static void audioCallback(void *userdata, Uint8 *stream, int len)
{

}

static void pollEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_QUIT:
                quit = true;
                break;
        }
    }
}

static void scanlineCallback(const uint8_t *data, int line, int w)
{
    // RGB * 0xAA + I * 0x55
    // (except brown)
    static const uint8_t palette[16][3]
    {
        {0x00, 0x00, 0x00}, // black
        {0x00, 0x00, 0xAA}, // blue
        {0x00, 0xAA, 0x00}, // green
        {0x00, 0xAA, 0xAA}, // cyan
        {0xAA, 0x00, 0x00}, // red
        {0xAA, 0x00, 0xAA}, // magenta
        {0xAA, 0x55, 0x00}, // brown
        {0xAA, 0xAA, 0xAA}, // light grey

        {0x55, 0x55, 0x55}, // dark grey
        {0x55, 0x55, 0xFF}, // light blue
        {0x55, 0xFF, 0x55}, // light green
        {0x55, 0xFF, 0xFF}, // light cyan
        {0xFF, 0x55, 0x55}, // light red
        {0xFF, 0x55, 0xFF}, // light magenta
        {0xFF, 0xFF, 0x55}, // yellow
        {0xFF, 0xFF, 0xFF}, // white
    };

    auto ptr = screenData + line * 640 * 4;

    for(int x = 0; x < w; x += 2)
    {
        int index = *data & 0xF;

        *ptr++ = palette[index][2];
        *ptr++ = palette[index][1];
        *ptr++ = palette[index][0];
        ptr++;

        index = *data++ >> 4;
        
        *ptr++ = palette[index][2];
        *ptr++ = palette[index][1];
        *ptr++ = palette[index][0];
        ptr++;
    }

    curScreenW = w;
}

int main(int argc, char *argv[])
{
    int screenWidth = 640;
    int screenHeight = 480;
    int textureHeight = 200;
    int screenScale = 2;

    uint32_t timeToRun = 0;
    bool timeLimit = false;

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

    std::ifstream biosFile(basePath + "bios.rom", std::ios::binary);
    if(biosFile)
    {
        biosFile.read(reinterpret_cast<char *>(biosROM), sizeof(biosROM));

        cpu.getMem().setBIOSROM(biosROM);
    }
    else
    {
        std::cerr << "bios.rom not found in " << basePath << "\n";
        return 1;
    }

    cpu.getMem().setCGAScanlineCallback(scanlineCallback);

    cpu.reset();

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        std::cerr << "Failed to init SDL!\n";
        return 1;
    }

    auto window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                   screenWidth * screenScale, screenHeight * screenScale,
                                   SDL_WINDOW_RESIZABLE);

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

            cpu.getMem().updateForDisplay();
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
