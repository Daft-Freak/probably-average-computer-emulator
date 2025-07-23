#pragma once

// PSRAM config first as that only depends on the board
// and not any additional thing it's plugged into
#ifdef PIMORONI_PICO_PLUS2_RP2350
#define PSRAM_CS_PIN PIMORONI_PICO_PLUS2_PSRAM_CS_PIN
#elif defined(PIMORONI_PICO_PLUS2_W_RP2350)
#define PSRAM_CS_PIN PIMORONI_PICO_PLUS2_W_PSRAM_CS_PIN
#elif defined(SOLDERPARTY_RP2350_STAMP_XL)
#define PSRAM_CS_PIN 8
#elif !defined(DISABLE_PSRAM)
#error "No PSRAM CS!"
#endif

#ifdef EXTRA_BOARD_STAMP_CARRIER
#define DVI_CLK_P 14
#define DVI_D0_P  12
#define DVI_D1_P  18
#define DVI_D2_P  16

// these are not the slot on the carrier, it conflicts with the PSRAM CS
#define SD_SCK    39
#define SD_MOSI   37
#define SD_MISO   38
#define SD_CS     36

#elif defined(EXTRA_BOARD_VGABOARD)
#define AUDIO_I2S_PIO 0
#define AUDIO_I2S_CLOCK_PIN_BASE 27
#define AUDIO_I2S_DATA_PIN       26

#define SD_SCK     5
#define SD_MOSI   18
#define SD_MISO   19
#define SD_CS     22

#elif defined(PIMORONI_PICO_PLUS2_RP2350)
// as I was using a mess of jumper wires, there's not really a right answer here
#define DVI_CLK_P 14
#define DVI_D0_P  12
#define DVI_D1_P  18
#define DVI_D2_P  16

#define SD_SCK  2
#define SD_MOSI 4
#define SD_MISO 3
#define SD_CS   5

#else
#error "No board configuration!"
#endif
