#pragma once

// for SD code
#define OVERCLOCK_250 1

#ifdef SOLDERPARTY_RP2350_STAMP_XL
// these are not the slot on the carrier, it conflicts with the PSRAM CS
#define SD_SCK    39
#define SD_MOSI   37
#define SD_MISO   38
#define SD_CS     36

#define PSRAM_CS_PIN 8

#elif defined(PIMORONI_PICO_PLUS2_RP2350)
#define SD_SCK  2
#define SD_MOSI 4
#define SD_MISO 3
#define SD_CS   5

#define PSRAM_CS_PIN PIMORONI_PICO_PLUS2_PSRAM_CS_PIN
#else
#error "No board configuration!"
#endif
