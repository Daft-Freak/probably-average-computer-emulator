# Probably Average Computer Emulator

## Features

- Mostly complete 8088 CPU core
- Basic chipset support (PIC/PIT/PPI), DMA is mostly missing so disk support is a bit of a hack]
- Keyboard input
- Colour Graphics Adapter
- Some floppy controller support
- Some fixed disk adapter support
- A basic serial mouse
- An "Above Board" for up to 8MB of expanded memory (TESTAB says it's a "Matched Memory Classic")

## BIOS

A few BIOS files are required:
- `bios-xt.rom` - _Theoretically_ any XT-compatible BIOS. If using the original IBM BIOS, this is both ROMs merged together.
- `fixed-disk-bios.rom` - The BIOS for the IBM Fixed Disk Adapter, required to emulate it. (Optional if you don't want a hard drive)

The location of these files depends on the frontend being used.

## SDL2

For running on... a PC, the only dependency is SDL2.

Supports up to four floppy drives and two hard drives. The BIOS files should be placed next to the executable.

### Building

```
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build
```

### Command Line Options

- `--turbo` - Run as fast as possible
- `--bios name.rom` - Specify an alternate BIOS file
- `--floppyN name.img` Specify an image file for floppy drive N (0-3)
- `--floppy-next name.img` Specify an image file to be loaded in floppy drive 0 later, can be used multiple times (RCTRL+RSHIFT+f cycles through)
- `--fixedN name.img` Specify an image file for fixed/hard disk N (0-1)

For example:
```
PACE_SDL --fixed0 hd0.img --floppy-next disk1.img --floppy-next disk2.img
```
would boot from `hd0.img` and allow installing something from the two floppy images later.

## PicoVision

The more interesting frontend, depends on the pico-sdk.

Currently only supports a single hard disk, loaded from `hd0.img` on the SD card. Keyboard/mouse input is supported through USB HID. The BIOS files should be placed at the root of the repository before building.

Supports the full 640k of memory and 6MB of expanded memory through paging 16k blocks in and out of the PicoVision's PSRAMs. 192k of memory is kept in the Pico's RAM at once, software that accesses a lot of RAM frequently may cause display glitches as I have to force a wait for vsync to write the other PSRAM. (If possible, I try to flush dirty memory at the end of a frame to avoid this).

### Building

```
cmake -B build.picovision -DCMAKE_BUILD_TYPE=Release -DPICO_SDK_PATH=path/to/pico-sdk .
cmake --build build.picovision
```

## "Pico 2"
Theoretically any RP2350-based board with PSRAM and DVI/DPI output. Similar to the PicoVision build but without the need for paging memory. Also generally faster.

### Building (Stamp XL + Carrier)

```
cmake -B build.pico2 -DCMAKE_BUILD_TYPE=Release -DPICO_SDK_PATH=path/to/pico-sdk -DPICO_BOARD=solderparty_rp2350_stamp_xl .
cmake --build build.pico2
```

### Building (Pico Plus 2 + VGA Board)

```
cmake -B build.pico2 -DCMAKE_BUILD_TYPE=Release -DPICO_SDK_PATH=path/to/pico-sdk -DPICO_BOARD=pimoroni_pico_plus2_rp2350 -DEXTRA_BOARD=vgaboard .
cmake --build build.pico2
```

### Additional Options

```sh
-DPICO2_EMULATOR_ON_CORE1=1 # Run emulator on core1
-DPICO2_CPU_IN_RAM=1        # Move main CPU interpreter+memory access code to RAM
```