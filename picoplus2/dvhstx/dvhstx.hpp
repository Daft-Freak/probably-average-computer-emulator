#pragma once

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

// DVI HSTX driver for use with Pimoroni PicoGraphics
// modifed to NOT be used with PicoGraphics :)

namespace pimoroni {
  typedef uint32_t RGB888;

  // Digital Video using HSTX
  // Valid screen modes are:
  //   Pixel doubled: 640x480 (60Hz), 720x480 (60Hz), 720x400 (70Hz), 720x576 (50Hz), 
  //                  800x600 (60Hz), 800x480 (60Hz), 800x450 (60Hz), 960x540 (60Hz), 1024x768 (60Hz)
  //   Pixel doubled or quadrupled: 1280x720 (50Hz)
  //
  // Giving valid resolutions:
  //   320x180, 640x360 (well supported, square pixels on a 16:9 display)
  //   480x270, 400x225 (sometimes supported, square pixels on a 16:9 display)
  //   320x240, 360x240, 360x200, 360x288, 400x300, 512x384 (well supported, but pixels aren't square)
  //   400x240 (sometimes supported, pixels aren't square)
  //
  // Note that the double buffer is in RAM, so 640x360 uses almost all of the available RAM.
  class DVHSTX {
  public:
    static constexpr int PALETTE_SIZE = 256;

    enum Mode {
      MODE_PALETTE = 2,
      MODE_RGB565 = 1,
      MODE_RGB888 = 3,
    };

    //--------------------------------------------------
    // Variables
    //--------------------------------------------------
  protected:
    friend void vsync_callback();

    uint16_t display_width = 320;
    uint16_t display_height = 180;
    uint16_t frame_width = 320;
    uint16_t frame_height = 180;
    uint8_t frame_bytes_per_pixel = 2;
    uint8_t bank = 0;
    uint8_t h_repeat = 4;
    uint8_t v_repeat = 4;
    Mode mode = MODE_RGB565;

  public:
    DVHSTX()
    {}

    //--------------------------------------------------
    // Methods
    //--------------------------------------------------
    public:
      uint8_t *get_back_buffer();

      // 256 colour palette mode.
      void set_palette(RGB888 new_palette[PALETTE_SIZE]);
      void set_palette_colour(uint8_t entry, RGB888 colour);
      RGB888* get_palette();

      void clear();

      bool init(uint16_t width, uint16_t height, Mode mode = MODE_RGB565);

      // Wait for vsync and then flip the buffers
      void flip_blocking();

      // Flip immediately without waiting for vsync
      void flip_now();

      void wait_for_vsync();

      // flip_async queues a flip to happen next vsync but returns without blocking.
      // You should call wait_for_flip before doing any more reads or writes, defining sprites, etc.
      void flip_async();
      void wait_for_flip();

      // DMA handlers, should not be called externally
      void gfx_dma_handler();

    private:
      RGB888 palette[PALETTE_SIZE];

      uint8_t* frame_buffer_display;
      uint8_t* frame_buffer_back;

      void display_setup_clock();

      // DMA scanline filling
      uint ch_num = 0;
      int line_num = -1;

      volatile uint v_scanline = 2;
      volatile bool flip_next;

      uint32_t* line_buffers;
      const struct dvi_timing* timing_mode;
      uint v_inactive_total;
      uint v_total_active_lines;

      uint h_repeat_shift;
      uint v_repeat_shift;
      int line_bytes_per_pixel;

      uint32_t* display_palette = nullptr;
  };
}
