// largely "borrowed" from 32blit driver

#include <array>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "aps6404.hpp"
#include "swd_load.hpp"
#include "pico-stick.h"

#include "Display.h"

static const uint8_t cga_palette[16][3]
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

// pins
static constexpr uint CS     = 17;
static constexpr uint D0     = 19;
static constexpr uint VSYNC  = 16;
static constexpr uint RAM_SEL = 8;

static constexpr uint I2C_SDA = 6;
static constexpr uint I2C_SCL = 7;

// i2c
static constexpr uint I2C_ADDR = 0x0D;
static constexpr uint I2C_REG_SET_RES = 0xFC;
static constexpr uint I2C_REG_START = 0xFD;
static constexpr uint I2C_REG_STOP = 0xFF;

static constexpr uint32_t base_address = 0x10000;

static pimoroni::APS6404 ram(CS, D0, pio1);
static uint8_t ram_bank = 0;

static bool display_enabled = false;
static uint8_t need_mode_change = 2;

static volatile bool do_render = true;

static void vsync_callback(uint gpio, uint32_t events){
    if(!do_render) {
        ram_bank ^= 1;
        gpio_put(RAM_SEL, ram_bank);

        do_render = true;
    }
}

static void write_frame_setup(uint16_t width, uint16_t height, int pixel_stride, uint8_t h_repeat, uint8_t v_repeat) {
    constexpr int buf_size = 32;
    uint32_t buf[buf_size];

    int dv_format = 2; // 5 bit palette

    uint32_t full_width = width * h_repeat;
    buf[0] = 0x4F434950; // "PICO"

    // setup
    buf[1] = 0x01000101 + ((uint32_t)v_repeat << 16);
    buf[2] = full_width << 16;
    buf[3] = (uint32_t)height << 16;

    // frame table header
    buf[4] = 0x00000001; // 1 frame, start at frame 0
    buf[5] = 0x00010000 + height + ((uint32_t)ram_bank << 24); // frame rate divider 1
    buf[6] = 0x00000001; // 1 palette, don't advance, 0 sprites

    ram.write(0, buf, 7 * 4);
    ram.wait_for_finish_blocking();

    // write frame table
    uint frame_table_addr = 4 * 7;
    
    for(int y = 0; y < height; y += buf_size) {
        int step = std::min(buf_size, height - y);
        for(int i = 0; i < step; i++) {
            uint32_t line_addr = base_address + (y + i) * width * pixel_stride;
            buf[i] = dv_format << 27 | h_repeat << 24 | line_addr;
        }

        ram.write(frame_table_addr, buf, step * 4);
        ram.wait_for_finish_blocking();
        frame_table_addr += 4 * step;
    }

    // write palette
    ram.write(frame_table_addr, (uint32_t *)cga_palette, sizeof(cga_palette));
}

void init_display() {
    gpio_init(RAM_SEL);
    gpio_put(RAM_SEL, 0);
    gpio_set_dir(RAM_SEL, GPIO_OUT);

    gpio_init(VSYNC);
    gpio_set_dir(VSYNC, GPIO_IN);
    gpio_set_irq_enabled_with_callback(VSYNC, GPIO_IRQ_EDGE_RISE, true, vsync_callback);

    sleep_ms(200);
    swd_load_program(section_addresses, section_data, section_data_len, std::size(section_data_len), 0x20000001, 0x15004000, true);

    // init RAM
    ram.init();
    sleep_us(100);

    gpio_put(RAM_SEL, 1);
    ram.init();
    sleep_us(100);

    gpio_put(RAM_SEL, 0);
    sleep_ms(100);

    // i2c init
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);

    uint8_t resolution = 0; // 640x480
    uint8_t buf[2] = {I2C_REG_SET_RES, resolution};
    i2c_write_blocking(i2c1, I2C_ADDR, buf, 2, false);
}

void write_display(int x, int y, int count, uint8_t *data)
{
    ram.write(base_address + (x + y * 640) * 1, (uint32_t *)data, count);
}

void update_display() {

    ram.wait_for_finish_blocking();

    // handle mode change
    if(need_mode_change) {
        // hardcoded 640x240
        uint8_t h_repeat = 1, v_repeat = 2;
        write_frame_setup(640, 240, 1, h_repeat, v_repeat);
        need_mode_change--;
    }

    // enable display after first render
    if(!display_enabled) {
        // swap banks now
        ram_bank ^= 1;
        gpio_put(RAM_SEL, ram_bank);

        uint8_t buf[2] = {I2C_REG_START, 1};
        i2c_write_blocking(i2c1, I2C_ADDR, buf, 2, false);
        display_enabled = true;
    } else
        do_render = false;
}

bool display_render_needed() {
    return do_render;
}
