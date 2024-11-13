#include <array>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"

#include "Display.h"

#include "config.h"

// DVI constants

#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

// mode
// active area needs to be consistent with (2x) DISPLAY_WIDTH/_HEIGHT
//#define MODE_H_SYNC_POLARITY 0 // unused, assumed to be active-low
#define MODE_H_FRONT_PORCH   16
#define MODE_H_SYNC_WIDTH    96
#define MODE_H_BACK_PORCH    48
#define MODE_H_ACTIVE_PIXELS 640

//#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH   10
#define MODE_V_SYNC_WIDTH    2
#define MODE_V_BACK_PORCH    33
#define MODE_V_ACTIVE_LINES  480

/*
#define MODE_H_TOTAL_PIXELS ( \
    MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
    MODE_H_BACK_PORCH  + MODE_H_ACTIVE_PIXELS \
)*/
#define MODE_V_TOTAL_LINES  ( \
    MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
    MODE_V_BACK_PORCH  + MODE_V_ACTIVE_LINES \
)

#define HSTX_CMD_RAW         (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT  (0x1u << 12)
#define HSTX_CMD_TMDS        (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP         (0xfu << 12)

// HSTX command lists

// Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
// pingponging and tripping up the IRQs.

static uint32_t vblank_line_vsync_off[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
    SYNC_V1_H1,
    HSTX_CMD_NOP
};

static uint32_t vblank_line_vsync_on[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V0_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V0_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
    SYNC_V0_H1,
    HSTX_CMD_NOP,
};

static const uint32_t vactive_line[] = {
    HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT | MODE_H_BACK_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_TMDS       | MODE_H_ACTIVE_PIXELS
};

// DMA logic

#define HSTX_DMA_CH_BASE 0
#define HSTX_NUM_DMA_CHANNELS 3

static uint8_t cur_dma_ch = HSTX_DMA_CH_BASE;

// pixel/line repeat
static uint8_t h_shift = 0;
static uint8_t new_h_shift = 0;

static uint v_scanline = HSTX_NUM_DMA_CHANNELS;
static uint32_t in_scanline = 0, last_in_scanline = 1;
static uint32_t in_scanline_step = 1 << 16;
static uint32_t new_scanline_step = 0;

static bool started = false;
static volatile bool do_render = true;
static volatile bool need_mode_change = false;
static uint8_t *cur_display_buffer = nullptr, *cur_update_buffer = nullptr;
static uint8_t framebuffer[640 * 240 * 2];

// temp buffer for expanding lines (pixel double)
// two scanlines + include the cmdlist(s) so we can avoid an irq
static uint32_t scanline_buffer[(MODE_H_ACTIVE_PIXELS * sizeof(uint16_t) + sizeof(vactive_line)) / sizeof(uint32_t) * 2];

static constexpr uint16_t col_565(uint8_t r, uint8_t g, uint8_t b) {
    return (r >> 3) | ((g >> 2) << 5) | ((b >> 3) << 11);
}

static uint16_t cga_palette[16]
{
    col_565(0x00, 0x00, 0x00), // black
    col_565(0x00, 0x00, 0xAA), // blue
    col_565(0x00, 0xAA, 0x00), // green
    col_565(0x00, 0xAA, 0xAA), // cyan
    col_565(0xAA, 0x00, 0x00), // red
    col_565(0xAA, 0x00, 0xAA), // magenta
    col_565(0xAA, 0x55, 0x00), // brown
    col_565(0xAA, 0xAA, 0xAA), // light grey

    col_565(0x55, 0x55, 0x55), // dark grey
    col_565(0x55, 0x55, 0xFF), // light blue
    col_565(0x55, 0xFF, 0x55), // light green
    col_565(0x55, 0xFF, 0xFF), // light cyan
    col_565(0xFF, 0x55, 0x55), // light red
    col_565(0xFF, 0x55, 0xFF), // light magenta
    col_565(0xFF, 0xFF, 0x55), // yellow
    col_565(0xFF, 0xFF, 0xFF), // white
};

static void __scratch_x("") dma_irq_handler() {
    // cur_dma_ch indicates the channel that just finished, which is the one
    // we're about to reload.
    dma_channel_hw_t *ch = &dma_hw->ch[cur_dma_ch];
    dma_hw->intr = 1u << cur_dma_ch;

    if(cur_dma_ch + 1 == HSTX_DMA_CH_BASE + HSTX_NUM_DMA_CHANNELS)
        cur_dma_ch = HSTX_DMA_CH_BASE;
    else
        cur_dma_ch++;

    if (v_scanline >= MODE_V_FRONT_PORCH && v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH)) {
        ch->read_addr = (uintptr_t)vblank_line_vsync_on;
    } else if (v_scanline < MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH) {
        ch->read_addr = (uintptr_t)vblank_line_vsync_off;
        ch->transfer_count = std::size(vblank_line_vsync_off);
    } else {
        auto display_line = in_scanline >> 16;
        in_scanline += in_scanline_step;
        bool first = display_line != last_in_scanline;
        last_in_scanline = display_line;

        const auto line_buf_size_words = sizeof(scanline_buffer) / sizeof(uint32_t) / 2;
        auto temp_ptr = scanline_buffer + (display_line & 1) * line_buf_size_words;
        ch->read_addr = (uintptr_t)temp_ptr;

        ch->transfer_count = std::size(vactive_line) + (MODE_H_ACTIVE_PIXELS * 2) / sizeof(uint32_t);

        // expand line if needed
        if(first) {
            auto w = MODE_H_ACTIVE_PIXELS >> h_shift;
            auto fb_line_ptr = cur_display_buffer + display_line * w;

            temp_ptr += std::size(vactive_line);
            // palette lookup
            if(h_shift == 0) {
                for(int i = 0; i < w / 2; i++) {
                    auto px = *fb_line_ptr++;
                    *temp_ptr++ = cga_palette[px & 0xF] | cga_palette[px >> 4] << 16;
                }
            } else {
                for(int i = 0; i < w / 2; i++) {
                    auto px = *fb_line_ptr++;
                    *temp_ptr++ = cga_palette[px & 0xF] | cga_palette[px & 0xF] << 16;
                    *temp_ptr++ = cga_palette[px >> 4] | cga_palette[px >> 4] << 16;
                }
            }
        }
    }

    v_scanline++;

    if(v_scanline == MODE_V_TOTAL_LINES) {
        v_scanline = 0;
        in_scanline = 0;
    } else if(v_scanline == 2) {
        // new frame, swap buffers and trigger render
        // wait until scanline 2 so that there are no active lines in progress

        if(!do_render) {
            std::swap(cur_update_buffer, cur_display_buffer);
            do_render = true;
        }

        // set h/v shift
        if(need_mode_change) {
            h_shift = new_h_shift;
            in_scanline_step = new_scanline_step;

            need_mode_change = false;
            new_h_shift = 0xFF;
            new_scanline_step = 0;
        }
    }
}

void init_display() {
    // reset HSTX to make sure it's in a good state
    reset_unreset_block_num_wait_blocking(RESET_HSTX);

    // divide down if we're overclocking (we probably are)
#if OVERCLOCK_250
    clock_configure(clk_hstx, 0, CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS, 250000000, 125000000);
#endif

    // Configure HSTX's TMDS encoder for RGB565
    // (it starts from bit 7)
    hstx_ctrl_hw->expand_tmds =
        4  << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB | // R
        29 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB   | //
        5  << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB | // G
        3  << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB   | //
        4  << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB | // B
        8  << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;    //

    // Pixels (TMDS) come in 2 16-bit chunks. Control symbols (RAW) are an
    // entire 32-bit word.
    hstx_ctrl_hw->expand_shift =
        2  << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
        16 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
        1  << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
        0  << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

    // Serial output config: clock period of 5 cycles, pop from command
    // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
    hstx_ctrl_hw->csr = 0;
    hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EXPAND_EN_BITS |
        5u << HSTX_CTRL_CSR_CLKDIV_LSB |
        5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
        2u << HSTX_CTRL_CSR_SHIFT_LSB |
        HSTX_CTRL_CSR_EN_BITS;


    // HSTX outputs 0 through 7 appear on GPIO 12 through 19.
    // Pinout on Pico DVI sock:
    //
    //   GP12 D0+  GP13 D0-
    //   GP14 CK+  GP15 CK-
    //   GP16 D2+  GP17 D2-
    //   GP18 D1+  GP19 D1-

    // Assign clock pair to two neighbouring pins:
    hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
    for(uint lane = 0; lane < 3; ++lane) {
        // For each TMDS lane, assign it to the correct GPIO pair based on the
        // desired pinout:
        static const int lane_to_output_bit[3] = {0, 6, 4};
        int bit = lane_to_output_bit[lane];
        // Output even bits during first half of each HSTX cycle, and odd bits
        // during second half. The shifter advances by two bits each cycle.
        uint32_t lane_data_sel_bits =
            (lane * 10    ) << HSTX_CTRL_BIT0_SEL_P_LSB |
            (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
        // The two halves of each pair get identical data, but one pin is inverted.
        hstx_ctrl_hw->bit[bit    ] = lane_data_sel_bits;
        hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
    }

    for(int i = 12; i <= 19; ++i)
        gpio_set_function(i, GPIO_FUNC_HSTX);

    // All channels are set up identically, to transfer a whole scanline and
    // then chain to the next channel. Each time a channel finishes, we
    // reconfigure the one that just finished, meanwhile the next channel
    // is already making progress.

    for(int i = 0; i < HSTX_NUM_DMA_CHANNELS; i++) {
        dma_channel_claim(HSTX_DMA_CH_BASE + i);
        dma_channel_config c;
        c = dma_channel_get_default_config(HSTX_DMA_CH_BASE + i);

        int next_chan = i == (HSTX_NUM_DMA_CHANNELS - 1) ? 0 : i + 1;

        channel_config_set_chain_to(&c, HSTX_DMA_CH_BASE + next_chan);
        channel_config_set_dreq(&c, DREQ_HSTX);
        dma_channel_configure(
            HSTX_DMA_CH_BASE + i,
            &c,
            &hstx_fifo_hw->fifo,
            vblank_line_vsync_off,
            count_of(vblank_line_vsync_off),
            false
        );
    }

    const unsigned chan_mask = (1 << HSTX_NUM_DMA_CHANNELS) - 1;

    dma_hw->ints0 = (chan_mask << HSTX_DMA_CH_BASE);
    dma_hw->inte0 = (chan_mask << HSTX_DMA_CH_BASE);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // fill line headers
    const auto line_buf_size = sizeof(scanline_buffer) / 2;
    auto line_buf0 = scanline_buffer;
    auto line_buf1 = scanline_buffer + line_buf_size / sizeof(uint32_t);

    for(size_t i = 0; i < std::size(vactive_line); i++)
        *line_buf0++ = *line_buf1++ = vactive_line[i];

    // set irq to highest priority
    irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);

    cur_display_buffer = framebuffer;
    cur_update_buffer = framebuffer + 640 * 240;
}

void set_display_size(int w, int h) {
    // prevent buffer swap while we're doing this
    do_render = true;

    // set h shift/v scale
    new_h_shift = 0;
    
    new_scanline_step = (h << 16) / MODE_V_ACTIVE_LINES;

    while(MODE_H_ACTIVE_PIXELS >> new_h_shift > w)
        new_h_shift++;

    // check if we're actually changing scale
    if(new_scanline_step == in_scanline_step && new_h_shift == h_shift) {
        new_h_shift = 0xFF;
        new_scanline_step = 0;
        return;
    }

    // don't do it yet if already started
    // (will set need_mode_change after next render)
    if(started)
        return;

    h_shift = new_h_shift;
    in_scanline_step = new_scanline_step;
    new_h_shift = 0;
}

void update_display() {

    // start dma after first render
    if(!started) {
        started = true;
        dma_channel_start(HSTX_DMA_CH_BASE);
    } else if(new_h_shift != 0xFF) {
        need_mode_change = true;
    }
    do_render = false;

    // check if dma channels have encountered a read error and reset
    // usually this happens because of a breakpoint
    bool error = false;

    for(int i = 0; i < HSTX_NUM_DMA_CHANNELS; i++)
        error = error || (dma_hw->ch[HSTX_DMA_CH_BASE + i].al1_ctrl & DMA_CH0_CTRL_TRIG_READ_ERROR_BITS);

    if(error) {
        for(int i = 0; i < HSTX_NUM_DMA_CHANNELS; i++)
            hw_set_bits(&dma_hw->ch[HSTX_DMA_CH_BASE + i].al1_ctrl, DMA_CH0_CTRL_TRIG_READ_ERROR_BITS);

        // disable/enable HSTX
        hw_clear_bits(&hstx_ctrl_hw->csr, HSTX_CTRL_CSR_EN_BITS);
        hw_set_bits(&hstx_ctrl_hw->csr, HSTX_CTRL_CSR_EN_BITS);

        // restart DMA
        v_scanline = 2;
        cur_dma_ch = HSTX_DMA_CH_BASE;

        for(int i = 0; i < HSTX_NUM_DMA_CHANNELS; i++) {
            dma_channel_set_read_addr(HSTX_DMA_CH_BASE + i, vblank_line_vsync_off, false);
            dma_channel_set_trans_count(HSTX_DMA_CH_BASE + i, std::size(vblank_line_vsync_off), false);
        }

        dma_channel_start(HSTX_DMA_CH_BASE);
    }
}

bool display_render_needed() {
    return do_render;
}

uint8_t *display_get_framebuffer() {
    return cur_update_buffer;
}
