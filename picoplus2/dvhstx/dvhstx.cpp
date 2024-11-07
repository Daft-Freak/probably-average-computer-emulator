#include <cstdio>
#include <string.h>
#include <pico/stdlib.h>

extern "C" {
#include <pico/lock_core.h>
}

#include <algorithm>
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/sio.h"

#include "hardware/structs/ioqspi.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"

#include "dvi.hpp"
#include "dvhstx.hpp"

using namespace pimoroni;


// ----------------------------------------------------------------------------
// HSTX command lists

// Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
// pingponging and tripping up the IRQs.

static uint32_t vblank_line_vsync_off[] = {
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H1
};

static uint32_t vblank_line_vsync_on[] = {
    HSTX_CMD_RAW_REPEAT,
    SYNC_V0_H1,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V0_H0,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V0_H1
};

static uint32_t vactive_line_header[] = {
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H1,
    HSTX_CMD_TMDS      
};

static uint32_t vactive_text_line_header[] = {
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT,
    SYNC_V1_H1,
    HSTX_CMD_RAW | 6,
    BLACK_PIXEL_A,
    BLACK_PIXEL_B,
    BLACK_PIXEL_A,
    BLACK_PIXEL_B,
    BLACK_PIXEL_A,
    BLACK_PIXEL_B,
    HSTX_CMD_TMDS
};

#define NUM_FRAME_LINES 2
#define NUM_CHANS 3

static DVHSTX* display = nullptr;

// ----------------------------------------------------------------------------
// DMA logic

void __scratch_x("display") dma_irq_handler() {
    display->gfx_dma_handler();
}

void __scratch_x("display") DVHSTX::gfx_dma_handler() {
    // ch_num indicates the channel that just finished, which is the one
    // we're about to reload.
    dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
    dma_hw->intr = 1u << ch_num;
    if (++ch_num == NUM_CHANS) ch_num = 0;

    if (v_scanline >= timing_mode->v_front_porch && v_scanline < (timing_mode->v_front_porch + timing_mode->v_sync_width)) {
        ch->read_addr = (uintptr_t)vblank_line_vsync_on;
        ch->transfer_count = count_of(vblank_line_vsync_on);
    } else if (v_scanline < v_inactive_total) {
        ch->read_addr = (uintptr_t)vblank_line_vsync_off;
        ch->transfer_count = count_of(vblank_line_vsync_off);
    } else {
        const int y = (v_scanline - v_inactive_total) >> v_repeat_shift;
        const int new_line_num = (v_repeat_shift == 0) ? ch_num : (y & (NUM_FRAME_LINES - 1));
        const uint line_buf_total_len = ((timing_mode->h_active_pixels * line_bytes_per_pixel) >> 2) + count_of(vactive_line_header);

        ch->read_addr = (uintptr_t)&line_buffers[new_line_num * line_buf_total_len];
        ch->transfer_count = line_buf_total_len;

        // Fill line buffer
        if (line_num != new_line_num)
        {
            line_num = new_line_num;
            uint32_t* dst_ptr = &line_buffers[line_num * line_buf_total_len + count_of(vactive_line_header)];

            if (line_bytes_per_pixel == 2) {
                uint16_t* src_ptr = (uint16_t*)&frame_buffer_display[y * 2 * (timing_mode->h_active_pixels >> h_repeat_shift)];
                for (uint i = 0; i < timing_mode->h_active_pixels >> 1; i += 2) {
                    uint32_t val = (uint32_t)(*src_ptr++) * 0x10001;
                    *dst_ptr++ = val;
                    *dst_ptr++ = val;
                }
            }
            else if (line_bytes_per_pixel == 4) {
                uint8_t* src_ptr = &frame_buffer_display[y * (timing_mode->h_active_pixels >> h_repeat_shift)];
                if (h_repeat_shift == 2) {
                    for (uint i = 0; i < timing_mode->h_active_pixels; i += 4) {
                        uint32_t val = display_palette[*src_ptr++];
                        *dst_ptr++ = val;
                        *dst_ptr++ = val;
                        *dst_ptr++ = val;
                        *dst_ptr++ = val;
                    }
                }
                else if(h_repeat_shift == 1) {
                    for (uint i = 0; i < timing_mode->h_active_pixels; i += 2) {
                        uint32_t val = display_palette[*src_ptr++];
                        *dst_ptr++ = val;
                        *dst_ptr++ = val;
                    }
                } else {
                    for (uint i = 0; i < timing_mode->h_active_pixels; i++) {
                        uint32_t val = display_palette[*src_ptr++];
                        *dst_ptr++ = val;
                    }
                }
            }
        }
    }

    if (++v_scanline == v_total_active_lines) {
        v_scanline = 0;
        line_num = -1;
        if (flip_next) {
            flip_next = false;
            display->flip_now();
        }
        __sev();
    }
}

// ----------------------------------------------------------------------------
// Experimental clock config

static void __no_inline_not_in_flash_func(set_qmi_timing)() {
    // Make sure flash is deselected - QMI doesn't appear to have a busy flag(!)
    while ((ioqspi_hw->io[1].status & IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_BITS) != IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_BITS)
        ;

    qmi_hw->m[0].timing = 0x40000202;
    //qmi_hw->m[0].timing = 0x40000101;
    // Force a read through XIP to ensure the timing is applied
    volatile uint32_t* ptr = (volatile uint32_t*)0x14000000;
    (void) *ptr;
}

void DVHSTX::display_setup_clock() {
    // Before messing with clock speeds ensure QSPI clock is nice and slow
    hw_write_masked(&qmi_hw->m[0].timing, 6, QMI_M0_TIMING_CLKDIV_BITS);

    // We're going to go fast, boost the voltage a little
    vreg_set_voltage(VREG_VOLTAGE_1_15);

    // Force a read through XIP to ensure the timing is applied before raising the clock rate
    volatile uint32_t* ptr = (volatile uint32_t*)0x14000000;
    (void) *ptr;

    const uint32_t dvi_clock_khz = timing_mode->bit_clk_khz >> 1;
    uint vco_freq, post_div1, post_div2;
    if (!check_sys_clock_khz(dvi_clock_khz, &vco_freq, &post_div1, &post_div2))
        panic("System clock of %u kHz cannot be exactly achieved", dvi_clock_khz);
    const uint32_t freq = vco_freq / (post_div1 * post_div2);

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1)
        tight_loop_contents();
    hw_write_masked(&clocks_hw->clk[clk_ref].ctrl, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_ref].selected != 0x4)
        tight_loop_contents();

    // Stop the other clocks so we don't worry about overspeed
    clock_stop(clk_usb);
    clock_stop(clk_adc);
    clock_stop(clk_peri);
    clock_stop(clk_hstx);

    // Set the sys PLL to the requested freq, set USB PLL to 528MHz
    pll_init(pll_sys, PLL_COMMON_REFDIV, vco_freq, post_div1, post_div2);
    pll_init(pll_usb, PLL_COMMON_REFDIV, 1584 * MHZ, 3, 1);

    const uint32_t usb_pll_freq = 528 * MHZ;

    // CLK SYS = PLL USB 528MHz / 2 = 264MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    usb_pll_freq, usb_pll_freq / 2);

    // CLK PERI = PLL USB 528MHz / 4 = 132MHz
    clock_configure(clk_peri,
                    0, // Only AUX mux on ADC
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    usb_pll_freq, usb_pll_freq / 4);

    // CLK USB = PLL USB 528MHz / 11 = 48MHz
    clock_configure(clk_usb,
                    0, // No GLMUX
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    usb_pll_freq,
                    USB_CLK_KHZ * KHZ);

    // CLK ADC = PLL USB 528MHz / 11 = 48MHz
    clock_configure(clk_adc,
                    0, // No GLMUX
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    usb_pll_freq,
                    USB_CLK_KHZ * KHZ);

    // CLK HSTX = Requested freq
    clock_configure(clk_hstx,
                    0,
                    CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq, freq);

    // Now we are running fast set fast QSPI clock and read delay
    set_qmi_timing();
}

uint8_t *DVHSTX::get_back_buffer()
{
    return frame_buffer_back;
}

void DVHSTX::set_palette(RGB888 new_palette[PALETTE_SIZE])
{
    memcpy(palette, new_palette, PALETTE_SIZE * sizeof(RGB888));
}

void DVHSTX::set_palette_colour(uint8_t entry, RGB888 colour)
{
    palette[entry] = colour;
}

RGB888* DVHSTX::get_palette()
{
    return palette;
}

void DVHSTX::clear()
{
    memset(frame_buffer_back, 0, frame_width * frame_height * frame_bytes_per_pixel);
}

bool DVHSTX::init(uint16_t width, uint16_t height, Mode mode_, bool double_buffer)
{
    display_width = width;
    display_height = height;
    frame_width = width;
    frame_height = height;
    mode = mode_;

    timing_mode = nullptr;
    if (width == 320 && height == 180) {
        h_repeat_shift = 2;
        v_repeat_shift = 2;
        timing_mode = &dvi_timing_1280x720p_rb_50hz;
    }
    else if (width == 640 && height == 360) {
        h_repeat_shift = 1;
        v_repeat_shift = 1;
        timing_mode = &dvi_timing_1280x720p_rb_50hz;
    }
    else if (width == 480 && height == 270) {
        h_repeat_shift = 2;
        v_repeat_shift = 2;
        timing_mode = &dvi_timing_1920x1080p_rb2_30hz;
    }
    else
    {
        uint16_t full_width = display_width;
        uint16_t full_height = display_height;
        h_repeat_shift = 0;
        v_repeat_shift = 0;

        if (display_width < 640) {
            h_repeat_shift = 1;
            full_width *= 2;
        }

        if (display_height < 400) {
            v_repeat_shift = 1;
            full_height *= 2;
        }

        if (full_width == 640) {
            if (full_height == 480) timing_mode = &dvi_timing_640x480p_60hz;
        }
        else if (full_width == 720) {
            if (full_height == 480) timing_mode = &dvi_timing_720x480p_60hz;
            else if (full_height == 400) timing_mode = &dvi_timing_720x400p_70hz;
            else if (full_height == 576) timing_mode = &dvi_timing_720x576p_50hz;
        }
        else if (full_width == 800) {
            if (full_height == 600) timing_mode = &dvi_timing_800x600p_60hz;
            else if (full_height == 480) timing_mode = &dvi_timing_800x480p_60hz;
            else if (full_height == 450) timing_mode = &dvi_timing_800x450p_60hz;
        }
        else if (full_width == 960) {
            if (full_height == 540) timing_mode = &dvi_timing_960x540p_60hz;
        }
        else if (full_width == 1024) {
            if (full_height == 768) timing_mode = &dvi_timing_1024x768_rb_60hz;
        }
    }

    if (!timing_mode) {
        printf("Unsupported resolution %dx%d", width, height);
        return false;
    }

    display = this;
    display_palette = get_palette();
    
    display_setup_clock();

    stdio_init_all();

    v_inactive_total = timing_mode->v_front_porch + timing_mode->v_sync_width + timing_mode->v_back_porch;
    v_total_active_lines = v_inactive_total + timing_mode->v_active_lines;
    v_repeat = 1 << v_repeat_shift;
    h_repeat = 1 << h_repeat_shift;

    vblank_line_vsync_off[0] |= timing_mode->h_front_porch;
    vblank_line_vsync_off[2] |= timing_mode->h_sync_width;
    vblank_line_vsync_off[4] |= timing_mode->h_back_porch + timing_mode->h_active_pixels;

    vblank_line_vsync_on[0] |= timing_mode->h_front_porch;
    vblank_line_vsync_on[2] |= timing_mode->h_sync_width;
    vblank_line_vsync_on[4] |= timing_mode->h_back_porch + timing_mode->h_active_pixels;

    vactive_line_header[0] |= timing_mode->h_front_porch;
    vactive_line_header[2] |= timing_mode->h_sync_width;
    vactive_line_header[4] |= timing_mode->h_back_porch;
    vactive_line_header[6] |= timing_mode->h_active_pixels;

    vactive_text_line_header[0] |= timing_mode->h_front_porch;
    vactive_text_line_header[2] |= timing_mode->h_sync_width;
    vactive_text_line_header[4] |= timing_mode->h_back_porch;
    vactive_text_line_header[7+6] |= timing_mode->h_active_pixels - 6;

    switch (mode) {
    case MODE_RGB565:
        frame_bytes_per_pixel = 2;
        line_bytes_per_pixel = 2;
        break;
    case MODE_PALETTE:
        frame_bytes_per_pixel = 1;
        line_bytes_per_pixel = 4;
        break;
    case MODE_RGB888:
        frame_bytes_per_pixel = 4;
        line_bytes_per_pixel = 4;
        break;
    default:
        printf("Unsupported mode %d", (int)mode);
        return false;
    }

    frame_buffer_display = (uint8_t*)malloc(frame_width * frame_height * frame_bytes_per_pixel);
    memset(frame_buffer_display, 0, frame_width * frame_height * frame_bytes_per_pixel);

    if(double_buffer) {
        frame_buffer_back = (uint8_t*)malloc(frame_width * frame_height * frame_bytes_per_pixel);
        memset(frame_buffer_back, 0, frame_width * frame_height * frame_bytes_per_pixel);
    } else
        frame_buffer_back = frame_buffer_display;

    memset(palette, 0, sizeof(palette));

    frame_buffer_display = frame_buffer_display;

    const int frame_pixel_words = (frame_width * h_repeat * line_bytes_per_pixel + 3) >> 2;
    const int frame_line_words = frame_pixel_words + count_of(vactive_line_header);
    const int frame_lines = (v_repeat == 1) ? NUM_CHANS : NUM_FRAME_LINES;
    line_buffers = (uint32_t*)malloc(frame_line_words * 4 * frame_lines);

    for (int i = 0; i < frame_lines; ++i)
        memcpy(&line_buffers[i * frame_line_words], vactive_line_header, count_of(vactive_line_header) * sizeof(uint32_t));

    switch (mode) {
    case MODE_RGB565:
        // Configure HSTX's TMDS encoder for RGB565
        hstx_ctrl_hw->expand_tmds =
            4  << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
            8 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB   |
            5  << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
            3  << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB   |
            4  << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
            29 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

        // Pixels (TMDS) come in 2 16-bit chunks. Control symbols (RAW) are an
        // entire 32-bit word.
        hstx_ctrl_hw->expand_shift =
            2 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
            16 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
            1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;
        break;

    case MODE_PALETTE:
        // Configure HSTX's TMDS encoder for RGB888
        hstx_ctrl_hw->expand_tmds =
            7  << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
            16 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB   |
            7  << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
            8  << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB   |
            7  << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
            0  << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

        // Pixels and control symbols (RAW) are an
        // entire 32-bit word.
        hstx_ctrl_hw->expand_shift =
            1 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
            1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;
        break;

    default:
        printf("Unsupported mode %d", (int)mode);
        return false;
    }

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

    // Assign clock pair to two neighbouring pins:
    hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
    for (uint lane = 0; lane < 3; ++lane) {
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

    for (int i = 12; i <= 19; ++i) {
        gpio_set_function(i, GPIO_FUNC_HSTX);
        gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_4MA);
    }

    // Always use the bottom channels
    dma_claim_mask((1 << NUM_CHANS) - 1);

    // The channels are set up identically, to transfer a whole scanline and
    // then chain to the next channel. Each time a channel finishes, we
    // reconfigure the one that just finished, meanwhile the other channel(s)
    // are already making progress.
    // Using just 2 channels was insufficient to avoid issues with the IRQ.
    dma_channel_config c;
    c = dma_channel_get_default_config(0);
    channel_config_set_chain_to(&c, 1);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
        0,
        &c,
        &hstx_fifo_hw->fifo,
        vblank_line_vsync_off,
        count_of(vblank_line_vsync_off),
        false
    );
    c = dma_channel_get_default_config(1);
    channel_config_set_chain_to(&c, 2);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
        1,
        &c,
        &hstx_fifo_hw->fifo,
        vblank_line_vsync_off,
        count_of(vblank_line_vsync_off),
        false
    );
    for (int i = 2; i < NUM_CHANS; ++i) {
        c = dma_channel_get_default_config(i);
        channel_config_set_chain_to(&c, (i+1) % NUM_CHANS);
        channel_config_set_dreq(&c, DREQ_HSTX);
        dma_channel_configure(
            i,
            &c,
            &hstx_fifo_hw->fifo,
            vblank_line_vsync_off,
            count_of(vblank_line_vsync_off),
            false
        );
    }

    dma_hw->ints0 = (1 << NUM_CHANS) - 1;
    dma_hw->inte0 = (1 << NUM_CHANS) - 1;
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(0);

    for (int i = 0; i < frame_height; ++i) {
        memset(&frame_buffer_display[i * frame_width * frame_bytes_per_pixel], i, frame_width * frame_bytes_per_pixel);
    }

    return true;
}

void DVHSTX::flip_blocking() {
    wait_for_vsync();
    flip_now();
}

void DVHSTX::flip_now() {
    std::swap(frame_buffer_display, frame_buffer_back);
}

void DVHSTX::wait_for_vsync() {
    while (v_scanline >= timing_mode->v_front_porch) __wfe();
}

void DVHSTX::flip_async() {
    flip_next = true;
}

void DVHSTX::wait_for_flip() {
    while (flip_next) __wfe();
}