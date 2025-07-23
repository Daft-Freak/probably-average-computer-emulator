#include <cmath>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/time.h"

#include "Audio.h"
#include "config.h"

#include "i2s.pio.h"

#define audio_pio __CONCAT(pio, AUDIO_I2S_PIO)

#define AUDIO_SAMPLE_FREQ 44100
#define AUDIO_BUFFER_SIZE 256
#define AUDIO_NUM_BUFFERS 4

static int16_t audio_buffer[AUDIO_BUFFER_SIZE * AUDIO_NUM_BUFFERS];
static volatile bool queue_full = false;
static volatile bool underflow = true;
static int cur_read_buffer_index = 1;
static int cur_write_buffer_index = 0;

static int write_buffer_offset = 0;

static const int dma_irq = 1; // we usually use DMA_IRQ_0 for display stuff
static int dma_channel;

void update_audio();

static void __not_in_flash_func(i2s_irq_handler)() {
    if(!dma_irqn_get_channel_status(dma_irq, dma_channel))
        return;

    dma_irqn_acknowledge_channel(dma_irq, dma_channel);

    underflow = !queue_full && cur_read_buffer_index == cur_write_buffer_index;

    // setup for next buffer
    dma_channel_set_read_addr(dma_channel, audio_buffer + cur_read_buffer_index * AUDIO_BUFFER_SIZE, !underflow);

    cur_read_buffer_index = (cur_read_buffer_index + 1) % AUDIO_NUM_BUFFERS;
    queue_full = false;
}

void init_audio() {
#ifdef AUDIO_I2S_MUTE_PIN
    gpio_set_dir(AUDIO_I2S_MUTE_PIN, GPIO_OUT);
    gpio_put(AUDIO_I2S_MUTE_PIN, 1);
    gpio_set_function(AUDIO_I2S_MUTE_PIN, GPIO_FUNC_SIO);
#endif

  // setup PIO
#if AUDIO_I2S_DATA_PIN >= 32 || AUDIO_I2S_CLOCK_PIN_BASE >= 31
    // this assumes anything else using this PIO can also deal with the base
    static_assert(AUDIO_I2S_DATA_PIN >= 16 && AUDIO_I2S_CLOCK_PIN_BASE >= 16);
    pio_set_gpio_base(audio_pio, 16);
#endif

    int pio_offset = pio_add_program(audio_pio, &i2s_program);

    int pio_sm = pio_claim_unused_sm(audio_pio, true);

    pio_sm_config cfg = i2s_program_get_default_config(pio_offset);

    const int bitrate = AUDIO_SAMPLE_FREQ * 16 * 2;
    const int clkdiv = clock_get_hz(clk_sys) / float(bitrate * 2);
    sm_config_set_clkdiv(&cfg, clkdiv);

    sm_config_set_out_shift(&cfg, false, true, 32);
    sm_config_set_out_pins(&cfg, AUDIO_I2S_DATA_PIN, 1);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
    sm_config_set_sideset_pins(&cfg, AUDIO_I2S_CLOCK_PIN_BASE);

    // init pins
    pio_gpio_init(audio_pio, AUDIO_I2S_DATA_PIN);
    pio_gpio_init(audio_pio, AUDIO_I2S_CLOCK_PIN_BASE + 0); // bit clock
    pio_gpio_init(audio_pio, AUDIO_I2S_CLOCK_PIN_BASE + 1); // left/right clock

    pio_sm_set_consecutive_pindirs(audio_pio, pio_sm, AUDIO_I2S_DATA_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(audio_pio, pio_sm, AUDIO_I2S_CLOCK_PIN_BASE, 2, true);

    pio_sm_init(audio_pio, pio_sm, pio_offset, &cfg);

    // setup DMA
    dma_channel = dma_claim_unused_channel(true);

    dma_channel_config c;
    c = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq(&c, pio_get_dreq(audio_pio, pio_sm, true));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16); // free mono -> stereo!
    dma_channel_configure(
        dma_channel,
        &c,
        &audio_pio->txf[pio_sm],
        audio_buffer,
        AUDIO_BUFFER_SIZE,
        false
    );

    // enable irq
    dma_irqn_set_channel_enabled(dma_irq, dma_channel, true);
    irq_add_shared_handler(DMA_IRQ_0 + dma_irq, i2s_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0 + dma_irq, true);

    // start
    pio_sm_set_enabled(audio_pio, pio_sm, true);
}

void audio_queue_sample(int16_t sample)
{
    if(queue_full)
    {
        //uhoh, overflow
        return;
    }

    auto buffer = audio_buffer + cur_write_buffer_index * AUDIO_BUFFER_SIZE;

    buffer[write_buffer_offset++] = sample;

    // just return if we're still filling the buffer
    if(write_buffer_offset != AUDIO_BUFFER_SIZE)
        return;

    write_buffer_offset = 0;

    // prevent irq while updating queue
    auto saved = save_and_disable_interrupts();

    cur_write_buffer_index = (cur_write_buffer_index + 1) % AUDIO_NUM_BUFFERS;
    // DMA will still be reading the prev buffer
    queue_full = !underflow && (cur_read_buffer_index + AUDIO_NUM_BUFFERS - 1) % AUDIO_NUM_BUFFERS == cur_write_buffer_index;

    restore_interrupts(saved);

    // restart after underflow
    if(underflow) {
        underflow = false;
        dma_channel_start(dma_channel);
    }
}