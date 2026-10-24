#include "audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(audio_i2s, LOG_LEVEL_INF);

#define I2S_NODE DT_ALIAS(i2s_tx)

#define SAMPLES_PER_FRAME_MAX 480
#define CHANNELS              2
#define BYTES_PER_SAMPLE      sizeof(int16_t)
#define BLOCK_SIZE            (SAMPLES_PER_FRAME_MAX * CHANNELS * BYTES_PER_SAMPLE)
#define BLOCK_COUNT           12
#define PREBUFFER_FRAMES      4

K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static const struct device * const i2s_dev        = DEVICE_DT_GET(I2S_NODE);
static bool                        tx_running     = false;
static int                         buffered_count = 0;

int
audio_i2s_init(uint32_t sample_rate_hz)
{
    if (!device_is_ready(i2s_dev))
    {
        LOG_ERR("I2S device %s not ready", i2s_dev->name);
        return -ENODEV;
    }

    struct i2s_config config = {
        .word_size      = 16,
        .channels       = 2,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
        .frame_clk_freq = sample_rate_hz,
        .mem_slab       = &i2s_mem_slab,
        .block_size     = BLOCK_SIZE,
        .timeout        = 0, // CRITICAL: Non-blocking (K_NO_WAIT) to protect BLE RX thread
    };

    int err = i2s_configure(i2s_dev, I2S_DIR_TX, &config);
    if (err < 0)
    {
        LOG_ERR("I2S configure error: %d", err);
        return err;
    }

    tx_running     = false;
    buffered_count = 0;
    LOG_INF("I2S initialized in READY state (%u Hz)", sample_rate_hz);
    return 0;
}

void
audio_i2s_stop(void)
{
    if (tx_running)
    {
        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
        tx_running = false;
    }
    buffered_count = 0;
    LOG_INF("I2S TX stopped");
}

int
audio_i2s_write_frame(const int16_t *mono_pcm, size_t num_samples)
{
    if (!mono_pcm || num_samples > SAMPLES_PER_FRAME_MAX)
    {
        return -EINVAL;
    }

    void *tx_buf;
    int   err = k_mem_slab_alloc(&i2s_mem_slab, &tx_buf, K_NO_WAIT);
    if (err)
    {
        // Discard frame cleanly if ring pool is full to prevent ISO thread starvation
        return -ENOBUFS;
    }

    // Interleave single incoming channel into both Left and Right DAC channels
    int16_t *out = (int16_t *)tx_buf;
    for (size_t i = 0; i < num_samples; i++)
    {
        out[2 * i]     = mono_pcm[i]; // Left
        out[2 * i + 1] = mono_pcm[i]; // Right
    }

    size_t payload_bytes = num_samples * CHANNELS * BYTES_PER_SAMPLE;
    err                  = i2s_write(i2s_dev, tx_buf, payload_bytes);

    // If an underrun forced the driver into State 4 (Error), recover cleanly
    if (err == -EIO)
    {
        LOG_WRN("I2S underrun recovered, resetting state machine");
        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
        tx_running     = false;
        buffered_count = 0;
        err            = i2s_write(i2s_dev, tx_buf, payload_bytes);
    }

    if (err < 0)
    {
        k_mem_slab_free(&i2s_mem_slab, tx_buf);
        return err;
    }

    // Trigger DMA engine once prebuffer depth is satisfied
    if (!tx_running)
    {
        buffered_count++;
        if (buffered_count >= PREBUFFER_FRAMES)
        {
            err = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
            if (err == 0)
            {
                tx_running = true;
                LOG_INF("I2S DMA streaming started");
            }
        }
    }

    return 0;
}