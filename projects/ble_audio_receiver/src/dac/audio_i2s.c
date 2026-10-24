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

/* i2s_nrfx internal tx_queue holds 4 blocks before START.
 * Prebuffering 3 blocks leaves headroom for jitter bursts while giving 30 ms reserve. */
#define PREBUFFER_FRAMES     3
#define LOW_WATERMARK_BLOCKS 2

K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static const struct device * const i2s_dev          = DEVICE_DT_GET(I2S_NODE);
static bool                        tx_running       = false;
static int                         buffered_count   = 0;
static int16_t                     last_mono_sample = 0;
static int16_t                     last_frame_stereo[SAMPLES_PER_FRAME_MAX * CHANNELS];

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
        .timeout        = 15, /* Safe in dedicated worker thread; prevents -EAGAIN drops */
    };

    int err = i2s_configure(i2s_dev, I2S_DIR_TX, &config);
    if (err < 0)
    {
        LOG_ERR("I2S configure error: %d", err);
        return err;
    }

    tx_running       = false;
    buffered_count   = 0;
    last_mono_sample = 0;
    memset(last_frame_stereo, 0, sizeof(last_frame_stereo));

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
    else
    {
        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    }

    buffered_count   = 0;
    last_mono_sample = 0;
    memset(last_frame_stereo, 0, sizeof(last_frame_stereo));
    LOG_INF("I2S TX stopped");
}

static void
build_pad_frame(int16_t *dst, int16_t start_sample, int16_t end_sample, size_t num_samples)
{
    /* Smoothly ramp from start_sample to end_sample to avoid step discontinuities */
    for (size_t i = 0; i < num_samples; i++)
    {
        int32_t interp = start_sample + (((int32_t)(end_sample - start_sample) * (int32_t)i) / (int32_t)num_samples);
        dst[2 * i]     = (int16_t)interp;
        dst[2 * i + 1] = (int16_t)interp;
    }
}

int
audio_i2s_write_frame(const int16_t *mono_pcm, size_t num_samples)
{
    if (!mono_pcm || num_samples == 0 || num_samples > SAMPLES_PER_FRAME_MAX)
    {
        return -EINVAL;
    }

    size_t payload_bytes = num_samples * CHANNELS * BYTES_PER_SAMPLE;

    /* Low-Watermark Drift Protection:
     * If EasyDMA is down to <= 2 blocks in flight while running, inject a smooth
     * bridge frame before queuing the new frame so DMA never hits empty. */
    if (tx_running && k_mem_slab_num_used_get(&i2s_mem_slab) <= LOW_WATERMARK_BLOCKS)
    {
        void *pad_buf = NULL;
        if (k_mem_slab_alloc(&i2s_mem_slab, &pad_buf, K_NO_WAIT) == 0)
        {
            build_pad_frame((int16_t *)pad_buf, last_mono_sample, mono_pcm[0], num_samples);
            if (i2s_write(i2s_dev, pad_buf, payload_bytes) != 0)
            {
                k_mem_slab_free(&i2s_mem_slab, pad_buf);
            }
        }
    }

    void *tx_buf = NULL;
    int   err    = k_mem_slab_alloc(&i2s_mem_slab, &tx_buf, K_MSEC(15));
    if (err)
    {
        return -ENOBUFS;
    }

    int16_t *out = (int16_t *)tx_buf;
    for (size_t i = 0; i < num_samples; i++)
    {
        out[2 * i]     = mono_pcm[i];
        out[2 * i + 1] = mono_pcm[i];
    }

    last_mono_sample = mono_pcm[num_samples - 1];
    memcpy(last_frame_stereo, out, payload_bytes);

    err = i2s_write(i2s_dev, tx_buf, payload_bytes);

    /* Instantaneous recovery if State 4 (Error) ever occurs */
    if (err == -EIO)
    {
        LOG_WRN("I2S underrun detected, performing instant restart");
        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
        tx_running     = false;
        buffered_count = 0;

        /* Pre-load one cached block + current block and restart immediately */
        void *lookahead_buf = NULL;
        if (k_mem_slab_alloc(&i2s_mem_slab, &lookahead_buf, K_NO_WAIT) == 0)
        {
            memcpy(lookahead_buf, last_frame_stereo, payload_bytes);
            if (i2s_write(i2s_dev, lookahead_buf, payload_bytes) == 0)
            {
                buffered_count++;
            }
            else
            {
                k_mem_slab_free(&i2s_mem_slab, lookahead_buf);
            }
        }

        err = i2s_write(i2s_dev, tx_buf, payload_bytes);
        if (err == 0)
        {
            buffered_count++;
            if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START) == 0)
            {
                tx_running = true;
            }
        }
        else
        {
            k_mem_slab_free(&i2s_mem_slab, tx_buf);
            return err;
        }
        return 0;
    }

    if (err < 0)
    {
        k_mem_slab_free(&i2s_mem_slab, tx_buf);
        return err;
    }

    /* Trigger DMA engine once prebuffer depth is satisfied */
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