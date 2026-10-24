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
#define BLOCK_COUNT           16

#define PREBUFFER_FRAMES    6
#define HIGH_WATERMARK_DROP 14
#define UNITY_GAIN_Q15      32768

K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static const struct device * const i2s_dev          = DEVICE_DT_GET(I2S_NODE);
static bool                        tx_running       = false;
static int                         buffered_count   = 0;
static volatile int32_t            target_gain_q15  = 20000; /* Default ~78% initial volume */
static int32_t                     current_gain_q15 = 20000;

void
audio_i2s_set_volume(uint8_t volume, bool muted)
{
    if (muted || volume == 0U)
    {
        target_gain_q15 = 0;
        return;
    }

    if (volume >= 255U)
    {
        target_gain_q15 = UNITY_GAIN_Q15;
        return;
    }

    /* Standard audio quadratic taper: G = (volume / 255)^2 * UNITY_GAIN_Q15
     * Max value at v=254: 254 * 254 * 32768 = 2,114,064,384 (cleanly fits in uint32_t).
     * Provides smooth, natural decibel progression across the entire slider with zero overflow. */
    uint32_t v    = (uint32_t)volume;
    uint32_t gain = (v * v * (uint32_t)UNITY_GAIN_Q15) / (255U * 255U);

    if (gain == 0U)
    {
        gain = 1U;
    }

    target_gain_q15 = (int32_t)gain;
}

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
        .timeout        = 20,
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
    i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    tx_running     = false;
    buffered_count = 0;
    LOG_INF("I2S TX stopped");
}

int
audio_i2s_write_stereo_frame(const int16_t *left_pcm, const int16_t *right_pcm, size_t num_samples)
{
    if (!left_pcm || !right_pcm || num_samples == 0 || num_samples > SAMPLES_PER_FRAME_MAX)
    {
        return -EINVAL;
    }

    /* High-watermark guard: prevent slab exhaustion if phone clock is slightly faster */
    uint32_t used_blocks = k_mem_slab_num_used_get(&i2s_mem_slab);
    if (tx_running && used_blocks >= HIGH_WATERMARK_DROP)
    {
        return 0;
    }

    void *tx_buf = NULL;
    int   err    = k_mem_slab_alloc(&i2s_mem_slab, &tx_buf, K_MSEC(20));
    if (err)
    {
        return -ENOBUFS;
    }

    int32_t  tgt_gain = target_gain_q15;
    int32_t  cur_gain = current_gain_q15;
    int16_t *out      = (int16_t *)tx_buf;

    for (size_t i = 0; i < num_samples; i++)
    {
        /* Smooth gain stepping per sample to eliminate zipper noise during volume adjustment */
        if (cur_gain < tgt_gain)
        {
            cur_gain += 32;
            if (cur_gain > tgt_gain)
            {
                cur_gain = tgt_gain;
            }
        }
        else if (cur_gain > tgt_gain)
        {
            cur_gain -= 32;
            if (cur_gain < tgt_gain)
            {
                cur_gain = tgt_gain;
            }
        }

        /* Apply Q15 gain with symmetric rounding */
        int32_t l_scaled = ((int32_t)left_pcm[i] * cur_gain + 16384) >> 15;
        int32_t r_scaled = ((int32_t)right_pcm[i] * cur_gain + 16384) >> 15;

        out[2 * i]     = (int16_t)l_scaled; /* Left channel  */
        out[2 * i + 1] = (int16_t)r_scaled; /* Right channel */
    }

    current_gain_q15 = cur_gain;

    size_t payload_bytes = num_samples * CHANNELS * BYTES_PER_SAMPLE;
    err                  = i2s_write(i2s_dev, tx_buf, payload_bytes);

    if (err == -EIO)
    {
        LOG_WRN("I2S queue starved; re-buffering %d frames", PREBUFFER_FRAMES);
        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
        tx_running     = false;
        buffered_count = 0;

        err = i2s_write(i2s_dev, tx_buf, payload_bytes);
    }

    if (err < 0)
    {
        k_mem_slab_free(&i2s_mem_slab, tx_buf);
        return err;
    }

    if (!tx_running)
    {
        buffered_count++;
        if (buffered_count >= PREBUFFER_FRAMES)
        {
            err = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
            if (err == 0)
            {
                tx_running = true;
                LOG_INF("I2S DMA streaming started (%d frames prebuffered)", buffered_count);
            }
        }
    }

    return 0;
}