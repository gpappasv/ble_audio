#include "audio_i2s.h"
#include "../dsp/audio_dsp.h"
#include "../diag/audio_telemetry.h"
#include "../hw/board_leds.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <limits.h>

LOG_MODULE_REGISTER(audio_i2s, LOG_LEVEL_INF);

#if DT_NODE_EXISTS(DT_ALIAS(i2s_tx))
#define I2S_DEV_NODE DT_ALIAS(i2s_tx)
#else
#define I2S_DEV_NODE DT_NODELABEL(i2s0)
#endif

#define SAMPLE_FREQUENCY      48000U
#define SAMPLE_BIT_WIDTH      16U
#define NUM_CHANNELS          2U
#define BLOCK_SAMPLE_PAIRS    480U /* 10 ms at 48 kHz */
#define BYTES_PER_SAMPLE_PAIR (sizeof(int16_t) * NUM_CHANNELS)
#define BLOCK_SIZE_BYTES      (BLOCK_SAMPLE_PAIRS * BYTES_PER_SAMPLE_PAIR)

/* 20 total DMA blocks (200 ms absorption headroom) */
#define NUM_DMA_BLOCKS        20U
#define PREBUFFER_BLOCKS      8U
#define TARGET_DMA_BLOCKS     8U
#define IDLE_CLOCK_GATE_MS    3000U

K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, BLOCK_SIZE_BYTES, NUM_DMA_BLOCKS, 4);

static const struct device *const i2s_dev = DEVICE_DT_GET(I2S_DEV_NODE);

static bool     i2s_configured    = false;
static bool     i2s_streaming     = false;
static bool     i2s_clock_gated   = false;
static uint32_t prebuf_cnt        = 0;

/* Filtered queue depth (Q8) */
static int32_t  filtered_slabs_q8 = (int32_t)(TARGET_DMA_BLOCKS << 8);

/* Staging accumulator for sample-accurate drift compensation & block packing */
static int16_t  staging_block[BLOCK_SAMPLE_PAIRS * NUM_CHANNELS];
static size_t   staging_pair_idx  = 0;

/* Intermediate DSP output buffers */
static int16_t  dsp_left[BLOCK_SAMPLE_PAIRS];
static int16_t  dsp_right[BLOCK_SAMPLE_PAIRS];

static struct k_work_delayable idle_gate_work;
static K_MUTEX_DEFINE(i2s_lock);

/**
 * @brief Scan an acoustic sub-window to locate the sample index with minimum slope
 *        (|dL| + |dR|) and minimum amplitude (zero crossing).
 */
static size_t
find_zero_crossing_in_range(const int16_t *l, const int16_t *r, size_t start, size_t end)
{
    if (end <= start + 2U)
    {
        return (start + end) / 2U;
    }

    size_t  best_idx = (start + end) / 2U;
    int32_t min_cost = INT32_MAX;

    for (size_t i = start + 1U; i < end; i++)
    {
        int32_t dl         = (int32_t)l[i] - (int32_t)l[i - 1U];
        int32_t dr         = (int32_t)r[i] - (int32_t)r[i - 1U];
        int32_t slope_cost = (dl < 0 ? -dl : dl) + (dr < 0 ? -dr : dr);
        int32_t amp_l      = (l[i] < 0) ? -(int32_t)l[i] : (int32_t)l[i];
        int32_t amp_r      = (r[i] < 0) ? -(int32_t)r[i] : (int32_t)r[i];
        int32_t cost       = slope_cost + ((amp_l + amp_r) >> 3);

        if (cost < min_cost)
        {
            min_cost = cost;
            best_idx = i;
            if (cost <= 2)
            {
                break; /* Exact zero-crossing found */
            }
        }
    }

    return best_idx;
}

static int
configure_i2s_hardware(void)
{
    if (i2s_clock_gated)
    {
#ifdef CONFIG_PM_DEVICE
        pm_device_action_run(i2s_dev, PM_DEVICE_ACTION_RESUME);
#endif
        i2s_clock_gated = false;
        LOG_INF("I2S peripheral resumed from low-power clock gating");
    }

    if (i2s_configured)
    {
        return 0;
    }

    struct i2s_config cfg = {
        .word_size      = SAMPLE_BIT_WIDTH,
        .channels       = NUM_CHANNELS,
        .format         = I2S_FMT_DATA_FORMAT_I2S,
        .options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = SAMPLE_FREQUENCY,
        .mem_slab       = &i2s_tx_slab,
        .block_size     = BLOCK_SIZE_BYTES,
        .timeout        = 0,
    };

    int err = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
    if (err != 0)
    {
        LOG_ERR("Failed configuring I2S TX (err %d)", err);
        return err;
    }

    i2s_configured = true;
    return 0;
}

static void
idle_gate_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&i2s_lock, K_FOREVER);
    if (!i2s_streaming && !i2s_clock_gated)
    {
        i2s_configured = false;
#ifdef CONFIG_PM_DEVICE
        if (pm_device_action_run(i2s_dev, PM_DEVICE_ACTION_SUSPEND) == 0)
        {
            i2s_clock_gated = true;
            LOG_INF("I2S clocks gated (idle standby mode)");
        }
#else
        i2s_clock_gated = true;
        LOG_INF("I2S marked idle (standby)");
#endif
    }
    k_mutex_unlock(&i2s_lock);
}

static void
submit_dma_block(const int16_t *interleaved_samples)
{
    void *mem_block = NULL;
    int err = k_mem_slab_alloc(&i2s_tx_slab, &mem_block, K_NO_WAIT);
    if (err != 0)
    {
        return;
    }

    memcpy(mem_block, interleaved_samples, BLOCK_SIZE_BYTES);

    err = i2s_write(i2s_dev, mem_block, BLOCK_SIZE_BYTES);
    if (err != 0)
    {
        k_mem_slab_free(&i2s_tx_slab, mem_block);

        if (i2s_streaming)
        {
            audio_telemetry_record_underrun();
            board_leds_notify_event(LED_EVT_ERROR);
            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
            i2s_streaming     = false;
            prebuf_cnt        = 0;
            staging_pair_idx  = 0;
            filtered_slabs_q8 = (int32_t)(TARGET_DMA_BLOCKS << 8);
        }
        return;
    }

    if (!i2s_streaming)
    {
        prebuf_cnt++;
        if (prebuf_cnt >= PREBUFFER_BLOCKS)
        {
            err = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
            if (err == 0)
            {
                i2s_streaming     = true;
                filtered_slabs_q8 = (int32_t)(PREBUFFER_BLOCKS << 8);
                audio_telemetry_set_active(true);
                LOG_INF("I2S DMA streaming started (%u frames prebuffered)", prebuf_cnt);
            }
            else
            {
                LOG_ERR("I2S_TRIGGER_START failed (err %d); resetting queue", err);
                i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
                prebuf_cnt       = 0;
                staging_pair_idx = 0;
            }
        }
    }
}

static inline void
push_stereo_pair_to_staging(int16_t l, int16_t r)
{
    staging_block[staging_pair_idx * 2U]      = l;
    staging_block[staging_pair_idx * 2U + 1U] = r;
    staging_pair_idx++;

    if (staging_pair_idx >= BLOCK_SAMPLE_PAIRS)
    {
        submit_dma_block(staging_block);
        staging_pair_idx = 0;
    }
}

int
audio_i2s_init(void)
{
    if (!device_is_ready(i2s_dev))
    {
        LOG_ERR("I2S device not ready");
        return -ENODEV;
    }

    audio_dsp_init();
    audio_telemetry_init();
    k_work_init_delayable(&idle_gate_work, idle_gate_work_handler);

    return configure_i2s_hardware();
}

void
audio_i2s_set_volume(uint8_t volume, bool muted)
{
    audio_dsp_set_volume(volume, muted);
}

void
audio_i2s_write_stereo_frame(const int16_t *left, const int16_t *right, size_t num_samples)
{
    if (left == NULL || right == NULL || num_samples == 0)
    {
        return;
    }

    size_t clamped_samples = MIN(num_samples, (size_t)BLOCK_SAMPLE_PAIRS);

    k_mutex_lock(&i2s_lock, K_FOREVER);

    k_work_cancel_delayable(&idle_gate_work);

    if (!i2s_configured)
    {
        if (configure_i2s_hardware() != 0)
        {
            k_mutex_unlock(&i2s_lock);
            return;
        }
    }

    if (!i2s_streaming && prebuf_cnt == 0 && staging_pair_idx == 0)
    {
        audio_dsp_notify_stream_start();
    }

    /* 1. Run DSP conditioning (DC blocker, PLC burst envelope, log volume, soft-knee, SSAT) */
    audio_dsp_process_stereo(left, right, dsp_left, dsp_right, clamped_samples);

    /* 2. Multi-Tier Zero-Crossing Drift Controller */
    uint32_t used_slabs = k_mem_slab_num_used_get(&i2s_tx_slab);
    uint8_t  do_pad     = 0;
    uint8_t  do_slip    = 0;

    if (i2s_streaming)
    {
        audio_telemetry_record_queue_depth(used_slabs);

        /* 16-frame exponential moving average filter on slab occupancy */
        filtered_slabs_q8 += (((int32_t)used_slabs << 8) - filtered_slabs_q8) >> 4;

        if (clamped_samples >= 240U)
        {
            if (used_slabs <= 3U)
            {
                /* Emergency starvation guard: +6250 ppm */
                do_pad = 3;
            }
            else if (used_slabs <= 5U)
            {
                /* Active recovery zone: +4166 ppm (overcomes +2392 ppm by +1774 ppm) */
                do_pad = 2;
            }
            else if (used_slabs <= 7U)
            {
                /* Soft pacing zone: dynamic 2-sample or 1-sample recovery */
                if (filtered_slabs_q8 < (int32_t)(7 << 8))
                {
                    do_pad = 2;
                }
                else
                {
                    do_pad = 1;
                }
            }
            else if (used_slabs <= 9U)
            {
                /* Target 8-9 block sweet spot: pad 1 to track +2392 ppm closely */
                do_pad = 1;
            }
            else if (used_slabs >= 15U)
            {
                do_slip = 2; /* -4166 ppm */
            }
            else if (used_slabs >= 12U)
            {
                do_slip = 1; /* -2083 ppm */
            }
            /* used_slabs == 10, 11: neutral deadband, do_pad = 0, do_slip = 0 */
        }
    }

    size_t pad_idx[3]  = { 0 };
    size_t slip_idx[2] = { 0 };

    /* Discover optimal zero-crossing splice points across separated sub-regions */
    if (do_pad == 1)
    {
        pad_idx[0] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 4U, (3U * clamped_samples) / 4U);
    }
    else if (do_pad == 2)
    {
        pad_idx[0] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 6U, clamped_samples / 2U);
        pad_idx[1] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 2U, (5U * clamped_samples) / 6U);
    }
    else if (do_pad == 3)
    {
        pad_idx[0] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 8U, (3U * clamped_samples) / 8U);
        pad_idx[1] = find_zero_crossing_in_range(dsp_left, dsp_right, (3U * clamped_samples) / 8U, (5U * clamped_samples) / 8U);
        pad_idx[2] = find_zero_crossing_in_range(dsp_left, dsp_right, (5U * clamped_samples) / 8U, (7U * clamped_samples) / 8U);
    }

    if (do_slip == 1)
    {
        slip_idx[0] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 4U, (3U * clamped_samples) / 4U);
    }
    else if (do_slip == 2)
    {
        slip_idx[0] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 6U, clamped_samples / 2U);
        slip_idx[1] = find_zero_crossing_in_range(dsp_left, dsp_right, clamped_samples / 2U, (5U * clamped_samples) / 6U);
    }

    uint8_t curr_pad  = 0;
    uint8_t curr_slip = 0;

    for (size_t i = 0; i < clamped_samples; i++)
    {
        if (curr_slip < do_slip && i == slip_idx[curr_slip])
        {
            if (i + 1U < clamped_samples)
            {
                dsp_left[i + 1U]  = (int16_t)(((int32_t)dsp_left[i] + (int32_t)dsp_left[i + 1U]) / 2);
                dsp_right[i + 1U] = (int16_t)(((int32_t)dsp_right[i] + (int32_t)dsp_right[i + 1U]) / 2);
            }
            audio_telemetry_record_drift_correction(true);
            curr_slip++;
            continue;
        }

        if (curr_pad < do_pad && i == pad_idx[curr_pad])
        {
            int16_t interp_l = (int16_t)(((int32_t)dsp_left[i - 1U] + (int32_t)dsp_left[i]) / 2);
            int16_t interp_r = (int16_t)(((int32_t)dsp_right[i - 1U] + (int32_t)dsp_right[i]) / 2);
            push_stereo_pair_to_staging(interp_l, interp_r);
            audio_telemetry_record_drift_correction(false);
            curr_pad++;
        }

        push_stereo_pair_to_staging(dsp_left[i], dsp_right[i]);
    }

    k_mutex_unlock(&i2s_lock);
}

void
audio_i2s_stop(void)
{
    k_mutex_lock(&i2s_lock, K_FOREVER);

    bool was_active   = i2s_streaming || (prebuf_cnt > 0);
    staging_pair_idx  = 0;
    prebuf_cnt        = 0;
    filtered_slabs_q8 = (int32_t)(TARGET_DMA_BLOCKS << 8);

    if (i2s_streaming)
    {
        void *fade_block = NULL;
        if (k_mem_slab_alloc(&i2s_tx_slab, &fade_block, K_NO_WAIT) == 0)
        {
            audio_dsp_generate_fade_out_frame((int16_t *)fade_block, BLOCK_SAMPLE_PAIRS);
            if (i2s_write(i2s_dev, fade_block, BLOCK_SIZE_BYTES) != 0)
            {
                k_mem_slab_free(&i2s_tx_slab, fade_block);
            }
        }

        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
        i2s_streaming = false;
    }
    else if (was_active)
    {
        i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
    }

    audio_telemetry_set_active(false);

    if (was_active)
    {
        LOG_INF("I2S TX stopped");
    }

    k_work_reschedule(&idle_gate_work, K_MSEC(IDLE_CLOCK_GATE_MS));

    k_mutex_unlock(&i2s_lock);
}