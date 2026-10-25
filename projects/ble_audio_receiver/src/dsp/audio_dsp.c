#include "audio_dsp.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>

/* 10 ms fade-in duration at 48 kHz = 480 samples -> step = 32767 / 480 = 68 */
#define FADE_IN_STEP_Q15    68
#define UNITY_GAIN_Q15      32767
#define SOFT_KNEE_THRESHOLD 28672 /* ~ -1.15 dBFS */

struct dc_blocker_state
{
    int32_t prev_x;
    int32_t acc_q8;
};

/* 17-point logarithmic volume lookup table (0..256 in steps of 16)
 * Maps VCP 0..255 across a natural 56 dB psychoacoustic dynamic range in Q15. */
static const uint16_t vcp_log_lut_q15[17] = {
    0,     /*   0: Mute (-inf dB) */
    52,    /*  16: -56.0 dB */
    104,   /*  32: -50.0 dB */
    184,   /*  48: -45.0 dB */
    328,   /*  64: -40.0 dB */
    519,   /*  80: -36.0 dB */
    823,   /*  96: -32.0 dB */
    1304,  /* 112: -28.0 dB */
    1951,  /* 128: -24.5 dB */
    2920,  /* 144: -21.0 dB */
    4369,  /* 160: -17.5 dB */
    6538,  /* 176: -14.0 dB */
    9235,  /* 192: -11.0 dB */
    13045, /* 208:  -8.0 dB */
    18427, /* 224:  -5.0 dB */
    24576, /* 240:  -2.5 dB */
    32767  /* 256:   0.0 dB */
};

static struct dc_blocker_state dc_left;
static struct dc_blocker_state dc_right;

static int32_t target_gain_q23  = (24576 << 8);
static int32_t current_gain_q23 = (24576 << 8);
static int32_t fade_env_q15     = 0;

static int16_t last_sample_l = 0;
static int16_t last_sample_r = 0;

static inline int16_t
sat16(int32_t x)
{
#if defined(__ARM_FEATURE_SAT)
    int32_t res;
    __asm__ volatile("ssat %0, %1, %2" : "=r"(res) : "I"(16), "r"(x));
    return (int16_t)res;
#else
    if (x > 32767)
    {
        return 32767;
    }
    if (x < -32768)
    {
        return -32768;
    }
    return (int16_t)x;
#endif
}

static inline int32_t
apply_dc_blocker(struct dc_blocker_state *st, int16_t x)
{
    /* 1st-order high-pass filter: y[n] = x[n] - x[n-1] + (1 - 2^-9) * y[n-1]
     * Cutoff ~14.9 Hz at 48 kHz. State kept in Q8 to prevent quantization limit cycles. */
    int32_t diff_q8 = ((int32_t)x - st->prev_x) << 8;
    st->prev_x      = (int32_t)x;
    st->acc_q8      = diff_q8 + st->acc_q8 - (st->acc_q8 >> 9);
    return (st->acc_q8 >> 8);
}

static inline int16_t
apply_soft_knee_and_sat(int32_t sample)
{
    if (sample > SOFT_KNEE_THRESHOLD)
    {
        sample = SOFT_KNEE_THRESHOLD + ((sample - SOFT_KNEE_THRESHOLD) >> 1);
    }
    else if (sample < -SOFT_KNEE_THRESHOLD)
    {
        sample = -SOFT_KNEE_THRESHOLD + ((sample + SOFT_KNEE_THRESHOLD) >> 1);
    }
    return sat16(sample);
}

static uint16_t
vcp_volume_to_q15(uint8_t vcp_vol, bool muted)
{
    if (muted || vcp_vol == 0U)
    {
        return 0U;
    }
    if (vcp_vol == 255U)
    {
        return UNITY_GAIN_Q15;
    }

    uint32_t idx  = vcp_vol >> 4;    /* 0..15 */
    uint32_t frac = vcp_vol & 0x0FU; /* 0..15 */

    uint32_t y0 = vcp_log_lut_q15[idx];
    uint32_t y1 = vcp_log_lut_q15[idx + 1U];

    return (uint16_t)(y0 + (((y1 - y0) * frac) >> 4));
}

void
audio_dsp_init(void)
{
    memset(&dc_left, 0, sizeof(dc_left));
    memset(&dc_right, 0, sizeof(dc_right));
    fade_env_q15  = 0;
    last_sample_l = 0;
    last_sample_r = 0;
}

void
audio_dsp_set_volume(uint8_t vcp_volume, bool muted)
{
    uint16_t gain_q15 = vcp_volume_to_q15(vcp_volume, muted);
    target_gain_q23   = ((int32_t)gain_q15) << 8;
}

void
audio_dsp_notify_stream_start(void)
{
    memset(&dc_left, 0, sizeof(dc_left));
    memset(&dc_right, 0, sizeof(dc_right));
    fade_env_q15     = 0;
    current_gain_q23 = target_gain_q23;
    last_sample_l    = 0;
    last_sample_r    = 0;
}

void
audio_dsp_process_stereo(
    const int16_t *in_left, const int16_t *in_right, int16_t *out_left, int16_t *out_right, size_t num_samples)
{
    for (size_t i = 0; i < num_samples; i++)
    {
        /* 1. Per-sample volume slew (de-zippering, tau ~ 2.67 ms) */
        current_gain_q23 += (target_gain_q23 - current_gain_q23) >> 7;
        int32_t vol_q15 = current_gain_q23 >> 8;

        /* 2. Soft 10 ms stream startup fade-in envelope */
        if (fade_env_q15 < UNITY_GAIN_Q15)
        {
            fade_env_q15 += FADE_IN_STEP_Q15;
            if (fade_env_q15 > UNITY_GAIN_Q15)
            {
                fade_env_q15 = UNITY_GAIN_Q15;
            }
        }

        int32_t combined_gain_q15 = (vol_q15 * fade_env_q15) >> 15;

        /* 3. Subsonic DC Blocker */
        int32_t clean_l = apply_dc_blocker(&dc_left, in_left[i]);
        int32_t clean_r = apply_dc_blocker(&dc_right, in_right[i]);

        /* 4. Gain scaling + Soft-knee limiter + Hardware SSAT */
        int32_t scaled_l = (clean_l * combined_gain_q15) >> 15;
        int32_t scaled_r = (clean_r * combined_gain_q15) >> 15;

        out_left[i]  = apply_soft_knee_and_sat(scaled_l);
        out_right[i] = apply_soft_knee_and_sat(scaled_r);
    }

    if (num_samples > 0)
    {
        last_sample_l = out_left[num_samples - 1];
        last_sample_r = out_right[num_samples - 1];
    }
}

void
audio_dsp_generate_fade_out_frame(int16_t *interleaved_out, size_t num_samples)
{
    if (interleaved_out == NULL || num_samples == 0)
    {
        return;
    }

    int32_t start_l = last_sample_l;
    int32_t start_r = last_sample_r;
    int32_t n       = (int32_t)num_samples;

    for (int32_t i = 0; i < n; i++)
    {
        int32_t rem                = (n - 1) - i;
        interleaved_out[i * 2]     = (int16_t)((start_l * rem) / n);
        interleaved_out[i * 2 + 1] = (int16_t)((start_r * rem) / n);
    }

    last_sample_l = 0;
    last_sample_r = 0;
    fade_env_q15  = 0;
}