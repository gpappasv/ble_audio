#ifndef AUDIO_DSP_H
#define AUDIO_DSP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Initialize DSP state, filters, and volume envelope.
 */
void audio_dsp_init(void);

/**
 * @brief Set target VCP volume (0..255) and mute state.
 *        Mapped through a logarithmic dB curve and slewed per-sample.
 */
void audio_dsp_set_volume(uint8_t vcp_volume, bool muted);

/**
 * @brief Notify DSP that a new stream is starting (triggers 10ms soft fade-in
 *        and clears DC blocker history).
 */
void audio_dsp_notify_stream_start(void);

/**
 * @brief Process a frame of decoded Left and Right PCM samples in-place or to output buffers.
 *        Applies 15Hz DC blocking, per-sample slewed log volume, soft-knee limiting, and SSAT.
 */
void audio_dsp_process_stereo(
    const int16_t *in_left, const int16_t *in_right, int16_t *out_left, int16_t *out_right, size_t num_samples);

/**
 * @brief Generate a smooth ramp-down frame from the last output sample voltage
 *        to exact zero (0V) to prevent DAC analog pops when stopping I2S DMA.
 *
 * @param interleaved_out Pointer to stereo interleaved buffer (size = num_samples * 2)
 * @param num_samples     Number of stereo sample pairs to ramp over (e.g. 480)
 */
void audio_dsp_generate_fade_out_frame(int16_t *interleaved_out, size_t num_samples);

#endif /* AUDIO_DSP_H */