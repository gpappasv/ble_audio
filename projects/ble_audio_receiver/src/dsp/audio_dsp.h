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
 */
void audio_dsp_set_volume(uint8_t vcp_volume, bool muted);

/**
 * @brief Notify DSP that a new stream is starting (triggers 10ms soft fade-in
 *        and clears DC blocker history).
 */
void audio_dsp_notify_stream_start(void);

/**
 * @brief Notify DSP whether the current frame being submitted was synthesized
 *        by Packet Loss Concealment (PLC). Applies progressive burst-loss
 *        attenuation and smooth 2ms recovery ramp when valid audio returns.
 */
void audio_dsp_notify_plc(bool is_plc_frame);

/**
 * @brief Process a frame of decoded Left and Right PCM samples.
 */
void audio_dsp_process_stereo(
    const int16_t *in_left, const int16_t *in_right, int16_t *out_left, int16_t *out_right, size_t num_samples);

/**
 * @brief Generate a smooth ramp-down frame from the last output sample voltage
 *        to exact zero (0V) to prevent DAC analog pops when stopping I2S DMA.
 */
void audio_dsp_generate_fade_out_frame(int16_t *interleaved_out, size_t num_samples);

#endif /* AUDIO_DSP_H */