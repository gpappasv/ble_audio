#ifndef AUDIO_I2S_H
#define AUDIO_I2S_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Initialize the I2S peripheral, DSP subsystem, and telemetry engine.
 */
int audio_i2s_init(void);

/**
 * @brief Process and queue a stereo PCM frame for DMA output with automatic
 *        ASRC-lite 1-sample drift compensation.
 */
void audio_i2s_write_stereo_frame(const int16_t *left, const int16_t *right, size_t num_samples);

/**
 * @brief Smoothly ramp output to 0V, stop I2S DMA, and schedule idle clock gating.
 */
void audio_i2s_stop(void);

/**
 * @brief Update digital volume and mute state via logarithmic DSP curve.
 */
void audio_i2s_set_volume(uint8_t volume, bool muted);

#endif /* AUDIO_I2S_H */