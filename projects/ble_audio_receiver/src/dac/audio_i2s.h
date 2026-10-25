#ifndef AUDIO_I2S_H_
#define AUDIO_I2S_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int  audio_i2s_init(uint32_t sample_rate_hz);
void audio_i2s_stop(void);
void audio_i2s_set_volume(uint8_t volume, bool muted);
int  audio_i2s_write_stereo_frame(const int16_t *left_pcm, const int16_t *right_pcm, size_t num_samples);

#endif /* AUDIO_I2S_H_ */