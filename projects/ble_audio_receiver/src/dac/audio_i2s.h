#ifndef AUDIO_I2S_H_
#define AUDIO_I2S_H_

#include <stddef.h>
#include <stdint.h>

int  audio_i2s_init(uint32_t sample_rate_hz);
void audio_i2s_stop(void);
int  audio_i2s_write_frame(const int16_t *mono_pcm, size_t num_samples);

#endif /* AUDIO_I2S_H_ */