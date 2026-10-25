#ifndef AUDIO_TELEMETRY_H
#define AUDIO_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize lockless telemetry counters and periodic reporter work item.
 */
void audio_telemetry_init(void);

/**
 * @brief Start or stop the 30-second periodic telemetry log output.
 */
void audio_telemetry_set_active(bool streaming_active);

/**
 * @brief Record an incoming ISO SDU packet event.
 */
void audio_telemetry_record_sdu(bool valid_data, bool is_plc, uint16_t seq_gap);

/**
 * @brief Record instantaneous I2S DMA slab queue occupancy (in blocks).
 */
void audio_telemetry_record_queue_depth(uint32_t used_blocks);

/**
 * @brief Record an I2S DMA underrun or restart event.
 */
void audio_telemetry_record_underrun(void);

/**
 * @brief Record a 1-sample ASRC-lite clock drift correction (+1 pad or -1 slip).
 */
void audio_telemetry_record_drift_correction(bool is_slip);

#endif /* AUDIO_TELEMETRY_H */