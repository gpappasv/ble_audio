#ifndef BLE_AUDIO_STREAM_H
#define BLE_AUDIO_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/conn.h>

#define BLE_AUDIO_MAX_UNICAST_STREAMS   (CONFIG_BT_MAX_CONN * CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT)
#define BLE_AUDIO_MAX_BROADCAST_STREAMS CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT

void                  ble_audio_stream_init(void);
struct bt_bap_stream *ble_audio_stream_alloc_unicast(void);
void                  ble_audio_stream_get_broadcast_streams(struct bt_bap_stream **streams, size_t max_streams);
int  ble_audio_stream_configure(struct bt_bap_stream *stream, const struct bt_audio_codec_cfg *codec_cfg);
void ble_audio_stream_cleanup(struct bt_bap_stream *stream);
void ble_audio_stream_evict_conn(struct bt_conn *conn);
void ble_audio_stream_reset_broadcast(void);
bool ble_audio_stream_has_active_unicast(void);
bool ble_audio_stream_has_active_broadcast(void);

#endif // BLE_AUDIO_STREAM_H