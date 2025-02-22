#ifndef BLE_BAP_UNICAST_SERVER_H
#define BLE_BAP_UNICAST_SERVER_H

// --- includes ----------------------------------------------------------------
#include <zephyr/bluetooth/audio/audio.h>

// --- defines -----------------------------------------------------------------
#define AVAILABLE_SINK_CONTEXT  (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
    BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
    BT_AUDIO_CONTEXT_TYPE_MEDIA | \
    BT_AUDIO_CONTEXT_TYPE_GAME | \
    BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
     BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
     BT_AUDIO_CONTEXT_TYPE_MEDIA | \
     BT_AUDIO_CONTEXT_TYPE_GAME)

// --- functions declarations --------------------------------------------------
void ble_bap_unicast_server_start(void);

#endif // BLE_BAP_UNICAST_SERVER_H