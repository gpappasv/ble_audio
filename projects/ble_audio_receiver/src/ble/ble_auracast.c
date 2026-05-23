// --- includes ----------------------------------------------------------------
#include "ble_auracast.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/logging/log.h>
#include "lc3.h"

// --- logging settings --------------------------------------------------------
LOG_MODULE_REGISTER(ble_auracast, LOG_LEVEL_INF);

// --- defines -----------------------------------------------------------------
#define MAX_SAMPLE_RATE            48000
#define MAX_FRAME_DURATION_US      10000
#define MAX_NUM_SAMPLES            ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define BROADCAST_SNK_STREAM_COUNT 2 // Local definition bypasses Kconfig cache drift

// --- static variables definitions --------------------------------------------
static struct bt_bap_broadcast_sink *bcast_sink;
static struct bt_bap_stream          broadcast_streams[BROADCAST_SNK_STREAM_COUNT];

// Caching parameters discovered over the air
static uint32_t target_broadcast_id;

// Isolated LC3 decoder instances for handling Auracast frames
static int16_t               audio_buf[BROADCAST_SNK_STREAM_COUNT][MAX_NUM_SAMPLES];
static lc3_decoder_t         lc3_decoders[BROADCAST_SNK_STREAM_COUNT];
static lc3_decoder_mem_48k_t lc3_decoder_mems[BROADCAST_SNK_STREAM_COUNT];
static int                   octets_per_frame[BROADCAST_SNK_STREAM_COUNT];

// --- forward declarations ----------------------------------------------------
static void stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf);
static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason);
static void stream_started(struct bt_bap_stream *stream);
static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf);

// --- Scan Delegator Callbacks ------------------------------------------------
static int
pa_sync_req_cb(struct bt_conn                                *conn,
               const struct bt_bap_scan_delegator_recv_state *recv_state,
               bool                                           past_avail,
               uint16_t                                       pa_interval)
{
    LOG_INF("Broadcast Assistant (Phone) requested PA Synchronization (Interval: %u). Request Approved.", pa_interval);

    int err = bt_bap_scan_delegator_set_pa_state(recv_state->src_id, BT_BAP_PA_STATE_SYNCED);
    if (err != 0)
    {
        LOG_ERR("Failed updating Scan Delegator PA state register (err %d)", err);
    }
    return 0;
}

static int
pa_sync_term_req_cb(struct bt_conn *conn, const struct bt_bap_scan_delegator_recv_state *recv_state)
{
    LOG_INF("Broadcast Assistant (Phone) requested PA Termination. Processing orderly cleanup...");

    // 1. CLEAR AUDIO STREAMS FIRST: Explicitly notify the phone that all BIS channels are dropped (set to 0)
    uint32_t bis_clear[1] = { 0 };
    int      err          = bt_bap_scan_delegator_set_bis_sync_state(recv_state->src_id, bis_clear);
    if (err != 0)
    {
        LOG_WRN("Scan Delegator BIS channels already inactive or cleared (err %d)", err);
    }

    // 2. CLEAR METADATA SECOND: Reset the PA status to NOT_SYNCED
    err = bt_bap_scan_delegator_set_pa_state(recv_state->src_id, BT_BAP_PA_STATE_NOT_SYNCED);
    if (err != 0)
    {
        LOG_ERR("Failed resetting Scan Delegator PA state register to NOT_SYNCED (err %d)", err);
    }

    return 0;
}

static int
bis_sync_req_cb(struct bt_conn                                *conn,
                const struct bt_bap_scan_delegator_recv_state *recv_state,
                const uint32_t                                 bis_sync_req[])
{
    // If the PA is already down, bypass completely as no context exists
    if (recv_state->pa_sync_state == BT_BAP_PA_STATE_NOT_SYNCED)
    {
        LOG_INF("BIS request ignored because PA metadata link is already down.");
        return 0;
    }

#ifndef CONFIG_BT_BAP_BASS_MAX_SUBGROUPS
#define CONFIG_BT_BAP_BASS_MAX_SUBGROUPS 1
#endif

    uint32_t bis_synced[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS];
    for (int i = 0; i < CONFIG_BT_BAP_BASS_MAX_SUBGROUPS; i++)
    {
        bis_synced[i] = bis_sync_req[i];
    }

    if (bis_synced[0] == 0)
    {
        LOG_INF("Broadcast Assistant requested AUDIO PAUSE (Stream bitmask cleared).");
    }
    else
    {
        LOG_INF("Broadcast Assistant requested AUDIO PLAY / SYNC (Bitmask: 0x%08X).", bis_synced[0]);
    }

    // Pass whatever the phone wants (active streams OR 0 for pause) directly to the stack
    int err = bt_bap_scan_delegator_set_bis_sync_state(recv_state->src_id, bis_synced);
    if (err != 0)
    {
        LOG_ERR("Failed updating Scan Delegator BIS state registers (err %d)", err);
    }

    return 0;
}

static struct bt_bap_scan_delegator_cb delegator_cbs = {
    .pa_sync_req      = pa_sync_req_cb,
    .bis_sync_req     = bis_sync_req_cb,
    .pa_sync_term_req = pa_sync_term_req_cb,
};

// --- Real-Time Broadcast Audio Stream Ops ------------------------------------
static struct bt_bap_stream_ops stream_ops = {
    .recv    = stream_recv,
    .stopped = stream_stopped,
    .started = stream_started,
};

static void
stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    int idx = -1;
    for (int i = 0; i < ARRAY_SIZE(broadcast_streams); i++)
    {
        if (stream == &broadcast_streams[i])
        {
            idx = i;
            break;
        }
    }

    if (idx < 0 || lc3_decoders[idx] == NULL || info == NULL || buf == NULL)
    {
        return;
    }

    if (buf->len < octets_per_frame[idx])
    {
        return;
    }

    const uint8_t *in_buf = (info->flags & BT_ISO_FLAGS_VALID) ? buf->data : NULL;
    int err = lc3_decode(lc3_decoders[idx], in_buf, octets_per_frame[idx], LC3_PCM_FORMAT_S16, audio_buf[idx], 1);
    if (err < 0)
    {
        LOG_WRN("Auracast Decoder [%d] processing failure", idx);
    }
}

static void
stream_started(struct bt_bap_stream *stream)
{
    LOG_INF("Auracast Broadcast Isochronous Stream active: %p", stream);
}

static void
stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    LOG_INF("Auracast Broadcast Isochronous Stream stopped: %p (reason 0x%02X)", stream, reason);
}

// --- BAP Broadcast Sink Global Callbacks -------------------------------------
static void
base_recv_cb(struct bt_bap_broadcast_sink *sink, const struct bt_bap_base *base, size_t base_size)
{
    LOG_INF("Broadcast Audio Source Endpoint (BASE) structural data block parsed successfully");
}

static void
sink_syncable_cb(struct bt_bap_broadcast_sink *sink, const struct bt_iso_biginfo *biginfo)
{
    int      err;
    uint32_t bis_index_bitmask = 0;
    // Create an array of pointers to satisfy the Zephyr API signature
    struct bt_bap_stream *streams_to_sync[BROADCAST_SNK_STREAM_COUNT] = { NULL };

    LOG_INF("Auracast Source Broadcast Group found! Syncable channels available: %d", biginfo->num_bis);

    for (int i = 0; i < MIN(biginfo->num_bis, ARRAY_SIZE(broadcast_streams)); i++)
    {
        bis_index_bitmask |= BIT(i + 1);

        int freq              = 48000;
        int frame_duration_us = 10000;
        octets_per_frame[i]   = 120;

        lc3_decoders[i] = lc3_setup_decoder(frame_duration_us, freq, freq, &lc3_decoder_mems[i]);
        bt_bap_stream_cb_register(&broadcast_streams[i], &stream_ops);

        // Assign the address of our static stream to the pointer array
        streams_to_sync[i] = &broadcast_streams[i];
    }

    // Pass the array of pointers (streams_to_sync) instead of the raw array of structs
    err = bt_bap_broadcast_sink_sync(sink, bis_index_bitmask, streams_to_sync, NULL);
    if (err != 0)
    {
        LOG_ERR("Failed instructing controller to sync with BIG (err %d)", err);
    }
}

static struct bt_bap_broadcast_sink_cb sink_cbs = {
    .base_recv = base_recv_cb,
    .syncable  = sink_syncable_cb,
};

// --- LE Scanner Callbacks ----------------------------------------------------
static void
pa_synced(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
    int err;

    LOG_INF("Periodic Advertising (PA) Synchronization locked successfully!");

    // Bind using the globally cached broadcast identification value
    err = bt_bap_broadcast_sink_create(sync, target_broadcast_id, &bcast_sink);
    if (err != 0)
    {
        LOG_ERR("Failed mapping PA tracking node to Broadcast Sink (err %d)", err);
    }
}

static struct bt_le_per_adv_sync_cb pa_sync_callbacks = {
    .synced = pa_synced,
};

static bool
scan_parse_report(struct bt_data *data, void *user_data)
{
    const struct bt_le_scan_recv_info *info = user_data;
    int                                err;

    // Look for Broadcast Audio Announcement Service (UUID 0x1852)
    if (data->type == BT_DATA_SVC_DATA16)
    {
        uint16_t service_uuid = data->data[0] | (data->data[1] << 8);
        if (data->data_len >= 5 && service_uuid == BT_UUID_BROADCAST_AUDIO_VAL)
        {

            // Extract the 24-bit Broadcast ID payload safely via bit-shifting
            target_broadcast_id = data->data[2] | (data->data[3] << 8) | (data->data[4] << 16);

            LOG_INF("Found an active Auracast Transmitter (ID: 0x%06X)! Syncing...", target_broadcast_id);

            bt_le_scan_stop();

            struct bt_le_per_adv_sync_param sync_param = {
                .skip    = 0,
                .timeout = 1000,
            };
            bt_addr_le_copy(&sync_param.addr, info->addr);
            sync_param.options = BT_LE_PER_ADV_SYNC_OPT_NONE;

            struct bt_le_per_adv_sync *pa_sync_instance = NULL;
            err                                         = bt_le_per_adv_sync_create(&sync_param, &pa_sync_instance);
            if (err != 0)
            {
                LOG_ERR("Failed creating periodic tracker (err %d)", err);
                bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
            }
            return false;
        }
    }
    return true;
}

static void
scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
    if (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV)
    {
        bt_data_parse(buf, scan_parse_report, (void *)info);
    }
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

// --- Public Interface --------------------------------------------------------
void
ble_auracast_start(void)
{
    int err;

    // Register our application core hook endpoints cleanly
    bt_bap_broadcast_sink_register_cb(&sink_cbs);
    bt_le_per_adv_sync_cb_register(&pa_sync_callbacks);
    bt_le_scan_cb_register(&scan_callbacks);

    bt_bap_scan_delegator_register_cb(&delegator_cbs);

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
    if (err != 0)
    {
        LOG_ERR("Failed initializing passive Auracast scan loop (err %d)", err);
        return;
    }

    LOG_INF("Auracast background scanner launched successfully");
}