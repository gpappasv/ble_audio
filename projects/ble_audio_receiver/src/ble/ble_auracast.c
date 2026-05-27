// --- includes ----------------------------------------------------------------
#include "ble_auracast.h"
#include "ble_bap_unicast_server.h"
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

// --- Defines -----------------------------------------------------------------
#define BROADCAST_SNK_STREAM_COUNT 2

#ifndef CONFIG_BT_BAP_BASS_MAX_SUBGROUPS
#define CONFIG_BT_BAP_BASS_MAX_SUBGROUPS 1
#endif
#define PA_SYNC_SKIP                                5U
#define PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO           5U
#define BT_GAP_MS_TO_PER_ADV_SYNC_TIMEOUT(_timeout) ((uint16_t)((_timeout) / 10U))
#define BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(_timeout) (BT_GAP_MS_TO_PER_ADV_SYNC_TIMEOUT((_timeout) / USEC_PER_MSEC))
#define BT_GAP_PER_ADV_INTERVAL_TO_US(_interval)    ((uint32_t)((_interval) * 1250U))

// --- static variables definitions --------------------------------------------
static uint32_t                      target_broadcast_id;
static uint32_t                      pending_bis_bitmask;
static bool                          is_radio_syncable = false; // Tracks if BIGInfo arrived
static uint8_t                       current_src_id;
static struct bt_le_per_adv_sync    *pa_sync_instance;
static struct bt_bap_broadcast_sink *bcast_sink;

extern struct bt_bap_stream *ble_bap_unicast_server_fetch_streams(void);

static void
execute_sink_sync(uint32_t phone_mask)
{
    if (bcast_sink == NULL)
    {
        LOG_ERR("Attempted to sync BIS channels but Sink instance is NULL!");
        return;
    }

    /* 1. Translate Phone's 0-based BASS mask to Zephyr's 1-based API mask */
    uint32_t zephyr_api_mask = 0;
    for (int i = 0; i < 32; i++)
    {
        if (phone_mask & BIT(i))
        {
            zephyr_api_mask |= BIT(i + 1);
        }
    }

    LOG_INF("Executing Audio Sync | Phone Mask: 0x%08X -> Zephyr Mask: 0x%08X", phone_mask, zephyr_api_mask);

    /* 2. Fetch the base pointer to the stream array from the audio module */
    struct bt_bap_stream *raw_array_base = ble_bap_unicast_server_fetch_streams();

    /* 3. Build the local array of pointers required by the Zephyr API signature */
    struct bt_bap_stream *streams_to_sync[BROADCAST_SNK_STREAM_COUNT] = { NULL };
    for (int i = 0; i < BROADCAST_SNK_STREAM_COUNT; i++)
    {
        streams_to_sync[i] = &raw_array_base[i];
    }

    /* 4. Command the controller to open the high-speed audio valves */
    int err = bt_bap_broadcast_sink_sync(bcast_sink, zephyr_api_mask, streams_to_sync, NULL);
    if (err != 0)
    {
        LOG_ERR("BIG synchronization failed (err %d)", err);
    }
}

// --- Scan Delegator Callbacks ------------------------------------------------
static int
pa_sync_req_cb(struct bt_conn                                *conn,
               const struct bt_bap_scan_delegator_recv_state *recv_state,
               bool                                           past_avail,
               uint16_t                                       pa_interval)
{
    LOG_INF("Phone requested PA Sync (PAST available: %d)", past_avail);

    target_broadcast_id = recv_state->broadcast_id;
    current_src_id      = recv_state->src_id;
    pending_bis_bitmask = 0;
    is_radio_syncable   = false; // Reset hardware lock flag on new hunt

    /* Handle PAST (0ms latency sync) if the phone supports it */
    if (IS_ENABLED(CONFIG_BT_PER_ADV_SYNC_TRANSFER_RECEIVER) && past_avail)
    {
        struct bt_le_per_adv_sync_transfer_param past_param = { 0 };
        past_param.skip                                     = PA_SYNC_SKIP;
        past_param.timeout = (pa_interval == BT_BAP_PA_INTERVAL_UNKNOWN)
                                 ? BT_GAP_PER_ADV_MAX_TIMEOUT
                                 : CLAMP(BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(BT_GAP_PER_ADV_INTERVAL_TO_US(pa_interval))
                                             * PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO,
                                         BT_GAP_PER_ADV_MIN_TIMEOUT,
                                         BT_GAP_PER_ADV_MAX_TIMEOUT);

        int err = bt_le_per_adv_sync_transfer_subscribe(conn, &past_param);
        if (err == 0)
        {
            LOG_INF("PAST subscription active. Bypassing manual scanner.");
            bt_bap_scan_delegator_set_pa_state(recv_state->src_id, BT_BAP_PA_STATE_INFO_REQ);
            return 0;
        }
        LOG_WRN("PAST subscription failed (err %d). Falling back to manual scanner.", err);
    }

    /* Fallback: Standard blind scanning */
    struct bt_le_per_adv_sync_param sync_param = { 0 };
    bt_addr_le_copy(&sync_param.addr, &recv_state->addr);
    sync_param.options = BT_LE_PER_ADV_SYNC_OPT_NONE;
    sync_param.timeout = 1500;

    int err = bt_le_per_adv_sync_create(&sync_param, &pa_sync_instance);
    if (err != 0)
    {
        LOG_ERR("Failed scheduling radio scanner (err %d)", err);
        return err;
    }

    bt_bap_scan_delegator_set_pa_state(recv_state->src_id, BT_BAP_PA_STATE_INFO_REQ);
    return 0;
}

static int
pa_sync_term_req_cb(struct bt_conn *conn, const struct bt_bap_scan_delegator_recv_state *recv_state)
{
    LOG_INF("Phone requested PA Termination. Running teardown...");

    if (bcast_sink)
    {
        bt_bap_broadcast_sink_delete(bcast_sink);
        bcast_sink = NULL;
    }
    if (pa_sync_instance)
    {
        bt_le_per_adv_sync_delete(pa_sync_instance);
        pa_sync_instance = NULL;
    }

    uint32_t bis_clear[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS] = { 0 };
    bt_bap_scan_delegator_set_bis_sync_state(recv_state->src_id, bis_clear);
    bt_bap_scan_delegator_set_pa_state(recv_state->src_id, BT_BAP_PA_STATE_NOT_SYNCED);

    pending_bis_bitmask = 0;
    is_radio_syncable   = false; // Clear flag on termination
    return 0;
}

static int
bis_sync_req_cb(struct bt_conn                                *conn,
                const struct bt_bap_scan_delegator_recv_state *recv_state,
                const uint32_t                                 bis_sync_req[])
{
    if (recv_state->pa_sync_state == BT_BAP_PA_STATE_NOT_SYNCED)
    {
        return 0;
    }

    uint32_t bis_synced[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS];
    bis_synced[0] = bis_sync_req[0];

    if (bis_synced[0] == 0)
    {
        LOG_INF("Audio Pause Command received.");
        pending_bis_bitmask = 0;
        if (bcast_sink)
        {
            bt_bap_broadcast_sink_stop(bcast_sink);
        }
    }
    else
    {
        if (bis_synced[0] == BT_BAP_BIS_SYNC_NO_PREF)
        {
            bis_synced[0] = 0x00000003; /* Default to stereo layout */
        }

        /* Two-Key Handshake: Check if both hardware lock and sink allocation are ready */
        if (!is_radio_syncable || bcast_sink == NULL)
        {
            LOG_INF("Hardware not syncable yet. Deferring bitmask (0x%08X)...", bis_synced[0]);
            pending_bis_bitmask = bis_synced[0];
        }
        else
        {
            execute_sink_sync(bis_synced[0]);
        }
    }

    bt_bap_scan_delegator_set_bis_sync_state(recv_state->src_id, bis_synced);
    return 0;
}

static struct bt_bap_scan_delegator_cb delegator_cbs = {
    .pa_sync_req      = pa_sync_req_cb,
    .bis_sync_req     = bis_sync_req_cb,
    .pa_sync_term_req = pa_sync_term_req_cb,
};

// --- BAP Broadcast Sink Global Callbacks -------------------------------------
static void
base_recv_cb(struct bt_bap_broadcast_sink *sink, const struct bt_bap_base *base, size_t base_size)
{
    LOG_INF("BASE Blueprint packet received.");
    /* No execution here to avoid err -11 (-EAGAIN). We wait for the BIGInfo structure. */
}

static void
syncable_cb(struct bt_bap_broadcast_sink *sink, const struct bt_iso_biginfo *biginfo)
{
    LOG_INF("BIG Metadata parsed. Hardware is officially SYNCABLE! Channels available: %d", biginfo->num_bis);

    is_radio_syncable = true; /* Turn Key 2 */

    /* Execute deferred intent if the phone requested play earlier */
    if (pending_bis_bitmask != 0)
    {
        LOG_INF("Executing deferred audio sync from syncable_cb...");
        execute_sink_sync(pending_bis_bitmask);
        pending_bis_bitmask = 0;
    }
}

static struct bt_bap_broadcast_sink_cb sink_cbs = {
    .base_recv = base_recv_cb,
    .syncable  = syncable_cb,
};

static void
pa_synced_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
    LOG_INF("🎯 Radio locked onto Periodic Advertising train!");
    pa_sync_instance = sync;

    int err = bt_bap_broadcast_sink_create(sync, target_broadcast_id, &bcast_sink);
    if (err != 0)
    {
        LOG_ERR("Failed creating Broadcast Sink instance (err %d)", err);
        return;
    }

    /* Inform phone the connection is solid */
    bt_bap_scan_delegator_set_pa_state(current_src_id, BT_BAP_PA_STATE_SYNCED);
}

static void
pa_terminated_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info)
{
    LOG_INF("Link Layer PA tracking terminated (Reason: 0x%02X)", info->reason);
    pa_sync_instance = NULL;
}

static struct bt_le_per_adv_sync_cb pa_sync_callbacks = {
    .synced = pa_synced_cb,
    .term   = pa_terminated_cb,
};

// --- Public Interface --------------------------------------------------------
void
ble_auracast_start(void)
{
    bt_le_per_adv_sync_cb_register(&pa_sync_callbacks);
    bt_bap_broadcast_sink_register_cb(&sink_cbs);
    bt_bap_scan_delegator_register_cb(&delegator_cbs);
}