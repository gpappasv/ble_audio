#include "ble_bap_broadcast_sink.h"
#include "ble_audio_stream.h"
#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_broadcast_sink, LOG_LEVEL_INF);

#define INVALID_BROADCAST_ID   0xFFFFFFFFU
#define PA_SYNC_TIMEOUT_MS     8000
#define PAST_FALLBACK_DELAY_MS 1200

static struct bt_bap_broadcast_sink                  *broadcast_sink = NULL;
static struct bt_le_per_adv_sync                     *pa_sync        = NULL;
static const struct bt_bap_scan_delegator_recv_state *req_recv_state = NULL;

static bt_addr_le_t target_addr;
static uint8_t      target_sid          = 0;
static uint16_t     target_interval     = 0;
static uint32_t     target_broadcast_id = INVALID_BROADCAST_ID;

/* BIS bitfields: what the broadcaster offers (BASE) vs what the phone requested (BASS) */
static uint32_t base_bis_bitfield  = 0;
static uint32_t requested_bis_sync = BT_BAP_BIS_SYNC_NO_PREF;
static size_t   base_bis_count     = 0;

/* Broadcast encryption key supplied by the phone via BASS */
static uint8_t stored_broadcast_code[BT_ISO_BROADCAST_CODE_SIZE];
static bool    has_broadcast_code = false;
static bool    sink_encrypted     = false;

static bool base_received        = false;
static bool sink_syncable        = false;
static bool big_synced           = false;
static bool big_stopping         = false;
static bool pending_full_reset   = false;
static bool scanning_for_pa      = false;
static bool suspended_by_unicast = false;

static struct k_work_delayable pa_create_work;
static struct k_work           big_sync_work;
static struct k_work           big_stopped_work;
static struct k_work           cleanup_work;
static struct k_work_delayable watchdog_work;

static uint16_t
interval_to_sync_timeout(uint16_t pa_interval)
{
    if (pa_interval == 0U || pa_interval == BT_BAP_PA_INTERVAL_UNKNOWN)
    {
        return 400U; /* 4.0s default timeout when interval is unknown */
    }
    uint32_t interval_ms = ((uint32_t)pa_interval * 5U) / 4U;
    return (uint16_t)CLAMP(interval_ms, 100U, BT_GAP_PER_ADV_MAX_TIMEOUT);
}

static void
stop_pa_scan_if_active(void)
{
    if (scanning_for_pa)
    {
        bt_le_scan_stop();
        scanning_for_pa = false;
    }
}

static void
teardown_pa_and_sink(void)
{
    k_work_cancel_delayable(&pa_create_work);
    k_work_cancel_delayable(&watchdog_work);

    stop_pa_scan_if_active();

    /* If BIG is currently active, stop BIG first and wait for big_stopped_work
     * so Zephyr cleanly detaches stream->ep and stream->group before sink deletion. */
    if (broadcast_sink != NULL && big_synced)
    {
        pending_full_reset = true;
        if (!big_stopping)
        {
            big_stopping = true;
            int err      = bt_bap_broadcast_sink_stop(broadcast_sink);
            if (err == 0 || err == -EALREADY)
            {
                return;
            }
            big_stopping = false;
        }
        else
        {
            return;
        }
    }

    pending_full_reset = false;
    big_stopping       = false;
    big_synced         = false;

    if (broadcast_sink != NULL)
    {
        bt_bap_broadcast_sink_delete(broadcast_sink);
        broadcast_sink = NULL;
    }

    if (pa_sync != NULL)
    {
        bt_le_per_adv_sync_delete(pa_sync);
        pa_sync = NULL;
    }

    ble_audio_stream_reset_broadcast();

    base_bis_bitfield = 0;
    base_bis_count    = 0;
    base_received     = false;
    sink_syncable     = false;
    sink_encrypted    = false;
}

/* --- Periodic Advertising Callbacks --- */

static void
pa_synced_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
    if (suspended_by_unicast)
    {
        bt_le_per_adv_sync_delete(sync);
        return;
    }

    /* PA is locked: cancel helper scan and PA watchdog */
    k_work_cancel_delayable(&pa_create_work);
    k_work_cancel_delayable(&watchdog_work);
    stop_pa_scan_if_active();

    pa_sync = sync;

    if (target_broadcast_id == INVALID_BROADCAST_ID && req_recv_state != NULL)
    {
        target_broadcast_id = req_recv_state->broadcast_id;
    }

    LOG_INF("PA synced (%s); creating Broadcast Sink for ID 0x%06X",
            (info->conn != NULL) ? "via Phone PAST" : "Direct RF",
            target_broadcast_id);

    if (broadcast_sink != NULL)
    {
        bt_bap_broadcast_sink_delete(broadcast_sink);
        broadcast_sink = NULL;
    }

    base_bis_bitfield = 0;
    base_bis_count    = 0;
    base_received     = false;
    sink_syncable     = false;
    big_synced        = false;
    big_stopping      = false;

    if (target_broadcast_id != INVALID_BROADCAST_ID)
    {
        int err = bt_bap_broadcast_sink_create(pa_sync, target_broadcast_id, &broadcast_sink);
        if (err != 0)
        {
            LOG_ERR("Failed creating Broadcast Sink (err %d)", err);
            k_work_submit(&cleanup_work);
            return;
        }
    }
}

static void
pa_term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info)
{
    if (sync != pa_sync)
    {
        return;
    }

    LOG_INF("PA sync terminated (reason 0x%02X)", info->reason);
    pa_sync = NULL;

    k_work_submit(&cleanup_work);
}

static struct bt_le_per_adv_sync_cb pa_sync_cbs = {
    .synced = pa_synced_cb,
    .term   = pa_term_cb,
};

/* --- BAP Broadcast Sink Callbacks (BASE & BIGInfo) --- */

static bool
base_bis_cb(const struct bt_bap_base_subgroup_bis *bis, void *user_data)
{
    uint32_t *bitfield = user_data;

    if (base_bis_count < BLE_AUDIO_MAX_BROADCAST_STREAMS)
    {
        *bitfield |= BT_ISO_BIS_INDEX_BIT(bis->index);
        base_bis_count++;
    }

    return true;
}

static bool
base_subgroup_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
    int ret = bt_bap_base_subgroup_foreach_bis(subgroup, base_bis_cb, user_data);
    return (ret >= 0);
}

static void
base_recv_cb(struct bt_bap_broadcast_sink *sink, const struct bt_bap_base *base, size_t base_size)
{
    ARG_UNUSED(base_size);

    if (sink != broadcast_sink || base_received || suspended_by_unicast)
    {
        return;
    }

    uint32_t bis_bitfield = 0;
    base_bis_count        = 0;

    int ret = bt_bap_base_foreach_subgroup(base, base_subgroup_cb, &bis_bitfield);
    if (ret < 0 || bis_bitfield == 0U)
    {
        return;
    }

    base_bis_bitfield = bis_bitfield;
    base_received     = true;

    if (sink_syncable && !big_synced && !big_stopping && requested_bis_sync != 0U)
    {
        k_work_submit(&big_sync_work);
    }
}

static void
syncable_cb(struct bt_bap_broadcast_sink *sink, const struct bt_iso_biginfo *biginfo)
{
    if (sink != broadcast_sink || suspended_by_unicast || big_synced || big_stopping)
    {
        return;
    }

    sink_encrypted = biginfo->encryption;
    sink_syncable  = true;

    if (base_received && requested_bis_sync != 0U)
    {
        k_work_submit(&big_sync_work);
    }
}

static struct bt_bap_broadcast_sink_cb broadcast_sink_cbs = {
    .base_recv = base_recv_cb,
    .syncable  = syncable_cb,
};

/* --- BAP Scan Delegator Callbacks (Driven by Phone Broadcast Assistant) --- */

static void
recv_state_updated_cb(struct bt_conn *conn, const struct bt_bap_scan_delegator_recv_state *recv_state)
{
    ARG_UNUSED(conn);

    if (recv_state != NULL)
    {
        req_recv_state      = recv_state;
        target_broadcast_id = recv_state->broadcast_id;
    }
}

static int
pa_sync_req_cb(struct bt_conn                                *conn,
               const struct bt_bap_scan_delegator_recv_state *recv_state,
               bool                                           past_avail,
               uint16_t                                       interval)
{
    ARG_UNUSED(conn);

    if (suspended_by_unicast || ble_audio_stream_has_active_unicast())
    {
        LOG_WRN("Rejecting BASS PA sync req: Unicast stream is active");
        return -EBUSY;
    }

    bool same_broadcaster = (target_broadcast_id == recv_state->broadcast_id)
                            && (bt_addr_le_cmp(&target_addr, &recv_state->addr) == 0)
                            && (target_sid == recv_state->adv_sid);

    req_recv_state      = recv_state;
    target_broadcast_id = recv_state->broadcast_id;
    target_sid          = recv_state->adv_sid;
    target_interval     = interval;
    bt_addr_le_copy(&target_addr, &recv_state->addr);

    uint32_t state_bis = 0U;
    for (size_t i = 0; i < recv_state->num_subgroups; i++)
    {
        state_bis |= recv_state->subgroups[i].bis_sync;
    }
    if (state_bis != 0U)
    {
        requested_bis_sync = state_bis;
    }
    else if (requested_bis_sync == 0U)
    {
        requested_bis_sync = BT_BAP_BIS_SYNC_NO_PREF;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&target_addr, addr_str, sizeof(addr_str));
    LOG_INF("Phone requested PA sync to 0x%06X (%s, SID %u, PAST: %s)",
            target_broadcast_id,
            addr_str,
            target_sid,
            past_avail ? "YES" : "NO");

    if (pa_sync != NULL)
    {
        if (same_broadcaster)
        {
            if (base_received && sink_syncable && !big_synced && !big_stopping && requested_bis_sync != 0U)
            {
                k_work_submit(&big_sync_work);
            }
            return 0;
        }

        teardown_pa_and_sink();
    }

    if (past_avail)
    {
        k_work_reschedule(&pa_create_work, K_MSEC(PAST_FALLBACK_DELAY_MS));
    }
    else
    {
        k_work_reschedule(&pa_create_work, K_NO_WAIT);
    }

    return 0;
}

static int
pa_sync_term_req_cb(struct bt_conn *conn, const struct bt_bap_scan_delegator_recv_state *recv_state)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(recv_state);

    LOG_INF("Phone requested PA sync termination");
    requested_bis_sync = 0U;
    k_work_submit(&cleanup_work);
    return 0;
}

static void
broadcast_code_cb(struct bt_conn                                *conn,
                  const struct bt_bap_scan_delegator_recv_state *recv_state,
                  const uint8_t                                  broadcast_code[BT_ISO_BROADCAST_CODE_SIZE])
{
    ARG_UNUSED(conn);
    ARG_UNUSED(recv_state);

    memcpy(stored_broadcast_code, broadcast_code, BT_ISO_BROADCAST_CODE_SIZE);
    has_broadcast_code = true;
    LOG_INF("Received Broadcast Code from phone");

    if (base_received && sink_syncable && !big_synced && !big_stopping && requested_bis_sync != 0U)
    {
        k_work_submit(&big_sync_work);
    }
}

static int
bis_sync_req_cb(struct bt_conn                                *conn,
                const struct bt_bap_scan_delegator_recv_state *recv_state,
                const uint32_t                                 bis_sync_req[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS])
{
    ARG_UNUSED(conn);
    req_recv_state = recv_state;

    uint32_t req_bits = 0U;
    for (size_t i = 0; i < CONFIG_BT_BAP_BASS_MAX_SUBGROUPS; i++)
    {
        req_bits |= bis_sync_req[i];
    }

    LOG_INF("Phone BIS sync request: 0x%08X", req_bits);

    if (req_bits == 0U)
    {
        /* Phone paused broadcast reception: stop BIG stream ONLY, keep PA sync alive */
        requested_bis_sync = 0U;
        if (big_synced && !big_stopping && broadcast_sink != NULL)
        {
            big_stopping = true;
            bt_bap_broadcast_sink_stop(broadcast_sink);
        }
        audio_i2s_stop();
        return 0;
    }

    requested_bis_sync = req_bits;

    if (base_received && sink_syncable && !big_synced && !big_stopping)
    {
        k_work_submit(&big_sync_work);
    }

    return 0;
}

static struct bt_bap_scan_delegator_cb scan_delegator_cbs = {
    .recv_state_updated = recv_state_updated_cb,
    .pa_sync_req        = pa_sync_req_cb,
    .pa_sync_term_req   = pa_sync_term_req_cb,
    .broadcast_code     = broadcast_code_cb,
    .bis_sync_req       = bis_sync_req_cb,
};

/* --- System Workqueue Handlers --- */

static void
pa_create_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (suspended_by_unicast || pa_sync != NULL || target_broadcast_id == INVALID_BROADCAST_ID)
    {
        return;
    }

    struct bt_le_per_adv_sync_param create_params = { 0 };
    bt_addr_le_copy(&create_params.addr, &target_addr);
    create_params.options = BT_LE_PER_ADV_SYNC_OPT_FILTER_DUPLICATE;
    create_params.sid     = target_sid;
    create_params.skip    = 0;
    create_params.timeout = interval_to_sync_timeout(target_interval);

    int err = bt_le_per_adv_sync_create(&create_params, &pa_sync);
    if (err != 0)
    {
        LOG_ERR("Failed creating PA sync (err %d)", err);
        return;
    }

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
    if (err == 0 || err == -EALREADY)
    {
        scanning_for_pa = true;
    }

    LOG_INF("Waiting for Periodic Advertising lock...");
    k_work_reschedule(&watchdog_work, K_MSEC(PA_SYNC_TIMEOUT_MS));
}

static void
big_sync_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (suspended_by_unicast || broadcast_sink == NULL || !base_received || !sink_syncable || big_synced || big_stopping
        || requested_bis_sync == 0U)
    {
        return;
    }

    if (sink_encrypted && !has_broadcast_code)
    {
        LOG_INF("Broadcast is encrypted; waiting for phone to supply Broadcast Code...");
        return;
    }

    uint32_t sync_bitfield = base_bis_bitfield;
    if (requested_bis_sync != BT_BAP_BIS_SYNC_NO_PREF)
    {
        sync_bitfield = base_bis_bitfield & requested_bis_sync;
        if (sync_bitfield == 0U)
        {
            sync_bitfield = base_bis_bitfield;
        }
    }

    size_t stream_cnt = 0;
    for (int i = 1; i <= BT_ISO_MAX_GROUP_ISO_COUNT; i++)
    {
        if ((sync_bitfield & BT_ISO_BIS_INDEX_BIT(i)) != 0U)
        {
            stream_cnt++;
        }
    }

    if (stream_cnt == 0 || stream_cnt > BLE_AUDIO_MAX_BROADCAST_STREAMS)
    {
        stream_cnt = MIN(base_bis_count, (size_t)BLE_AUDIO_MAX_BROADCAST_STREAMS);
    }

    struct bt_bap_stream *streams[BLE_AUDIO_MAX_BROADCAST_STREAMS + 1] = { NULL };
    ble_audio_stream_get_broadcast_streams(streams, stream_cnt);

    LOG_INF("Synchronizing to Auracast BIG (bitfield 0x%08X, %u streams)...", sync_bitfield, (unsigned int)stream_cnt);

    const uint8_t *code_ptr = sink_encrypted ? stored_broadcast_code : NULL;
    int            err      = bt_bap_broadcast_sink_sync(broadcast_sink, sync_bitfield, streams, code_ptr);
    if (err != 0)
    {
        LOG_ERR("Failed syncing to Broadcast Sink BIG (err %d)", err);
        return;
    }

    big_synced = true;
    k_work_cancel_delayable(&watchdog_work);
}

static void
big_stopped_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    big_synced   = false;
    big_stopping = false;
    /* Require a fresh BASE + BIGInfo (syncable_cb) before any future BIG sync attempt */
    sink_syncable = false;
    base_received = false;

    if (pending_full_reset || suspended_by_unicast || pa_sync == NULL)
    {
        teardown_pa_and_sink();
    }
}

static void
cleanup_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    teardown_pa_and_sink();
}

static void
watchdog_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (pa_sync != NULL && broadcast_sink == NULL)
    {
        LOG_WRN("Periodic Advertising lock timed out; cleaning up");
        k_work_submit(&cleanup_work);
    }
}

void
ble_bap_broadcast_sink_on_connected(struct bt_conn *conn)
{
    struct bt_le_per_adv_sync_transfer_param past_param = {
        .skip    = 0,
        .timeout = 400, /* 4.0s */
        .options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE,
    };

    int err = bt_le_per_adv_sync_transfer_subscribe(conn, &past_param);
    if (err != 0 && err != -EALREADY)
    {
        LOG_DBG("PAST subscribe returned %d", err);
    }
}

void
ble_bap_broadcast_sink_suspend(void)
{
    suspended_by_unicast = true;

    bool had_activity = scanning_for_pa || (pa_sync != NULL) || (broadcast_sink != NULL);
    teardown_pa_and_sink();

    if (had_activity)
    {
        audio_i2s_stop();
        LOG_INF("Auracast Broadcast Sink suspended for Unicast stream");
    }
}

void
ble_bap_broadcast_sink_resume(void)
{
    if (!ble_audio_stream_has_active_unicast())
    {
        suspended_by_unicast = false;
    }
}

void
ble_bap_broadcast_sink_on_big_stopped(void)
{
    LOG_INF("Auracast BIG stream stopped");
    k_work_submit(&big_stopped_work);
}

void
ble_bap_broadcast_sink_start(void)
{
    k_work_init_delayable(&pa_create_work, pa_create_work_handler);
    k_work_init(&big_sync_work, big_sync_work_handler);
    k_work_init(&big_stopped_work, big_stopped_work_handler);
    k_work_init(&cleanup_work, cleanup_work_handler);
    k_work_init_delayable(&watchdog_work, watchdog_work_handler);

    bt_bap_scan_delegator_register_cb(&scan_delegator_cbs);
    bt_bap_broadcast_sink_register_cb(&broadcast_sink_cbs);
    bt_le_per_adv_sync_cb_register(&pa_sync_cbs);

    LOG_INF("BASS Scan Delegator & Broadcast Sink ready (Phone-Assisted Mode)");
}