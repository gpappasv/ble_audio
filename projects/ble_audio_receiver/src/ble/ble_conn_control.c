#include "ble_conn_control.h"
#include "ble_bap_unicast_server.h"
#include "ble_bap_broadcast_sink.h"
#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_conn_control, LOG_LEVEL_INF);

static struct bt_conn       *active_conn   = NULL;
static struct bt_le_ext_adv *ext_adv       = NULL;
static bool                  adv_running   = false;
static bool                  reset_in_prog = false;

static struct k_work_delayable adv_work;
static struct k_work_delayable sec_work;
static struct k_work           evict_reset_work;
static struct k_work_delayable finalize_unpair_work;

/* Common Audio Service (CAS / CAP) announcement */
static uint8_t cap_svc_data[] = {
    BT_UUID_16_ENCODE(BT_UUID_CAS_VAL),
    BT_AUDIO_UNICAST_ANNOUNCEMENT_GENERAL,
};

/* BAP Unicast Server announcement */
static uint8_t bap_svc_data[] = {
    BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
    BT_AUDIO_UNICAST_ANNOUNCEMENT_GENERAL,
    BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
    BT_BYTES_LIST_LE16(0x0000), /* Source contexts: None */
    0x00,                       /* Metadata length: 0 */
};

/* BASS Broadcast Audio Scan Service announcement */
static uint8_t bass_svc_data[] = {
    BT_UUID_16_ENCODE(BT_UUID_BASS_VAL),
};

static const struct bt_data ad_payload[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME,
                  BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BASS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_PACS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_VCS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_CAS_VAL)),
    BT_DATA(BT_DATA_SVC_DATA16, cap_svc_data, ARRAY_SIZE(cap_svc_data)),
    BT_DATA(BT_DATA_SVC_DATA16, bap_svc_data, ARRAY_SIZE(bap_svc_data)),
    BT_DATA(BT_DATA_SVC_DATA16, bass_svc_data, ARRAY_SIZE(bass_svc_data)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void
stop_advertising_if_active(void)
{
    if (ext_adv != NULL && adv_running)
    {
        bt_le_ext_adv_stop(ext_adv);
        adv_running = false;
    }
}

static void
adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (reset_in_prog || active_conn != NULL)
    {
        return;
    }

    int err;
    if (ext_adv == NULL)
    {
        struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONNECTABLE,
                                                                BT_GAP_ADV_FAST_INT_MIN_1,
                                                                BT_GAP_ADV_FAST_INT_MAX_1,
                                                                NULL);

        err = bt_le_ext_adv_create(&adv_param, NULL, &ext_adv);
        if (err != 0)
        {
            LOG_ERR("Failed creating extended advertising set (err %d)", err);
            return;
        }

        err = bt_le_ext_adv_set_data(ext_adv, ad_payload, ARRAY_SIZE(ad_payload), NULL, 0);
        if (err != 0)
        {
            LOG_ERR("Failed setting extended advertising data (err %d)", err);
            return;
        }
    }

    err = bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err == 0 || err == -EALREADY)
    {
        adv_running = true;
        LOG_INF("Connectable LE Audio advertising active (%s)", CONFIG_BT_DEVICE_NAME);
    }
    else
    {
        LOG_WRN("Extended advertising start returned %d; retrying...", err);
        k_work_reschedule(&adv_work, K_MSEC(500));
    }
}

static void
sec_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (active_conn != NULL && bt_conn_get_security(active_conn) < BT_SECURITY_L2)
    {
        int err = bt_conn_set_security(active_conn, BT_SECURITY_L2);
        if (err != 0 && err != -EALREADY)
        {
            LOG_DBG("Security elevation request returned %d", err);
        }
    }
}

/* Step 2 of Button 1 Reset: Runs after ACL link has cleanly terminated */
static void
finalize_unpair_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (active_conn != NULL)
    {
        bt_conn_unref(active_conn);
        active_conn = NULL;
    }

    int err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
    if (err != 0)
    {
        LOG_WRN("bt_unpair returned %d", err);
    }
    else
    {
        LOG_INF("All paired device bonds purged from flash");
    }

    reset_in_prog = false;
    k_work_reschedule(&adv_work, K_MSEC(100));
}

/* Step 1 of Button 1 Reset: Evicts streams, disconnects active phone, schedules bond wipe */
static void
evict_reset_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    reset_in_prog = true;
    k_work_cancel_delayable(&adv_work);
    k_work_cancel_delayable(&sec_work);

    stop_advertising_if_active();

    /* 1. Kill Auracast sink & purge BASS source slots */
    ble_bap_broadcast_sink_stop_and_cleanup();
    ble_bap_broadcast_sink_resume();

    /* 2. Evict Unicast streams and kick out connected phone if present */
    if (active_conn != NULL)
    {
        ble_bap_unicast_server_evict_conn(active_conn);
        audio_i2s_stop();

        LOG_INF("Disconnecting active device prior to bond wipe...");
        int err = bt_conn_disconnect(active_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (err == 0 || err == -EALREADY)
        {
            /* Allow 250ms for LL_TERMINATE_IND to complete over the air */
            k_work_reschedule(&finalize_unpair_work, K_MSEC(250));
            return;
        }

        bt_conn_unref(active_conn);
        active_conn = NULL;
    }

    /* No active connection: wipe bonds immediately */
    k_work_reschedule(&finalize_unpair_work, K_NO_WAIT);
}

static void
connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err != 0)
    {
        LOG_WRN("Connection attempt failed: %s (err 0x%02x)", addr, err);
        k_work_reschedule(&adv_work, K_MSEC(200));
        return;
    }

    if (reset_in_prog)
    {
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    if (active_conn != NULL && conn != active_conn)
    {
        LOG_WRN("Rejecting secondary connection from %s; slot already occupied", addr);
        bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
        return;
    }

    active_conn = bt_conn_ref(conn);
    adv_running = false;

    LOG_INF("Connected: %s", addr);

    ble_bap_broadcast_sink_on_connected(conn);
    k_work_reschedule(&sec_work, K_MSEC(300));
}

static void
disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn == active_conn)
    {
        LOG_INF("Active device disconnected: %s (reason 0x%02x)", addr, reason);
        k_work_cancel_delayable(&sec_work);

        ble_bap_unicast_server_evict_conn(active_conn);
        ble_bap_broadcast_sink_stop_and_cleanup();
        ble_bap_broadcast_sink_resume();

        bt_conn_unref(active_conn);
        active_conn = NULL;

        if (reset_in_prog)
        {
            /* Disconnect finished during Button 1 reset: proceed immediately to bond wipe */
            k_work_reschedule(&finalize_unpair_work, K_MSEC(20));
            return;
        }

        k_work_reschedule(&adv_work, K_MSEC(150));
        return;
    }

    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);
    if (!reset_in_prog)
    {
        k_work_reschedule(&adv_work, K_MSEC(200));
    }
}

static void
security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    if (err == 0)
    {
        LOG_INF("Security level elevated to L%u", (unsigned int)level);
    }
    else
    {
        LOG_WRN("Security elevation failed: level %u, err %d", (unsigned int)level, err);
        if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING)
        {
            LOG_WRN("Peer missing bond key -> Purging stale local bond");
            bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
        }
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
};

bool
ble_conn_control_is_active_conn(const struct bt_conn *conn)
{
    return (conn != NULL && conn == active_conn);
}

void
ble_conn_control_reset_all_and_unpair(void)
{
    k_work_submit(&evict_reset_work);
}

void
ble_conn_control_init(void)
{
    k_work_init_delayable(&adv_work, adv_work_handler);
    k_work_init_delayable(&sec_work, sec_work_handler);
    k_work_init(&evict_reset_work, evict_reset_work_handler);
    k_work_init_delayable(&finalize_unpair_work, finalize_unpair_work_handler);

    k_work_reschedule(&adv_work, K_NO_WAIT);
}