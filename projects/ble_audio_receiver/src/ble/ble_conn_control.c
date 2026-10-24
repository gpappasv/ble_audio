#include "ble_conn_control.h"
#include "ble_bap_unicast_server.h"
#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_conn, LOG_LEVEL_INF);

static struct bt_conn       *active_conn    = NULL;
static struct bt_conn       *displaced_conn = NULL;
static struct bt_le_ext_adv *adv            = NULL;

static struct k_work_delayable adv_work;
static struct k_work_delayable sec_work;
static struct k_work           unpair_work;
static bt_addr_le_t            stale_peer_addr;
static bool                    has_stale_peer = false;

static void start_adv(void);

static uint8_t unicast_server_addata[] = {
    BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
    BT_AUDIO_UNICAST_ANNOUNCEMENT_GENERAL,
    BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
    BT_BYTES_LIST_LE16(0x0000), // Pure Sink
    0x00,                       // Metadata length
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_CAS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_PACS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_VCS_VAL)),
    BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

bool
ble_conn_control_is_active_conn(const struct bt_conn *conn)
{
    return (conn != NULL) && (conn == active_conn);
}

static void
adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (displaced_conn == NULL)
    {
        start_adv();
    }
}

static void
sec_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (active_conn == NULL)
    {
        return;
    }

    if (bt_conn_get_security(active_conn) < BT_SECURITY_L2)
    {
        int sec_err = bt_conn_set_security(active_conn, BT_SECURITY_L2);
        if (sec_err && sec_err != -EALREADY && sec_err != -EBUSY)
        {
            LOG_WRN("Security request returned %d", sec_err);
        }
    }
}

static void
unpair_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!has_stale_peer)
    {
        return;
    }

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&stale_peer_addr, addr, sizeof(addr));
    LOG_WRN("Purging stale bond for %s to allow clean re-pairing", addr);

    has_stale_peer = false;
    bt_unpair(BT_ID_DEFAULT, &stale_peer_addr);

    if (active_conn != NULL && bt_addr_le_cmp(bt_conn_get_dst(active_conn), &stale_peer_addr) == 0)
    {
        k_work_reschedule(&sec_work, K_MSEC(150));
    }
}

static void
connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        LOG_ERR("Connection failed: %s (err %u)", addr, err);
        k_work_reschedule(&adv_work, K_MSEC(200));
        return;
    }

    if (active_conn != NULL && active_conn != conn)
    {
        char old_addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(active_conn), old_addr, sizeof(old_addr));
        LOG_INF("Smart Connect takeover: %s displacing active device %s", addr, old_addr);

        ble_bap_unicast_server_evict_conn(active_conn);
        audio_i2s_stop();

        if (displaced_conn != NULL)
        {
            bt_conn_unref(displaced_conn);
            displaced_conn = NULL;
        }

        displaced_conn = active_conn;
        active_conn    = bt_conn_ref(conn);

        int disc_err = bt_conn_disconnect(displaced_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (disc_err)
        {
            LOG_WRN("Disconnect of displaced conn returned %d", disc_err);
            bt_conn_unref(displaced_conn);
            displaced_conn = NULL;
        }
    }
    else
    {
        LOG_INF("Connected: %s", addr);
        if (active_conn != NULL)
        {
            bt_conn_unref(active_conn);
        }
        active_conn = bt_conn_ref(conn);
    }

    k_work_reschedule(&sec_work, K_MSEC(350));
}

static void
disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn == displaced_conn)
    {
        LOG_INF("Displaced device disconnected cleanly: %s (reason 0x%02x)", addr, reason);
        bt_conn_unref(displaced_conn);
        displaced_conn = NULL;

        k_work_reschedule(&adv_work, K_MSEC(250));
        return;
    }

    if (conn == active_conn)
    {
        LOG_INF("Active device disconnected: %s (reason 0x%02x)", addr, reason);
        k_work_cancel_delayable(&sec_work);

        ble_bap_unicast_server_evict_conn(active_conn);
        audio_i2s_stop();

        bt_conn_unref(active_conn);
        active_conn = NULL;

        k_work_reschedule(&adv_work, K_MSEC(150));
        return;
    }

    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);
    k_work_reschedule(&adv_work, K_MSEC(200));
}

static void
security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err)
    {
        LOG_INF("Link secured (%s): level %u", addr, level);
        return;
    }

    LOG_ERR("Security elevation failed (%s): level %u, err %d", addr, level, err);

    if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING || err == BT_SECURITY_ERR_AUTH_FAIL
        || err == BT_SECURITY_ERR_PAIR_NOT_ALLOWED)
    {
        bt_addr_le_copy(&stale_peer_addr, bt_conn_get_dst(conn));
        has_stale_peer = true;
        k_work_submit(&unpair_work);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
};

static void
pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing complete: %s (bonded: %s)", addr, bonded ? "yes" : "no");
}

static void
pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_WRN("Pairing failed: %s (reason %d)", addr, reason);

    bt_addr_le_copy(&stale_peer_addr, bt_conn_get_dst(conn));
    has_stale_peer = true;
    k_work_submit(&unpair_work);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
    .pairing_complete = pairing_complete,
    .pairing_failed   = pairing_failed,
};

static void
start_adv(void)
{
    int err;

    if (!adv)
    {
        err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &adv);
        if (err)
        {
            LOG_ERR("Failed creating ext adv (err %d)", err);
            return;
        }

        err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err)
        {
            LOG_ERR("Failed setting adv data (err %d)", err);
            return;
        }
    }

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err && err != -EALREADY)
    {
        if (err != -ENOMEM)
        {
            LOG_WRN("Ext adv start deferred (err %d)", err);
        }
        return;
    }

    LOG_INF("LE Audio advertising active");
}

void
ble_conn_control_start(void)
{
    k_work_init_delayable(&adv_work, adv_work_handler);
    k_work_init_delayable(&sec_work, sec_work_handler);
    k_work_init(&unpair_work, unpair_work_handler);

    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    bt_conn_auth_info_cb_register(&auth_info_cb);

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        settings_load();
        LOG_INF("Persistent Bluetooth identity and bonds loaded from flash");
    }

    ble_bap_unicast_server_start();

    LOG_INF("Bluetooth initialized");
    start_adv();
}