#include "ble_conn_control.h"
#include "ble_bap_unicast_server.h"
#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_conn, LOG_LEVEL_INF);

static struct bt_conn       *ble_connection;
static struct bt_le_ext_adv *adv;
static struct k_work         adv_work;

static void start_adv(void);

static void
adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    start_adv();
}

// ASCS Targeted Announcement metadata
static uint8_t unicast_server_addata[] = {
    BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
    BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED,
    BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
    BT_BYTES_LIST_LE16(0x0000), // Pure Sink
    0x00,                       // Metadata length
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL), BT_UUID_16_ENCODE(BT_UUID_CAS_VAL)),
    BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void
connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        LOG_ERR("Connection failed: %s (err %u)", addr, err);
        ble_connection = NULL;
        k_work_submit(&adv_work);
        return;
    }

    LOG_INF("Connected: %s", addr);
    ble_connection = bt_conn_ref(conn);

    // Elevate security to L2 (encryption required by Android BAP clients)
    int sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec_err)
    {
        LOG_WRN("Security initiation returned %d", sec_err);
    }
}

static void
disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    if (conn != ble_connection)
    {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

    audio_i2s_stop();

    bt_conn_unref(ble_connection);
    ble_connection = NULL;

    // Re-advertise via workqueue after conn context is fully freed
    k_work_submit(&adv_work);
}

static void
security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    if (err)
    {
        LOG_ERR("Security elevation failed: level %u, err %d", level, err);
    }
    else
    {
        LOG_INF("Link secured: level %u", level);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
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
        LOG_ERR("Failed starting ext adv (err %d)", err);
        return;
    }

    LOG_INF("LE Audio advertising active");
}

void
ble_conn_control_start(void)
{
    k_work_init(&adv_work, adv_work_handler);

    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");
    ble_bap_unicast_server_start();
    start_adv();
}