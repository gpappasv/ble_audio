// --- includes ----------------------------------------------------------------
#include "ble_conn_control.h"

#include "ble_bap_unicast_server.h"

#include <stdbool.h>
#include <string.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/sys/byteorder.h>

// --- logging settings --------------------------------------------------------
LOG_MODULE_REGISTER(ble_m, LOG_LEVEL_INF);

// --- definitions -------------------------------------------------------------
#ifndef DEVICE_NAME
#define DEVICE_NAME     "nRF Audio Receiver"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#endif

// --- Basic GATT Service: Device Information Service -------------------------
// Read callback that returns a static manufacturer string.
static ssize_t
read_manufacturer(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const char *manufacturer = "Samsung";
    return bt_gatt_attr_read(conn, attr, buf, len, offset, manufacturer, strlen(manufacturer));
}

// Define the DIS (Device Information Service) with Manufacturer Name (UUID 0x2A29)
BT_GATT_SERVICE_DEFINE(dis_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0x180A)),
                       BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2A29),
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_manufacturer,
                                              NULL,
                                              NULL));

// --- static functions declarations -------------------------------------------
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

// --- static variables definitions --------------------------------------------
static struct bt_conn *ble_connection;

static K_SEM_DEFINE(sem_connected, 0U, 1U);
static K_SEM_DEFINE(sem_disconnected, 0U, 1U);
struct bt_le_ext_adv *adv;

static uint8_t unicast_server_addata[] = {
    BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),    /* ASCS UUID */
    BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, /* Target Announcement */
    BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
    BT_BYTES_LIST_LE16(AVAILABLE_SOURCE_CONTEXT),
    0x00, /* Metadata length */
};
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
    BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static K_SEM_DEFINE(ble_init_ok, 0, 1);

// --- Connection Callbacks ----------------------------------------------------
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

static void
connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err != 0U)
    {
        LOG_ERR("Failed to connect to %s (err %u)", addr, err);
        ble_connection = NULL;
        return;
    }

    LOG_INF("Connected: %s", addr);
    ble_connection = bt_conn_ref(conn);
    k_sem_give(&sem_connected);
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
    bt_conn_unref(ble_connection);
    ble_connection = NULL;
    k_sem_give(&sem_disconnected);
}

void
start_adv(void)
{
    int err;
    /* Create a connectable advertising set */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &adv);
    if (err)
    {
        LOG_ERR("Failed to create advertising set (err %d)\n", err);
        return;
    }

    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        LOG_ERR("Failed to set advertising data (err %d)\n", err);
        return;
    }

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err)
    {
        LOG_ERR("Failed to start advertising set (err %d)\n", err);
        return;
    }
}

// --- Functions Definitions ---------------------------------------------------
void
ble_conn_control_start(void)
{
    int error = bt_enable(NULL);
    if (error)
    {
        LOG_ERR("BLE initialization failed (err %d)", error);
        return;
    }

    LOG_INF("BLE initialized successfully");

    ble_bap_unicast_server_start();

    k_sem_give(&ble_init_ok);

    for (;;)
    {
        start_adv();
        // Fix this 
        error = k_sem_take(&sem_disconnected, K_FOREVER);
        if (error != 0)
        {
            printk("failed to take sem_disconnected (err %d)\n", error);
            return;
        }
    }
}
