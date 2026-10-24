#ifndef BLE_CONN_CONTROL_H
#define BLE_CONN_CONTROL_H

#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>

void ble_conn_control_start(void);
bool ble_conn_control_is_active_conn(const struct bt_conn *conn);

#endif // BLE_CONN_CONTROL_H