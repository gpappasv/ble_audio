#ifndef BLE_BAP_BROADCAST_SINK_H
#define BLE_BAP_BROADCAST_SINK_H

#include <zephyr/bluetooth/conn.h>

void ble_bap_broadcast_sink_start(void);
void ble_bap_broadcast_sink_suspend(void);
void ble_bap_broadcast_sink_resume(void);
void ble_bap_broadcast_sink_on_connected(struct bt_conn *conn);
void ble_bap_broadcast_sink_on_big_stopped(void);

#endif // BLE_BAP_BROADCAST_SINK_H