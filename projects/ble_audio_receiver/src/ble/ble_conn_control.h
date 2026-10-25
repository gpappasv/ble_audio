#ifndef BLE_CONN_CONTROL_H
#define BLE_CONN_CONTROL_H

#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>

/**
 * @brief Initialize BLE advertising, connection callbacks, and security worker.
 */
void ble_conn_control_init(void);

/**
 * @brief Check if the supplied connection handle is the active master connection.
 */
bool ble_conn_control_is_active_conn(const struct bt_conn *conn);

/**
 * @brief Terminate all active Unicast & Auracast streams, kick out any connected
 *        device, wipe all paired bonds from flash, and restart advertising.
 */
void ble_conn_control_reset_all_and_unpair(void);

#endif /* BLE_CONN_CONTROL_H */