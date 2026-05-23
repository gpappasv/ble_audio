// --- includes ----------------------------------------------------------------
#include "ble/ble_conn_control.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// --- logging settings --------------------------------------------------------
LOG_MODULE_REGISTER(main_m);

// --- main execution entry point ----------------------------------------------
int
main(void)
{
    // Initializes stack and starts background advertising
    ble_conn_control_start();
    LOG_INF("Main application execution environment loaded cleanly");

    while (1)
    {
        // Keep the main thread alive inside a low-power kernel sleep state
        k_sleep(K_FOREVER);
    }

    return 0;
}