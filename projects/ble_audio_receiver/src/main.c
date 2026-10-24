// --- includes ----------------------------------------------------------------
#include "ble/ble_conn_control.h"

#include "dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main_m);

int
main(void)
{
    audio_i2s_init(48000);
    // Initializes stack and starts background unicast advertising
    ble_conn_control_start();
    // Starts background Auracast scanning and synchronization loops
    // TODO: deactivate until audio streaming works
    // ble_auracast_start();

    LOG_INF("Main application execution environment loaded cleanly");

    while (1)
    {
        k_sleep(K_FOREVER);
    }

    return 0;
}