// --- includes ----------------------------------------------------------------
#include "ble/ble_conn_control.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// --- logging settings --------------------------------------------------------
LOG_MODULE_REGISTER(main_m);

// --- static functions declarations -------------------------------------------

// --- static variables definitions --------------------------------------------

// --- structs -----------------------------------------------------------------

// --- static functions definitions --------------------------------------------

// --- functions definitions ---------------------------------------------------
int
main(void)
{
    ble_conn_control_start();
    // TODO: At this point main is already blocked by ble_conn_control_start.
    // I need to set up a new thread for that.
    return 0;
}