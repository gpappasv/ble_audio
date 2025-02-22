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
    return;
}