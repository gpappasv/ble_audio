// --- includes ----------------------------------------------------------------
#include "ble/ble_conn_control.h"
#include "dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_clock.h>

LOG_MODULE_REGISTER(main_m, LOG_LEVEL_INF);

static void
init_hardware_clocks(void)
{
    /* 1. Run Application Core Cortex-M33 at full 128 MHz (Div1 instead of default 64 MHz Div2) */
    nrf_clock_hfclk_div_set(NRF_CLOCK, NRF_CLOCK_HFCLK_DIV_1);

    /* 2. Lock the 32 MHz external crystal (HFXO) permanently on so the Audio PLL
     *    never falls back to the internal RC oscillator (HFINT) between BLE events. */
    if (!nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY))
    {
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);

        for (int i = 0; i < 500; i++)
        {
            if (nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED)
                || nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY))
            {
                break;
            }
            k_busy_wait(10);
        }
    }

    LOG_INF("App Core running at 128 MHz, HFXO crystal locked");
}

int
main(void)
{
    init_hardware_clocks();

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