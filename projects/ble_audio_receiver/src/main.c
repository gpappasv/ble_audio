#include "dac/audio_i2s.h"
#include "hw/board_buttons.h"
#include "hw/board_leds.h"
#include "ble/ble_audio_stream.h"
#include "ble/ble_bap_unicast_server.h"
#include "ble/ble_bap_broadcast_sink.h"
#include "ble/ble_conn_control.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int
main(void)
{
    int err;

    LOG_INF("=== AuraLink LE Audio Receiver Starting ===");

    board_leds_init();

    err = audio_i2s_init();
    if (err != 0)
    {
        LOG_ERR("Audio I2S / DSP subsystem init failed (err %d)", err);
        board_leds_notify_event(LED_EVT_ERROR);
        return err;
    }

    board_buttons_init();

    err = bt_enable(NULL);
    if (err != 0)
    {
        LOG_ERR("Bluetooth stack init failed (err %d)", err);
        board_leds_notify_event(LED_EVT_ERROR);
        return err;
    }

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    ble_audio_stream_init();
    ble_bap_unicast_server_start();
    ble_bap_broadcast_sink_start();
    ble_conn_control_init();

    LOG_INF("=== AuraLink Ready (Unicast + Auracast Sink) ===");
    return 0;
}