#ifndef BOARD_LEDS_H
#define BOARD_LEDS_H

#include <stdbool.h>

enum board_led_state
{
    LED_STATE_IDLE_ADV = 0,
    LED_STATE_CONNECTED_IDLE,
    LED_STATE_UNICAST_STREAMING,
    LED_STATE_AURACAST_SYNCING,
    LED_STATE_AURACAST_STREAMING,
};

enum board_led_event
{
    LED_EVT_NONE = 0,
    LED_EVT_BOOT,
    LED_EVT_CONNECTED,
    LED_EVT_DISCONNECTED,
    LED_EVT_VOL_CHANGE,
    LED_EVT_VOL_MAX,
    LED_EVT_RESET_BONDS,
    LED_EVT_ERROR,
};

/**
 * @brief Initialize LED 1 (led0) and LED 2 (led1) and start the pattern engine.
 */
int board_leds_init(void);

/**
 * @brief Set the persistent background operating state for the LEDs.
 */
void board_leds_set_state(enum board_led_state state);

/**
 * @brief Trigger a high-priority transient visual animation before returning to base state.
 */
void board_leds_notify_event(enum board_led_event event);

/**
 * @brief Indicate whether the audio output is currently muted (or volume == 0).
 */
void board_leds_set_muted(bool muted);

#endif /* BOARD_LEDS_H */