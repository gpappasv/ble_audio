#include "board_leds.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_leds, LOG_LEVEL_INF);

#define LED_TICK_MS 50
#define LED0_NODE   DT_ALIAS(led0)
#define LED1_NODE   DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define HAS_LED0 1
#else
#define HAS_LED0 0
#endif

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
#define HAS_LED1 1
#else
#define HAS_LED1 0
#endif

static enum board_led_state current_state = LED_STATE_IDLE_ADV;
static enum board_led_event active_event  = LED_EVT_NONE;
static uint16_t             event_ticks   = 0;
static uint32_t             phase_tick    = 0;
static bool                 is_muted      = false;

static struct k_work_delayable led_tick_work;

static inline void
set_hw_leds(bool l0_on, bool l1_on)
{
#if HAS_LED0
    gpio_pin_set_dt(&led0, l0_on ? 1 : 0);
#else
    ARG_UNUSED(l0_on);
#endif
#if HAS_LED1
    gpio_pin_set_dt(&led1, l1_on ? 1 : 0);
#else
    ARG_UNUSED(l1_on);
#endif
}

static bool
render_base_state_led1(uint32_t tick)
{
    switch (current_state)
    {
        case LED_STATE_IDLE_ADV:
            /* Slow beacon flash: 100ms ON (2 ticks) every 2.0s (40 ticks) */
            return ((tick % 40U) < 2U);

        case LED_STATE_CONNECTED_IDLE:
        case LED_STATE_UNICAST_STREAMING:
        case LED_STATE_AURACAST_SYNCING:
        case LED_STATE_AURACAST_STREAMING:
        default:
            /* Solid ON whenever connected or active */
            return true;
    }
}

static bool
render_base_state_led2(uint32_t tick)
{
    if (current_state == LED_STATE_UNICAST_STREAMING || current_state == LED_STATE_AURACAST_STREAMING)
    {
        if (is_muted)
        {
            /* Muted warning during active stream: symmetric 500ms ON / 500ms OFF */
            return ((tick % 20U) < 10U);
        }
    }

    switch (current_state)
    {
        case LED_STATE_IDLE_ADV:
        case LED_STATE_CONNECTED_IDLE:
            return false;

        case LED_STATE_UNICAST_STREAMING:
            /* Solid ON for active Unicast stream */
            return true;

        case LED_STATE_AURACAST_SYNCING:
            /* Fast 5 Hz acquisition blink (100ms ON / 100ms OFF) */
            return ((tick % 4U) < 2U);

        case LED_STATE_AURACAST_STREAMING:
        {
            /* Double-heartbeat cadence: ON 100ms, OFF 100ms, ON 100ms, OFF 700ms (20 ticks) */
            uint32_t m = tick % 20U;
            return (m < 2U) || (m >= 4U && m < 6U);
        }

        default:
            return false;
    }
}

static void
led_tick_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    phase_tick++;

    bool l0 = render_base_state_led1(phase_tick);
    bool l1 = render_base_state_led2(phase_tick);

    /* Process transient overlay events if active */
    if (active_event != LED_EVT_NONE && event_ticks > 0U)
    {
        event_ticks--;

        switch (active_event)
        {
            case LED_EVT_BOOT:
            case LED_EVT_RESET_BONDS:
                /* Fast alternating knight-rider strobe (50ms flip) */
                l0 = ((event_ticks & 1U) == 0U);
                l1 = !l0;
                break;

            case LED_EVT_CONNECTED:
                /* Crisp synchronized double-flash */
                l0 = ((event_ticks % 3U) != 0U);
                l1 = l0;
                break;

            case LED_EVT_DISCONNECTED:
                /* Brief dual extinguish + single pulse */
                l0 = (event_ticks == 3U);
                l1 = (event_ticks == 3U);
                break;

            case LED_EVT_VOL_CHANGE:
                /* 1-tick tactile inversion blip on LED 2 */
                l1 = !l1;
                break;

            case LED_EVT_VOL_MAX:
                /* Rapid flutter on LED 2 */
                l1 = ((event_ticks & 1U) == 1U);
                break;

            case LED_EVT_ERROR:
                /* Rapid synchronized alarm bursts on both LEDs */
                l0 = ((event_ticks & 1U) == 1U);
                l1 = l0;
                break;

            default:
                break;
        }

        if (event_ticks == 0U)
        {
            active_event = LED_EVT_NONE;
        }
    }

    set_hw_leds(l0, l1);
    k_work_reschedule(&led_tick_work, K_MSEC(LED_TICK_MS));
}

int
board_leds_init(void)
{
#if HAS_LED0
    if (gpio_is_ready_dt(&led0))
    {
        gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    }
#endif
#if HAS_LED1
    if (gpio_is_ready_dt(&led1))
    {
        gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    }
#endif

    k_work_init_delayable(&led_tick_work, led_tick_work_handler);
    board_leds_notify_event(LED_EVT_BOOT);
    k_work_reschedule(&led_tick_work, K_NO_WAIT);

    LOG_INF("Dual-LED status engine initialized");
    return 0;
}

void
board_leds_set_state(enum board_led_state state)
{
    current_state = state;
}

void
board_leds_notify_event(enum board_led_event event)
{
    /* Do not allow minor volume blips to preempt major reset/error animations */
    if ((active_event == LED_EVT_RESET_BONDS || active_event == LED_EVT_ERROR)
        && (event == LED_EVT_VOL_CHANGE || event == LED_EVT_VOL_MAX))
    {
        return;
    }

    active_event = event;

    switch (event)
    {
        case LED_EVT_BOOT:
            event_ticks = 12U; /* 600 ms */
            break;
        case LED_EVT_RESET_BONDS:
            event_ticks = 16U; /* 800 ms */
            break;
        case LED_EVT_CONNECTED:
            event_ticks = 8U; /* 400 ms */
            break;
        case LED_EVT_DISCONNECTED:
            event_ticks = 6U; /* 300 ms */
            break;
        case LED_EVT_VOL_CHANGE:
            event_ticks = 1U; /* 50 ms blip */
            break;
        case LED_EVT_VOL_MAX:
            event_ticks = 6U; /* 300 ms flutter */
            break;
        case LED_EVT_ERROR:
            event_ticks = 10U; /* 500 ms alarm */
            break;
        default:
            event_ticks = 0U;
            break;
    }
}

void
board_leds_set_muted(bool muted)
{
    is_muted = muted;
}