#include "board_buttons.h"
#include "../ble/ble_conn_control.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_buttons, LOG_LEVEL_INF);

#define BUTTON_DEBOUNCE_MS 50
#define SW0_NODE           DT_ALIAS(sw0)

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
static const struct gpio_dt_spec button_sw0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, { 0 });
static struct gpio_callback      button_cb_data;
static struct k_work_delayable   button_debounce_work;

static void
button_debounce_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Confirm button is still physically pressed after 50ms debounce window */
    int val = gpio_pin_get_dt(&button_sw0);
    if (val > 0)
    {
        LOG_INF("Button 1 pressed -> Triggering full BLE eviction & bond reset");
        ble_conn_control_reset_all_and_unpair();
    }
}

static void
button_isr_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_work_reschedule(&button_debounce_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}
#endif

int
board_buttons_init(void)
{
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    if (!gpio_is_ready_dt(&button_sw0))
    {
        LOG_ERR("Button 1 GPIO device not ready");
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&button_sw0, GPIO_INPUT);
    if (err != 0)
    {
        LOG_ERR("Failed configuring Button 1 pin (err %d)", err);
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&button_sw0, GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0)
    {
        LOG_ERR("Failed configuring Button 1 interrupt (err %d)", err);
        return err;
    }

    k_work_init_delayable(&button_debounce_work, button_debounce_work_handler);
    gpio_init_callback(&button_cb_data, button_isr_callback, BIT(button_sw0.pin));
    gpio_add_callback(button_sw0.port, &button_cb_data);

    LOG_INF("Button 1 (sw0) initialized: Press to evict connections & clear bonds");
    return 0;
#else
    LOG_WRN("sw0 alias not found in devicetree; hardware button disabled");
    return 0;
#endif
}