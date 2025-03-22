#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/smf.h>

#include "calc_cycles.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// define macros
#define HEARTBEAT_PERIOD_MS 1000
#define HEARTBEAT_DUTY_CYCLE 25
#define HEARTBEAT_ON_TIME (HEARTBEAT_PERIOD_MS * HEARTBEAT_DUTY_CYCLE / 100)
#define HEARTBEAT_OFF_TIME (HEARTBEAT_PERIOD_MS - HEARTBEAT_ON_TIME)
#define LED_BLINK_DURATION_MS 5000
#define LED_DUTY_CYCLE 10

// declare function prototypes
void heartbeat_thread(void);
void setup_gpio(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void state_init_run(struct smf_ctx *ctx);
void state_idle_run(struct smf_ctx *ctx);
void state_reading_adc_run(struct smf_ctx *ctx);
void state_blinking_run(struct smf_ctx *ctx);

// define globals, DT-based hardware structs, and user structs
const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
const struct gpio_dt_spec led_out = GPIO_DT_SPEC_GET(DT_ALIAS(ledout), gpios);
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(getdata), gpios);
static struct gpio_callback button_cb_data;

// define states for state machine
enum state {
    STATE_INIT,
    STATE_IDLE,
    STATE_READING_ADC,
    STATE_BLINKING,
    STATE_COUNT
};

// State machine states
struct smf_state states[] = {
    [STATE_INIT] = SMF_CREATE_STATE(NULL, state_init_run, NULL, NULL, NULL),
    [STATE_IDLE] = SMF_CREATE_STATE(NULL, state_idle_run, NULL, NULL, NULL),
    [STATE_READING_ADC] = SMF_CREATE_STATE(NULL, state_reading_adc_run, NULL, NULL, NULL),
    [STATE_BLINKING] = SMF_CREATE_STATE(NULL, state_blinking_run, NULL, NULL, NULL),
};

// State machine context
struct smf_ctx smf;

// heartbeat thread
#define HEARTBEAT_THREAD_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIORITY 5

K_THREAD_DEFINE(heartbeat_thread_id, HEARTBEAT_THREAD_STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, HEARTBEAT_THREAD_PRIORITY, 0, 0);

// main function
int main(void)
{
    LOG_INF("Starting main function");
    smf_set_initial(&smf, &states[STATE_INIT]);

    while (1) {
        smf_run_state(&smf);
        k_sleep(K_MSEC(100));
    }
}

// function implementations
void heartbeat_thread(void)
{
    LOG_INF("Heartbeat thread started");
    while (1) {
        gpio_pin_set_dt(&heartbeat_led, 1);
        LOG_INF("Heartbeat LED ON");
        k_sleep(K_MSEC(HEARTBEAT_ON_TIME));
        gpio_pin_set_dt(&heartbeat_led, 0);
        LOG_INF("Heartbeat LED OFF");
        k_sleep(K_MSEC(HEARTBEAT_OFF_TIME));
    }
}

void setup_gpio(void)
{
    if (!device_is_ready(heartbeat_led.port)) {
        LOG_ERR("Heartbeat LED GPIO device not ready");
        return;
    }

    int ret = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure heartbeat LED GPIO pin");
    } else {
        LOG_INF("Heartbeat LED GPIO pin configured successfully");
    }

    if (!device_is_ready(led_out.port)) {
        LOG_ERR("LED output GPIO device not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&led_out, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED output GPIO pin");
    } else {
        LOG_INF("LED output GPIO pin configured successfully");
    }

    if (!device_is_ready(button.port)) {
        LOG_ERR("Button GPIO device not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button GPIO pin");
    } else {
        LOG_INF("Button GPIO pin configured successfully");
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure button interrupt");
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_INF("Button pressed, setting ADC read to 0V and changing state to BLINKING");
    smf_set_state(&smf, &states[STATE_READING_ADC]);
}

// state functions implementation
void state_init_run(struct smf_ctx *ctx)
{
    LOG_INF("State: INIT");
    setup_gpio();
    smf_set_state(ctx, &states[STATE_IDLE]);
}

void state_idle_run(struct smf_ctx *ctx)
{
    LOG_INF("State: IDLE");
    k_sleep(K_MSEC(1000));
}

void state_reading_adc_run(struct smf_ctx *ctx)
{
    LOG_INF("State: READING_ADC");
    // Set ADC read to 0V
    int32_t adc_value = 0;
    int32_t voltage = 0; // 0V
    LOG_INF("ADC value: %d, Voltage: %d mV", adc_value, voltage);

    smf_set_state(ctx, &states[STATE_BLINKING]);
}

void state_blinking_run(struct smf_ctx *ctx)
{
    LOG_INF("State: BLINKING");

    int32_t adc_value = 0;
    int32_t voltage = 0; // 0V
    int32_t blink_rate_hz = 1 + (voltage * 4 / 3000); // Map 0-3000 mV to 1-5 Hz
    int32_t blink_period_ms = 1000 / blink_rate_hz;
    int32_t blink_on_time = blink_period_ms * LED_DUTY_CYCLE / 100;
    int32_t blink_off_time = blink_period_ms - blink_on_time;

    LOG_INF("ADC value: %d, Voltage: %d mV, Blink rate: %d Hz", adc_value, voltage, blink_rate_hz);

    for (int i = 0; i < (LED_BLINK_DURATION_MS / blink_period_ms); i++) {
        gpio_pin_set_dt(&led_out, 1);
        k_sleep(K_MSEC(blink_on_time));
        gpio_pin_set_dt(&led_out, 0);
        k_sleep(K_MSEC(blink_off_time));
    }

    smf_set_state(ctx, &states[STATE_IDLE]);
}