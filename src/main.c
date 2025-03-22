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
void state_blinking_enter(struct smf_ctx *ctx);
void state_blinking_run(struct smf_ctx *ctx);
void timer1_handler(struct k_timer *timer);
void timer2_handler(struct k_timer *timer);
void timer3_handler(struct k_timer *timer);

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
    STATE_BLINKING
};

// State machine states
struct smf_state states[] = {
    [STATE_INIT] = SMF_CREATE_STATE(NULL, state_init_run, NULL, NULL, NULL),
    [STATE_IDLE] = SMF_CREATE_STATE(NULL, state_idle_run, NULL, NULL, NULL),
    [STATE_READING_ADC] = SMF_CREATE_STATE(NULL, state_reading_adc_run, NULL, NULL, NULL),
    [STATE_BLINKING] = SMF_CREATE_STATE(state_blinking_enter, state_blinking_run, NULL, NULL, NULL),
};

// State machine context
struct smf_ctx smf;

// heartbeat thread
#define HEARTBEAT_THREAD_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIORITY 5

K_THREAD_DEFINE(heartbeat_thread_id, HEARTBEAT_THREAD_STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, HEARTBEAT_THREAD_PRIORITY, 0, 0);

// timers
K_TIMER_DEFINE(timer1, timer1_handler, NULL);
K_TIMER_DEFINE(timer2, timer2_handler, NULL);
K_TIMER_DEFINE(timer3, timer3_handler, NULL);

// main function
int main(void)
{
    LOG_INF("Main function started");
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
        k_sleep(K_MSEC(HEARTBEAT_ON_TIME));
        gpio_pin_set_dt(&heartbeat_led, 0);
        k_sleep(K_MSEC(HEARTBEAT_OFF_TIME));
    }
}

void setup_gpio(void)
{
    if (!device_is_ready(heartbeat_led.port)) {
        LOG_ERR("Heartbeat LED GPIO not ready");
        return;
    }

    int ret = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure heartbeat LED GPIO");
    }

    if (!device_is_ready(led_out.port)) {
        LOG_ERR("LED output GPIO not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&led_out, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED output GPIO");
    } else {
        gpio_pin_set_dt(&led_out, 0); // Ensure LED is off during initialization
    }

    if (!device_is_ready(button.port)) {
        LOG_ERR("Button GPIO not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button GPIO");
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
    LOG_INF("Button pressed, changing state to READING_ADC");
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
    int32_t adc_value = 0;
    int32_t voltage = 0; // 0V
    LOG_INF("ADC value: %d, Voltage: %d mV", adc_value, voltage);

    smf_set_state(ctx, &states[STATE_BLINKING]);
}

void state_blinking_enter(struct smf_ctx *ctx)
{
    LOG_INF("State: BLINKING_ENTER");

    int32_t adc_value = 0;
    int32_t voltage = 0; // 0V
    int32_t blink_rate_hz = 1 + (voltage * 4 / 3000); // Map 0-3000 mV to 1-5 Hz
    int32_t blink_period_ms = 1000 / blink_rate_hz;
    int32_t blink_on_time = blink_period_ms * LED_DUTY_CYCLE / 100;

    LOG_INF("Blink rate: %d Hz, Blink period: %d ms, Blink on time: %d ms", blink_rate_hz, blink_period_ms, blink_on_time);

    k_timer_start(&timer1, K_MSEC(LED_BLINK_DURATION_MS), K_NO_WAIT);
    LOG_INF("Timer1 started for %d ms", LED_BLINK_DURATION_MS);

    k_timer_start(&timer2, K_NO_WAIT, K_MSEC(blink_period_ms));
    LOG_INF("Timer2 started with period %d ms", blink_period_ms);

    k_timer_start(&timer3, K_MSEC(blink_on_time), K_MSEC(blink_period_ms));
    LOG_INF("Timer3 started with initial delay %d ms and period %d ms", blink_on_time, blink_period_ms);
}

void state_blinking_run(struct smf_ctx *ctx)
{
    LOG_INF("State: BLINKING_RUN");
}

void timer1_handler(struct k_timer *timer)
{
    LOG_INF("Timer1 handler called, stopping timers 2 and 3");
    k_timer_stop(&timer2);
    LOG_INF("Timer2 stopped");
    k_timer_stop(&timer3);
    LOG_INF("Timer3 stopped");
    smf_set_state(&smf, &states[STATE_IDLE]);
}

void timer2_handler(struct k_timer *timer)
{
    LOG_INF("Timer2 handler called, turning LED on");
    gpio_pin_set_dt(&led_out, 1);
}

void timer3_handler(struct k_timer *timer)
{
    LOG_INF("Timer3 handler called, turning LED off");
    gpio_pin_set_dt(&led_out, 0);
}