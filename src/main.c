#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/smf.h> // CONFIG_SMF=y

#include "calc_cycles.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// Define macros for heartbeat and LED control
#define HEARTBEAT_PERIOD_MS 1000
#define HEARTBEAT_DUTY_CYCLE 25
#define HEARTBEAT_ON_TIME (HEARTBEAT_PERIOD_MS * HEARTBEAT_DUTY_CYCLE / 100)
#define HEARTBEAT_OFF_TIME (HEARTBEAT_PERIOD_MS - HEARTBEAT_ON_TIME)
#define LED_BLINK_DURATION_MS 5000
#define LED_DUTY_CYCLE 10

// ADC configuration macro
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                              \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}

// Declare function prototypes
void heartbeat_thread(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void state_init_run(struct smf_ctx *ctx);
void state_idle_run(struct smf_ctx *ctx);
void state_reading_adc_run(struct smf_ctx *ctx);
void state_blinking_enter(struct smf_ctx *ctx);
void state_blinking_run(struct smf_ctx *ctx);
void timer1_handler(struct k_timer *timer);
void timer2_handler(struct k_timer *timer);
void timer3_handler(struct k_timer *timer);
void setup(void);

// Define global variables and device tree-based hardware structs
const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
const struct gpio_dt_spec led_out = GPIO_DT_SPEC_GET(DT_ALIAS(ledout), gpios);
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(getdata), gpios);
static struct gpio_callback button_cb_data;

// ADC configuration
static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc);

// Global variable to store ADC value
static int32_t adc_value = 0;

// Define states for the state machine
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

// Heartbeat thread configuration
#define HEARTBEAT_THREAD_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIORITY 5

K_THREAD_DEFINE(heartbeat_thread_id, HEARTBEAT_THREAD_STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, HEARTBEAT_THREAD_PRIORITY, 0, 0);

// Timers
K_TIMER_DEFINE(timer1, timer1_handler, NULL);
K_TIMER_DEFINE(timer2, timer2_handler, NULL);
K_TIMER_DEFINE(timer3, timer3_handler, NULL);

// Main function
int main(void)
{
    LOG_INF("Main function started");
    smf_set_initial(&smf, &states[STATE_INIT]);

    while (1) {
        smf_run_state(&smf);
        k_sleep(K_MSEC(100));
    }
}

// Heartbeat thread function
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

// Button pressed callback function
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_INF("Button pressed, changing state to READING_ADC");
    smf_set_state(&smf, &states[STATE_READING_ADC]);
}

// State machine state functions
void state_init_run(struct smf_ctx *ctx)
{
    LOG_INF("State: INIT");
    setup();
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

    int16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf), // bytes
    };

    LOG_DBG("Initializing ADC sequence for %s (channel %d)", adc_vadc.dev->name, adc_vadc.channel_id);

    (void)adc_sequence_init_dt(&adc_vadc, &sequence);

    int ret = adc_read(adc_vadc.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC read failed (%d)", ret);
    } else {
        LOG_DBG("Raw ADC Buffer: %d", buf);
    }

    int32_t val_mv = buf;  // val_mv is now the raw ADC value
    ret = adc_raw_to_millivolts_dt(&adc_vadc, &val_mv); // remember that the vadc struct contains all the DT parameters
    if (ret < 0) {
        LOG_ERR("Failed to convert raw ADC value to millivolts (%d)", ret);
    } else {
        LOG_INF("ADC Value (mV): %d", val_mv);
    }

    // Store the ADC value in a global variable for use in the blinking state
    adc_value = val_mv;

    smf_set_state(ctx, &states[STATE_BLINKING]);
}

void state_blinking_enter(struct smf_ctx *ctx)
{
    LOG_INF("State: BLINKING_ENTER");

    int32_t voltage = adc_value; // Use the global adc_value
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

// Timer handlers
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

// Setup function to initialize GPIOs and ADC
void setup(void)
{
    // Setup GPIO for heartbeat LED
    if (!device_is_ready(heartbeat_led.port)) {
        LOG_ERR("Heartbeat LED GPIO not ready");
        return;
    }

    int ret = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure heartbeat LED GPIO");
    }

    // Setup GPIO for LED output
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

    // Setup GPIO for button input
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

    // Setup ADC
    if (!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("ADC controller device(s) not ready");
        return;
    }

    ret = adc_channel_setup_dt(&adc_vadc);
    if (ret < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", ret);
        return;
    }
}