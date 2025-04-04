#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/smf.h>

#include "calc_cycles.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Constants
#define HEARTBEAT_PERIOD_MS 1000
#define HEARTBEAT_DUTY_CYCLE 25
#define HEARTBEAT_ON_TIME (HEARTBEAT_PERIOD_MS * HEARTBEAT_DUTY_CYCLE / 100)
#define HEARTBEAT_OFF_TIME (HEARTBEAT_PERIOD_MS - HEARTBEAT_ON_TIME)
#define LED_BLINK_DURATION_MS 5000
#define LED_DUTY_CYCLE 10

// GPIO and ADC Specifications
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button0_cb;

static const struct adc_dt_spec adc_vadc_spec = {
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(vadc))),
    .channel_id = DT_REG_ADDR(DT_ALIAS(vadc)),
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(vadc))
};

// Function Prototypes
void heartbeat_thread(void *a, void *b, void *c);
void blink_handler(struct k_timer *timer);

void init_run(void *o);
void idle_run(void *o);
void adc_read_run(void *o);
void blinking_entry(void *o);
void blinking_run(void *o);

void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

void blink_duration_handler(struct k_timer *timer);
void led_on_handler(struct k_timer *timer);
void led_off_handler(struct k_timer *timer);

// System Context
struct system_context {
    struct smf_ctx ctx;
    bool button0_pressed;
    int32_t adc_voltage_mv;
    uint32_t led1_blink_period_ms;
} system_context;

// State Machine
enum state { INIT, IDLE, ADC_READ, BLINKING };
static const struct smf_state states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(NULL, idle_run, NULL, NULL, NULL),
    [ADC_READ] = SMF_CREATE_STATE(NULL, adc_read_run, NULL, NULL, NULL),
    [BLINKING] = SMF_CREATE_STATE(blinking_entry, blinking_run, NULL, NULL, NULL),
};

// Helper Functions
static void configure_gpio(const struct gpio_dt_spec *spec, gpio_flags_t flags) {
    if (!device_is_ready(spec->port)) {
        LOG_ERR("GPIO device not ready: %s", spec->port->name);
        return;
    }
    gpio_pin_configure_dt(spec, flags);
    LOG_INF("Configured GPIO: %s, pin: %d", spec->port->name, spec->pin);
}

static void configure_adc(const struct adc_dt_spec *spec) {
    if (!device_is_ready(spec->dev)) {
        LOG_ERR("ADC device not ready");
        return;
    }
    int err = adc_channel_setup_dt(spec);
    if (err < 0) {
        LOG_ERR("Failed to setup ADC channel: %d", err);
    } else {
        LOG_INF("ADC channel configured successfully");
    }
}

// Heartbeat Thread
K_THREAD_DEFINE(heartbeat_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

void heartbeat_thread(void *a, void *b, void *c) {
    while (1) {
        gpio_pin_set_dt(&led0_spec, 1);
        // LOG_INF("Heartbeat LED ON");
        k_msleep(HEARTBEAT_ON_TIME);
        gpio_pin_set_dt(&led0_spec, 0);
        // LOG_INF("Heartbeat LED OFF");
        k_msleep(HEARTBEAT_OFF_TIME);
    }
}

// Timers
K_TIMER_DEFINE(blink_timer, blink_handler, NULL);
K_TIMER_DEFINE(blink_duration_timer, blink_duration_handler, NULL);
K_TIMER_DEFINE(led_on_timer, led_on_handler, NULL);
K_TIMER_DEFINE(led_off_timer, led_off_handler, NULL);

// Main Function
void main(void) {
    LOG_INF("System initialization started");
    smf_set_initial(SMF_CTX(&system_context), &states[INIT]);
    while (1) {
        smf_run_state(SMF_CTX(&system_context));
        k_msleep(10);
    }
}

// Callback Functions
void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_INF("Button pressed, transitioning to ADC_READ state");
    system_context.button0_pressed = true;
    smf_set_state(SMF_CTX(&system_context), &states[ADC_READ]);
}

// Timer Functions
void blink_handler(struct k_timer *timer) {
    static bool led_state = false;
    led_state = !led_state;
    gpio_pin_set_dt(&led1_spec, led_state);
    LOG_INF("LED1 state toggled: %s", led_state ? "ON" : "OFF");
}

void blink_duration_handler(struct k_timer *timer) {
    LOG_INF("Blink duration timer expired, stopping LED blinking");
    gpio_pin_set_dt(&led1_spec, 0); // Ensure LED is off
    k_timer_stop(&led_on_timer);
    k_timer_stop(&led_off_timer);
    smf_set_state(SMF_CTX(&system_context), &states[IDLE]); // Transition to IDLE state
}

void led_on_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led1_spec, 1); // Turn LED on
    LOG_INF("LED1 turned ON");
    uint32_t off_delay = system_context.led1_blink_period_ms * LED_DUTY_CYCLE / 100;
    k_timer_start(&led_off_timer, K_MSEC(off_delay), K_NO_WAIT); // Schedule LED off
}

void led_off_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led1_spec, 0); // Turn LED off
    LOG_INF("LED1 turned OFF");
}

// State Functions
void init_run(void *o) {
    LOG_INF("Entering INIT state");
    configure_gpio(&button0_spec, GPIO_INPUT);
    configure_gpio(&led0_spec, GPIO_OUTPUT);
    configure_gpio(&led1_spec, GPIO_OUTPUT);

    gpio_pin_set_dt(&led1_spec, 0);

    int err = gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Failed to configure button interrupt: %d", err);
    } else {
        LOG_INF("Button interrupt configured successfully");
    }
    gpio_init_callback(&button0_cb, button0_callback, BIT(button0_spec.pin));
    gpio_add_callback_dt(&button0_spec, &button0_cb);

    configure_adc(&adc_vadc_spec);
    LOG_INF("ADC configured successfully");

    LOG_INF("Exiting INIT state, transitioning to IDLE state");
    smf_set_state(SMF_CTX(&system_context), &states[IDLE]);
}

void idle_run(void *o) {
    // LOG_INF("System is in IDLE state");
}

void adc_read_run(void *o) {
    LOG_INF("Entering ADC_READ state");
    int16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };
    adc_sequence_init_dt(&adc_vadc_spec, &sequence);

    int ret = adc_read(adc_vadc_spec.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC read failed: %d", ret);
    } else {
        system_context.adc_voltage_mv = buf;
        adc_raw_to_millivolts_dt(&adc_vadc_spec, &system_context.adc_voltage_mv);
        LOG_INF("ADC Value (mV): %d", system_context.adc_voltage_mv);
        system_context.led1_blink_period_ms = 1000 - (system_context.adc_voltage_mv * 800 / 3000);
        LOG_INF("Calculated LED1 blink period: %d ms", system_context.led1_blink_period_ms);
    }
    system_context.button0_pressed = false;
    LOG_INF("Exiting ADC_READ state, transitioning to BLINKING state");
    smf_set_state(SMF_CTX(&system_context), &states[BLINKING]);
}

void blinking_entry(void *o) {
    LOG_INF("Entering BLINKING state");

    uint32_t blink_period = system_context.led1_blink_period_ms;
    uint32_t on_time = blink_period * LED_DUTY_CYCLE / 100;

    // Start the blink duration timer (5 seconds)
    k_timer_start(&blink_duration_timer, K_MSEC(LED_BLINK_DURATION_MS), K_NO_WAIT);

    // Start the LED on timer (repeats every blink period)
    k_timer_start(&led_on_timer, K_NO_WAIT, K_MSEC(blink_period));

    LOG_INF("Blinking started: period = %d ms, on-time = %d ms, duration = %d ms",
            blink_period, on_time, LED_BLINK_DURATION_MS);
}

void blinking_run(void *o) {
    // LOG_INF("System is in BLINKING state");
}