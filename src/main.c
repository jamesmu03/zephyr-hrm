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
}

static void configure_adc(const struct adc_dt_spec *spec) {
    if (!device_is_ready(spec->dev)) {
        LOG_ERR("ADC device not ready");
        return;
    }
    int err = adc_channel_setup_dt(spec);
    if (err < 0) {
        LOG_ERR("Failed to setup ADC channel: %d", err);
    }
}

// Heartbeat Thread
K_THREAD_DEFINE(heartbeat_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

void heartbeat_thread(void *a, void *b, void *c) {
    while (1) {
        gpio_pin_set_dt(&led0_spec, 1);
        k_msleep(HEARTBEAT_ON_TIME);
        gpio_pin_set_dt(&led0_spec, 0);
        k_msleep(HEARTBEAT_OFF_TIME);
    }
}

// Timers
K_TIMER_DEFINE(blink_timer, blink_handler, NULL);

// Main Function
void main(void) {
    smf_set_initial(SMF_CTX(&system_context), &states[INIT]);
    while (1) {
        smf_run_state(SMF_CTX(&system_context));
        k_msleep(10);
    }
}

// Callback Functions
void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    system_context.button0_pressed = true;
    smf_set_state(SMF_CTX(&system_context), &states[ADC_READ]);
}

// Timer Functions
void blink_handler(struct k_timer *timer) {
    static bool led_state = false;
    led_state = !led_state;
    gpio_pin_set_dt(&led1_spec, led_state);
}

// State Functions
void init_run(void *o) {
    LOG_INF("Entering INIT state");
    configure_gpio(&button0_spec, GPIO_INPUT);
    configure_gpio(&led0_spec, GPIO_OUTPUT);
    configure_gpio(&led1_spec, GPIO_OUTPUT);

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
    // Do Nothing.
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
    uint32_t on_time = system_context.led1_blink_period_ms * LED_DUTY_CYCLE / 100;
    k_timer_start(&blink_timer, K_MSEC(on_time), K_MSEC(system_context.led1_blink_period_ms));
    LOG_INF("Blinking started, period: %d ms, on-time: %d ms", system_context.led1_blink_period_ms, on_time);
}

void blinking_run(void *o) {
    static uint64_t start_time = 0;
    if (start_time == 0) {
        start_time = k_uptime_get();
        LOG_INF("Blinking run started, uptime: %llu ms", start_time);
    }
    if (k_uptime_get() - start_time >= LED_BLINK_DURATION_MS) {
        LOG_INF("Blinking duration exceeded, stopping blink timer");
        k_timer_stop(&blink_timer);
        gpio_pin_set_dt(&led1_spec, 0);
        start_time = 0;
        LOG_INF("Exiting BLINKING state, transitioning to IDLE state");
        smf_set_state(SMF_CTX(&system_context), &states[IDLE]);
    }
}