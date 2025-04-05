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
#define SAMPLING_DURATION_US 2000000
#define INTERVAL_US 5000
#define DIFF_SAMPLES (SAMPLING_DURATION_US / INTERVAL_US)

// GPIO PWM ADC Specifications
static const struct gpio_dt_spec led0_spec = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1_spec = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec button0_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button1_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec button2_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct pwm_dt_spec led2_pwm_spec = PWM_DT_SPEC_GET(DT_ALIAS(pwm1));
static const struct pwm_dt_spec led3_pwm_spec = PWM_DT_SPEC_GET(DT_ALIAS(pwm2));
static struct gpio_callback button0_cb;
static struct gpio_callback button1_cb;
static struct gpio_callback button2_cb;

static const struct adc_dt_spec adc_vadc_spec = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_vadc_diff_spec = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);

// Function Prototypes
void heartbeat_thread(void *a, void *b, void *c);
void blink_handler(struct k_timer *timer);
void init_run(void *o);
void idle_run(void *o);
void adc_read_run(void *o);
void diff_adc_read_run(void *o);
void diff_process_run(void *o);
void blinking_entry(void *o);
void blinking_run(void *o);
void sinusoidal_modulation_run(void *o);
void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void blink_duration_handler(struct k_timer *timer);
void led_on_handler(struct k_timer *timer);
void led_off_handler(struct k_timer *timer);

// System Context
struct system_context {
    struct smf_ctx ctx;
    bool button0_pressed;
    bool button1_pressed;
    bool modulation_active;
    int32_t adc_voltage_mv;
    uint32_t led1_blink_period_ms;
    int16_t diff_buffer[DIFF_SAMPLES];
    int32_t diff_cycles;
} system_context;

// State Machine
enum state { INIT, IDLE, ADC_READ, DIFF_ADC_READ, DIFF_PROCESS, BLINKING, SINUSOIDAL_MODULATION };
static const struct smf_state states[] = {
    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(NULL, idle_run, NULL, NULL, NULL),
    [ADC_READ] = SMF_CREATE_STATE(NULL, adc_read_run, NULL, NULL, NULL),
    [DIFF_ADC_READ] = SMF_CREATE_STATE(NULL, diff_adc_read_run, NULL, NULL, NULL),
    [DIFF_PROCESS] = SMF_CREATE_STATE(NULL, diff_process_run, NULL, NULL, NULL),
    [BLINKING] = SMF_CREATE_STATE(blinking_entry, blinking_run, NULL, NULL, NULL),
    [SINUSOIDAL_MODULATION] = SMF_CREATE_STATE(NULL, sinusoidal_modulation_run, NULL, NULL, NULL),
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
        k_msleep(HEARTBEAT_ON_TIME);
        gpio_pin_set_dt(&led0_spec, 0);
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
        k_msleep(1);
    }
}

// Callback Functions
void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (system_context.button0_pressed) {
        LOG_INF("Button0 press ignored, already being processed");
        return;
    }
    LOG_INF("Button0 pressed, transitioning to ADC_READ state");
    system_context.button0_pressed = true;
    smf_set_state(SMF_CTX(&system_context), &states[ADC_READ]);
}

void button1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (system_context.button1_pressed) {
        LOG_INF("Button1 press ignored, already being processed");
        return;
    }
    LOG_INF("Button1 pressed, transitioning to DIFF_ADC_READ state");
    system_context.button1_pressed = true;
    smf_set_state(SMF_CTX(&system_context), &states[DIFF_ADC_READ]);
}

void button2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (system_context.modulation_active) {
        LOG_INF("Button2 pressed, stopping sinusoidal modulation");
        system_context.modulation_active = false;
    } else {
        LOG_INF("Button2 pressed, starting sinusoidal modulation");
        system_context.modulation_active = true;
        smf_set_state(SMF_CTX(&system_context), &states[SINUSOIDAL_MODULATION]);
    }
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
    gpio_pin_set_dt(&led1_spec, 0);
    k_timer_stop(&led_on_timer);
    k_timer_stop(&led_off_timer);
    system_context.button0_pressed = false;
    smf_set_state(SMF_CTX(&system_context), &states[IDLE]);
}

void led_on_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led1_spec, 1);
    LOG_INF("LED1 turned ON");
    uint32_t off_delay = system_context.led1_blink_period_ms * LED_DUTY_CYCLE / 100;
    k_timer_start(&led_off_timer, K_MSEC(off_delay), K_NO_WAIT);
}

void led_off_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led1_spec, 0);
    LOG_INF("LED1 turned OFF");
}

// State Functions
void init_run(void *o) {
    LOG_INF("Entering INIT state");
    configure_gpio(&button0_spec, GPIO_INPUT);
    configure_gpio(&button1_spec, GPIO_INPUT);
    configure_gpio(&button2_spec, GPIO_INPUT);
    configure_gpio(&led0_spec, GPIO_OUTPUT);
    configure_gpio(&led1_spec, GPIO_OUTPUT);

    gpio_pin_set_dt(&led1_spec, 0);

    int err = gpio_pin_interrupt_configure_dt(&button0_spec, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Failed to configure button0 interrupt: %d", err);
    } else {
        LOG_INF("Button0 interrupt configured successfully");
    }
    gpio_init_callback(&button0_cb, button0_callback, BIT(button0_spec.pin));
    gpio_add_callback_dt(&button0_spec, &button0_cb);

    err = gpio_pin_interrupt_configure_dt(&button1_spec, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Failed to configure button1 interrupt: %d", err);
    } else {
        LOG_INF("Button1 interrupt configured successfully");
    }
    gpio_init_callback(&button1_cb, button1_callback, BIT(button1_spec.pin));
    gpio_add_callback_dt(&button1_spec, &button1_cb);

    err = gpio_pin_interrupt_configure_dt(&button2_spec, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Failed to configure button2 interrupt: %d", err);
    } else {
        LOG_INF("Button2 interrupt configured successfully");
    }
    gpio_init_callback(&button2_cb, button2_callback, BIT(button2_spec.pin));
    gpio_add_callback_dt(&button2_spec, &button2_cb);

    configure_adc(&adc_vadc_spec);
    configure_adc(&adc_vadc_diff_spec);
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
        return;
    }

    system_context.adc_voltage_mv = buf;
    adc_raw_to_millivolts_dt(&adc_vadc_spec, &system_context.adc_voltage_mv);
    LOG_INF("ADC Value (mV): %d", system_context.adc_voltage_mv);

    uint32_t duty_cycle = CLAMP((system_context.adc_voltage_mv * 100) / 3000, 0, 100);
    LOG_INF("Mapped PWM duty cycle: %d%%", duty_cycle);

    ret = pwm_set_dt(&led2_pwm_spec, PWM_USEC(1000), PWM_USEC(1000 - (duty_cycle * 10)));
    if (ret < 0) {
        LOG_ERR("Failed to set PWM duty cycle: %d", ret);
    } else {
        LOG_INF("PWM duty cycle set successfully");
    }

    system_context.led1_blink_period_ms = 1000 - (system_context.adc_voltage_mv * 800 / 3000);
    LOG_INF("Calculated LED1 blink period: %d ms", system_context.led1_blink_period_ms);

    LOG_INF("Exiting ADC_READ state, transitioning to BLINKING state");
    smf_set_state(SMF_CTX(&system_context), &states[BLINKING]);
}

void diff_adc_read_run(void *o) {
    LOG_INF("Entering DIFF_ADC_READ state");
    gpio_pin_interrupt_configure_dt(&button1_spec, GPIO_INT_DISABLE);

    struct adc_sequence_options options = {
        .extra_samplings = DIFF_SAMPLES - 1,
        .interval_us = INTERVAL_US,
    };
    struct adc_sequence sequence = {
        .options = &options,
        .buffer = system_context.diff_buffer,
        .buffer_size = sizeof(system_context.diff_buffer),
    };
    adc_sequence_init_dt(&adc_vadc_diff_spec, &sequence);

    int ret = adc_read(adc_vadc_diff_spec.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("Differential ADC read failed: %d", ret);
    } else {
        LOG_INF("Differential ADC sampling complete, %d samples", DIFF_SAMPLES);
    }

    gpio_pin_interrupt_configure_dt(&button1_spec, GPIO_INT_EDGE_TO_ACTIVE);
    smf_set_state(SMF_CTX(&system_context), &states[DIFF_PROCESS]);
}

void diff_process_run(void *o) {
    LOG_INF("Entering DIFF_PROCESS state");
    LOG_HEXDUMP_INF(system_context.diff_buffer, sizeof(system_context.diff_buffer), "Raw ADC Buffer:");

    system_context.diff_cycles = calculate_cycles(system_context.diff_buffer, DIFF_SAMPLES);
    LOG_INF("Calculated cycles: %d", system_context.diff_cycles);

    system_context.button1_pressed = false;
    smf_set_state(SMF_CTX(&system_context), &states[IDLE]);
}

void blinking_entry(void *o) {
    LOG_INF("Entering BLINKING state");
    uint32_t blink_period = system_context.led1_blink_period_ms;
    uint32_t on_time = blink_period * LED_DUTY_CYCLE / 100;

    k_timer_start(&blink_duration_timer, K_MSEC(LED_BLINK_DURATION_MS), K_NO_WAIT);
    k_timer_start(&led_on_timer, K_NO_WAIT, K_MSEC(blink_period));

    LOG_INF("Blinking started: period = %d ms, on-time = %d ms, duration = %d ms",
            blink_period, on_time, LED_BLINK_DURATION_MS);
}

void blinking_run(void *o) {
    // LOG_INF("System is in BLINKING state");
}

void sinusoidal_modulation_run(void *o) {
    LOG_INF("Entering SINUSOIDAL_MODULATION state");

    static int16_t adc_value;
    struct adc_sequence sequence = {
        .buffer = &adc_value,
        .buffer_size = sizeof(adc_value),
    };

    adc_sequence_init_dt(&adc_vadc_diff_spec, &sequence);

    while (system_context.modulation_active) {
        int ret = adc_read(adc_vadc_diff_spec.dev, &sequence);
        if (ret < 0) {
            LOG_ERR("ADC read failed: %d", ret);
            break;
        }

        int32_t voltage_mv = adc_value;
        adc_raw_to_millivolts_dt(&adc_vadc_diff_spec, &voltage_mv);

        uint32_t duty_cycle = CLAMP((voltage_mv * 100) / 3000, 0, 100);
        LOG_INF("Mapped PWM duty cycle for LED3: %d%%", duty_cycle);

        ret = pwm_set_dt(&led3_pwm_spec, PWM_USEC(1000), PWM_USEC(1000 - (duty_cycle * 10)));
        if (ret < 0) {
            LOG_ERR("Failed to set PWM duty cycle for LED3: %d", ret);
        }

        k_msleep(5);
    }

    LOG_INF("Exiting SINUSOIDAL_MODULATION state");
    smf_set_state(SMF_CTX(&system_context), &states[IDLE]);
}