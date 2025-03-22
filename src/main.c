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

// define macros
#define HEARTBEAT_PERIOD_MS 1000
#define HEARTBEAT_DUTY_CYCLE 25
#define HEARTBEAT_ON_TIME (HEARTBEAT_PERIOD_MS * HEARTBEAT_DUTY_CYCLE / 100)
#define HEARTBEAT_OFF_TIME (HEARTBEAT_PERIOD_MS - HEARTBEAT_ON_TIME)

// declare function prototypes
void heartbeat_thread(void);

// define globals, DT-based hardware structs, and user structs
const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);

// define callback functions

// initialize GPIO Callback Structs

// define states for state machine

// heartbeat thread
#define HEARTBEAT_THREAD_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIORITY 5

K_THREAD_DEFINE(heartbeat_thread_id, HEARTBEAT_THREAD_STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, HEARTBEAT_THREAD_PRIORITY, 0, 0);

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
}

// main function
int main(void)
{
    LOG_INF("Starting main function");
    setup_gpio();
    k_thread_start(heartbeat_thread_id);
    LOG_INF("Heartbeat thread started");

    while (1) {
        // do stuff using state machine framework
        LOG_INF("Main loop iteration");
        k_sleep(K_MSEC(1000));
    }
}

// define functions