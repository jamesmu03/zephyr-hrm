#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>  // CONFIG_GPIO=y
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/smf.h> // CONFIG_SMF=y

#include "calc_rms.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// define macros 

// declare function prototypes

// define globals, DT-based hardware structs, and user structs

// define callback functions

// initialize GPIO Callback Structs

// define states for state machine

int main(void)
{

    while (1) {

        // do stuff using state machine framework

    }
}

// define functions