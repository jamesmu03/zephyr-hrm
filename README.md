# Lab: Zephyr ADC

## Fork and Clone the Repository

* Fork this repository to your own GitLab account.
* Add Dr. Palmeri and your TAs as `Maintainer`s to the GitLab repository.
* Use good git version control practices when working on this project.

## Part I: Single-Channel ADC Sampling

### Firmware Functional Specifications

* All firmware should be written using the **State Machine Framework**. 
* Include a state diagram in your repository (`state_diagram.png`) using UML (`state_diagram.puml`).  Include this diagram in your technical report (see below).
* Timers, work queues, callbacks, and interrupts should be used as appropriate.
* All good coding practices developed this semester should be followed.
* `LED` and `BUTTON` number references below are based on Devicetree labels (not the annotation on the DK board itself).
* There should be a heartbeat LED (`LED0`) that blinks at 1 Hz at all times when the firmware is running.
* When `BUTTON0` is pressed:
  * Make a measurement using the `AIN0` channel of the ADC.
    * Use the `ADC_REF_INTERNAL` reference voltage.
    * Linearly map 0-3.0 V measured on `AIN0` to an LED blink rate of 1-5 Hz (e.g., 0 V -> 1 Hz; 3.0 V -> 5 Hz).
  + Make a measurement using the `AIN1` channel of the ADC.
    * User the `ADC_REF_VDD_1_4` reference voltage.
    * Linearly map 0-3.0 V measuremed on `AIN1` to an LED ontime of 5-10 s (e.g., 0 V -> 5 s, 3.0 V -> 10 s)
  * Illuminate `LED1` at the blink rate and ontime duration specified by the ADC measurements in response to `BUTTON0` being pressed.

### Testing

For the following testing, you can use a power supply to provide known voltage inputs to `AIN0` and `AIN1`.

* Quantify how linear the relationship is between the voltage applied to `AIN0` and the `LED1` blinking frequency.  
  * Not sure how to quantify linearity?  Consider plotting the data and presenting the $R^2$ of a linear regression fit.
  * Remember that single data points for any single input voltage pair is not adequate; multiple measurements should be made and error bars presented on all plots.
* Quantify how linear the relationship is between the voltage applied to `AIN1` and the `LED1` ontime duration.
* Present all of these data, your analysis and your interpretation in a [testing/technical_report.ipynb](testing/technical_report.ipynb) Jupyter notebook.

## Part II: Buffered Differential ADC Sampling

### Firmware Functional Specifications

In a new development branch, add the following functionality to your firmware:

* Keep existing ADC inputs channels, but on the button press, add an additional differential ADC measurement using `AIN2` and `AIN4`.  *(Note - `AIN3` on the DK is mapped to `P0.05`, which is used for UART commication.)*
* Choose the reference voltage, gain, bit depth and acquisition time to adequately sample at least 20 cycles of a 500 Hz sinusoidal signal (Vpp = 2 V) with a 1 V DC offset (i.e., sinusoid voltage rage is 0-2 V).
* Implement the `extra_sampling` buffering of the ADC for this differential sinusoidal signal measurement so that all of the data is stored in an array in a single `adc_read()` call.
* Disable the acquisition button while your device is reading the analog voltages and performing calculations.
* Write a **library** called `calc_rms` that calculates the RMS value of the buffered ADC samples.
* Write log messages to your serial terminal that:
  * Display a hex array of the buffered ADC samples.  :warning: Be careful for the log message memory consumption!
  * Display the RMS value of the buffered ADC samples.
* Update your state diagram for this new functionality.

### Testing

* Input a 500 Hz sinusoidal signal with a 1 V DC offset into the differential ADC input.
* Create a plot of your input signal and the buffered ADC samples (using the hex array output).  Discuss any differences between the input signal and your sampled signal.
* Compare your calculated RMS value to the expected RMS value of the sinusoidal signal.  Discuss any differences between your calculated RMS value and the expected RMS value.
* Add this analysis to [testing/technical_report.ipynb](testing/technical_report.ipynb).

## Merge Your Development Branch and create a Merge Request

* Merge your branch for the buffered differential ADC sampling into your main branch.
* Create a Merge Request back to the parent repository `main` branch and assign it to Dr. Palmeri.