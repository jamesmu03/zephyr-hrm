# Lab: Zephyr ADC & PWM

The code in this repository has been built against `v2.6.2` of the Zephyr SDK.  You should use this version of the SDK when building your code.  :warning: If you use `v2.7.0` or later, you may encounter issues with some of the SMF macros and input syntax that has changed.

## Fork and Clone the Repository

* Fork this repository to your own GitLab account.
* Add Dr. Palmeri (`mlp6`) and your TAs (`hys3`, `cls157`, `bjl40`) as `Maintainers` to your GitLab repository.
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
    * Linearly map 0-3.0 V measured on `AIN0` to an `LED1` blink rate of 1-5 Hz (e.g., 0 V -> 1 Hz; 3.0 V -> 5 Hz).
  + Make a measurement using the `AIN1` channel of the ADC.
    * User the `ADC_REF_VDD_1_4` reference voltage.
    * Linearly map 0-3.0 V measuremed on `AIN1` to an `LED1` ontime of 5-10 s (e.g., 0 V -> 5 s, 3.0 V -> 10 s)
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
* Create a Merge Request back to the `main` branch **in your repository** (not the parent repository that you forked from)
  * Assign this Merge Request to yourself
  * Assign Dr. Palmeri as a `Reviewer`.
  * Merge your Merge Request once Dr. Palmeri has reviewed your code.

## Part III: Steady-State PWM Output

### Firmware Functional Specifications

* Using the RMS value that you calculate from your differential ADC input in the previous lab, map this RMS value to scale the maximum brightness of `LED2` using a PWM output.  
  * For example: 
    * If `Vpp` of the sinusoidal input is 0 V, then `LED2` should be off (0% duty cycle).  
    * If `Vpp` of the sinusoidal input is 2 V, then `LED2` should be at maximum brightness (100% duty cycle).  
  * Intermediate `Vpp` values should map to intermediate brightnesses by linearly scaling the duty cycle of the PWM output.
* Update your state diagram to include the new functionality.

### Testing

* Quantify the linearity of the maximum brightness of `LED2` as a function of the calculated RMS of the differential input voltage.
* Present your data and analysis in the technical report Jupyter notebook.

## Part IV: Sinusoidal Modulation of PWM Output

### Firmware Functional Specifications

* Create a branch from your branch for Part III to implement this Part IV functionality.
* Instead of holding the `LED2` brightness constant, modululate the brighness as a sinusoid at the same frequency as `LED1` from Part II, but 180 degrees out of phase.
* Set the maximum sinusoidal brightness to match that of the calculated RMS value.
* Update your state diagram to include the new functionality.
* Merge this branch back into your Part III branch once it is working (as verified by the testing below).

### Testing

* Calculate the frequenecy of the sinusoidal output (mean +/- 95% CI) relative to the nominal frequency.
* Calculate the linearity of the relationship between the RMS value of the differential input voltage and the maximum brightness of `LED2` when modulated as a sinusoid.
* Present your data and analysis in your Jupyter notebook.

## Discussion

Include discussion in your technical report about how your PWM outputs for maximum brightness (and sinusoidal fade) compare to the nominal values/waveforms.

## What to Submit

* Merge your Part III branch, which should include your Part IV development branch, into the `main` branch in your repository.
* Create an annotated tag of your `main` branch with all part of this labe merged in called `ready_for_grading`.
* Dr. Palmeri will clone all of your forks and grade them based on the `ready_for_grading` tag.
