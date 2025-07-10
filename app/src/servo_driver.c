#include "../include/servo_driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// This function calculates the clock divider and wrap value to achieve a specified
// frequency while allowing for setting the pulse width in microseconds.
void servo_driver_init(uint8_t gpio_pin, uint_fast16_t frequency_hz) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio_pin);

    // Get the system clock frequency.
    uint32_t f_sys = clock_get_hz(clk_sys);

    // We want each PWM "tick" to be 1 microsecond.
    // The formula is: f_pwm = f_sys / ((wrap + 1) * clkdiv)
    // We choose clkdiv such that f_sys / clkdiv = 1,000,000 (1MHz).
    // This makes the calculations very simple.
    float clkdiv = (float)f_sys / 1000000.0f;

    // Now, calculate the wrap value (period) for the desired frequency.
    // wrap + 1 = 1,000,000 / frequency_hz
    uint32_t wrap = 1000000 / frequency_hz - 1;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_config_set_wrap(&config, wrap);
    
    // Enable after we set the level first
    pwm_init(slice, &config, false);
}

void servo_driver_set_pulse_us(uint8_t gpio_pin, uint16_t pulse_width_us) {
    uint slice = pwm_gpio_to_slice_num(gpio_pin);
    uint chan = pwm_gpio_to_channel(gpio_pin);

    // Since our clock is ticking every microsecond, the "level" value
    // is simply the desired pulse width in microseconds.
    pwm_set_chan_level(slice, chan, pulse_width_us);
}