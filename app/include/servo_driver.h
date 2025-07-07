#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

/**
 * @brief Initializes a GPIO pin for servo/ESC control with a specific frequency.
 * 
 * @param gpio_pin The Pico GPIO pin number to use.
 * @param frequency_hz The frequency for the PWM signal (e.g., 50 for standard ESCs).
 */
void servo_driver_init(uint8_t gpio_pin, uint_fast16_t frequency_hz);

/**
 * @brief Sets the output pulse width on a previously initialized pin.
 * 
 * @param gpio_pin The GPIO pin to set the pulse width on.
 * @param pulse_width_us The desired pulse width in microseconds (e.g., 1000, 1500, 2000).
 */
void servo_driver_set_pulse_us(uint8_t gpio_pin, uint16_t pulse_width_us);

#endif // SERVO_DRIVER_H