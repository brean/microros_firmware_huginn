#include "../include/esc_driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "pico/stdlib.h"

#include <std_msgs/msg/int16.h>
#include <std_srvs/srv/trigger.h>

const uint PWM_FREQ = 50;       // 50 Hz, standard for most ESCs

// Duty cycle values (16-bit) matching the Python script
// These correspond to ~1ms and ~2ms pulse widths at 50Hz
const uint16_t MIN_PULSE_US  = 1000; // Standard minimum throttle pulse (1ms)
const uint16_t MAX_PULSE_US  = 2000; // Standard maximum throttle pulse (2ms)

static uint8_t esc_pin;
static bool is_armed = false;

// Private ROS objects
static rcl_subscription_t throttle_subscriber;
static rcl_service_t calib_service;
static std_msgs__msg__Int16 throttle_msg;
static std_srvs__srv__Trigger_Request calib_req;
static std_srvs__srv__Trigger_Response calib_res;

static void set_motor_pulse(uint16_t pulse_us) {
    // Safety: Clamp the pulse width to the allowed range
    if (pulse_us < MIN_PULSE_US) {
        pulse_us = MIN_PULSE_US;
    }
    if (pulse_us > MAX_PULSE_US) {
        pulse_us = MAX_PULSE_US;
    }
    servo_driver_set_pulse_us(esc_pin, pulse_us);
}

static uint16_t map_speed_to_pulse(int16_t speed_percent) {
    // First, clamp the input to the valid 0-1000 range for safety.
    if (speed_percent < 0) {
        speed_percent = 0;
    }
    if (speed_percent > 1000) {
        speed_percent = 1000;
    }

    // Calculate the pulse width range.
    uint16_t pulse_range = MAX_PULSE_US - MIN_PULSE_US;

    // Map the percentage to the pulse range using integer math.
    uint16_t pulse_us = MIN_PULSE_US + (speed_percent * pulse_range) / 1000;

    return pulse_us;
}


// Private ROS Callbacks
static void esc_throttle_callback(const void *msin) {
    if (!is_armed) {
        return;
    }
    const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msin;
    int16_t speed_permil = msg->data;
    uint16_t pulse_us = map_speed_to_pulse(speed_permil);
    set_motor_pulse(pulse_us);
}



static void calib_callback(const void *req, void *res) {
    if (!is_armed) {
        return;
    }
    set_motor_pulse(MAX_PULSE_US);
    // --> Connect power to the ESC now. Wait for the initial beeps, then wait some more.
    // sleep 10 seconds
    sleep_ms(10000);

    set_motor_pulse(MIN_PULSE_US);
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;
    res_in->success = true;
    res_in->message.data = "ESC should be armed, you can restart the ESC now.";
    res_in->message.size = strlen(res_in->message.data);

    sleep_ms(5000);
}

// Public hardware init function
void esc_driver_init(uint8_t gpio_pin) {
esc_pin = gpio_pin;
    is_armed = false;    
    servo_driver_init(esc_pin, PWM_FREQ);
    // Set to 0 initially for a clean power-up sequence
    servo_driver_set_pulse_us(esc_pin, 0);
    
    // initiate with min. duty
    set_motor_pulse(MIN_PULSE_US);
    sleep_ms(2000); 
    is_armed = true;
}

// Public ROS init function
void esc_driver_init_ros(
        rcl_node_t *node,
        rclc_support_t *support,
        rclc_executor_t *executor) {
    rclc_subscription_init_default(
        &throttle_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "/pico/esc/throttle_command");
    rclc_service_init_default(
        &calib_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/pico/esc/calibrate");
    
    rclc_executor_add_subscription(
        executor,
        &throttle_subscriber,
        &throttle_msg,
        &esc_throttle_callback,
        ON_NEW_DATA);
    rclc_executor_add_service(
        executor,
        &calib_service,
        &calib_req,
        &calib_res,
        &calib_callback);
}
