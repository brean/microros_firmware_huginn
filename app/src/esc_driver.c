#include "../include/esc_driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "pico/stdlib.h"

#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/trigger.h>

const uint16_t ESC_MIN_PULSE = 3277;
const uint16_t ESC_MAX_PULSE = 6554;
const uint16_t PWM_ESC_FREQUENCY = 50;

bool enabled = false;

// Private hardware variables
static uint esc_slice;
static uint esc_chan;
static uint32_t esc_wrap;

// Private ROS objects
static rcl_subscription_t throttle_subscriber;
static rcl_service_t calib_service;
static std_msgs__msg__Int32 throttle_msg;
static std_srvs__srv__Trigger_Request calib_req;
static std_srvs__srv__Trigger_Response calib_res;

// Private hardware control functions
static void set_pulse_width_us(uint32_t us) {
    if (us < ESC_MIN_PULSE) us = ESC_MIN_PULSE;
    if (us > ESC_MAX_PULSE) us = ESC_MAX_PULSE;
    
    uint32_t level = (us * esc_wrap) / (1000000 / PWM_ESC_FREQUENCY);
    pwm_set_chan_level(esc_slice, esc_chan, level);
}

static void set_throttle(uint16_t pulse_width) {
    if (pulse_width < ESC_MIN_PULSE) {
        pulse_width = ESC_MAX_PULSE;
    }
    if (pulse_width > ESC_MAX_PULSE) {
        pulse_width = ESC_MAX_PULSE;
    }
    if (!enabled) {
        pwm_set_enabled(esc_slice, true);
        enabled = true;
    }
    set_pulse_width_us(pulse_width);
}

// Private ROS Callbacks
static void esc_throttle_callback(const void *msin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msin;
    set_throttle(msg->data);
}

static void calib_callback(const void *req, void *res) {
    if (!enabled) {
        pwm_set_enabled(esc_slice, true);
        enabled = true;
    }
    pwm_set_enabled(esc_slice, true);
    set_pulse_width_us(ESC_MAX_PULSE);
    // --> Connect power to the ESC now. Wait for the initial beeps, then wait some more.
    // sleep 10 seconds
    sleep_ms(10000);

    set_pulse_width_us(ESC_MIN_PULSE);
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;
    res_in->success = true;
    res_in->message.data = "ESC should be armed.";
    res_in->message.size = strlen(res_in->message.data);

    // disable PWM shortly.
    pwm_set_enabled(esc_slice, false);
    enabled = false;
}

// Public hardware init function
void esc_driver_init(uint8_t gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    esc_slice = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_enabled(esc_slice, true);
    // send min. pulse for at least 2 seconds in the beginning to arm the ESC.
    set_pulse_width_us(ESC_MIN_PULSE);
    sleep_ms(2000);
}

// Public ROS init function
void esc_driver_init_ros(
        rcl_node_t *node,
        rclc_support_t *support,
        rclc_executor_t *executor) {
    rclc_subscription_init_default(
        &throttle_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
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
