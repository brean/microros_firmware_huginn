#include "../include/esc_driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"

#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/trigger.h>

const uint16_t ESC_MIN_PULSE = 1000;
const uint16_t ESC_MAX_PULSE = 2000;
const uint16_t PWM_ESC_FREQUENCY = 50;

// Private hardware variables
static uint esc_slice;
static uint esc_chan;
static uint32_t esc_wrap;

// Private ROS objects
static rcl_subscription_t throttle_subscriber;
static rcl_service_t calib_start_service;
static rcl_service_t calib_finish_service;
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
    set_pulse_width_us(pulse_width);
}

// Private ROS Callbacks
static void esc_throttle_callback(const void *msin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msin;
    set_throttle(msg->data);
}

static void calib_start_callback(const void *req, void *res) {
    set_pulse_width_us(ESC_MAX_PULSE);
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;
    res_in->success = true;
    res_in->message.data = "MAX throttle set. Power cycle ESC, wait for beeps, then call finish service.";
    res_in->message.size = strlen(res_in->message.data);
}

static void calib_finish_callback(const void *req, void *res) {
    set_pulse_width_us(ESC_MIN_PULSE);
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;
    res_in->success = true;
    res_in->message.data = "MIN throttle set. ESC should be armed.";
    res_in->message.size = strlen(res_in->message.data);
}

// Public hardware init function
void esc_driver_init(uint8_t gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    esc_slice = pwm_gpio_to_slice_num(gpio_pin);
    esc_chan = pwm_gpio_to_channel(gpio_pin);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint32_t clk_div = clock_get_hz(clk_peri) / 1000000;
    pwm_set_clkdiv_int_frac(esc_slice, clk_div, 0);
    esc_wrap = 1000000 / PWM_ESC_FREQUENCY;
    pwm_set_wrap(esc_slice, esc_wrap);
    pwm_set_enabled(esc_slice, true);
    set_pulse_width_us(ESC_MIN_PULSE);
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
        &calib_start_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/pico/esc/calibrate_start");
    rclc_service_init_default(
        &calib_finish_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/pico/esc/calibrate_finish");
    
    rclc_executor_add_subscription(
        executor,
        &throttle_subscriber,
        &throttle_msg,
        &esc_throttle_callback,
        ON_NEW_DATA);
    rclc_executor_add_service(
        executor,
        &calib_start_service,
        &calib_req,
        &calib_res,
        &calib_start_callback);
    rclc_executor_add_service(
        executor,
        &calib_finish_service,
        &calib_req,
        &calib_res,
        &calib_finish_callback);
}