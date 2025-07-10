#include "../include/pwm_driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include <std_msgs/msg/int32_multi_array.h>

#define PWM_GENERIC_FREQUENCY 50
#define CLOCK_SYS 150.0f

static rcl_subscription_t subscriber;
static std_msgs__msg__Int32MultiArray msg;

static void overwrite_freq(uint8_t gpio_pin, uint8_t wrap, float clkdiv) {
    // overwrite PWM frequency values
    uint slice = pwm_gpio_to_slice_num(gpio_pin);

    pwm_config config = pwm_get_default_config();
    pwm_init(slice, &config, false);
    // an alternative would be to call clock_get_hz(clk_sys), see esc-driver
    pwm_set_clkdiv(slice, clkdiv); // should be 150 on rpi pico 2
    pwm_set_wrap(slice, wrap); // defaults to 20000 on rpi pico
    pwm_set_enabled(slice, true);

}

static void set_duty_level(uint8_t gpio_pin, uint8_t level) {
    // TODO: error handling when either the level is too high
    uint slice = pwm_gpio_to_slice_num(gpio_pin);
    uint chan = pwm_gpio_to_channel(gpio_pin);
    pwm_set_chan_level(slice, chan, level);
}

// ROS callback to apply PWM values
static void pwm_set_duty_callback(const void *msin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msin;
    if (msg->data.size >= 2) {
        uint8_t pin = msg->data.data[0];
        uint8_t duty_level = msg->data.data[1];
        set_duty_level(pin, duty_level);
    }
}

// Public hardware init function
void pwm_driver_init(uint8_t gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
}

// Public ROS init function
void pwm_driver_init_ros(rcl_node_t *node, rclc_support_t *support, rclc_executor_t *executor) {
    rclc_subscription_init_default(
        &subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/pico/pwm/set_duty"
    );

    msg.data.capacity = 2;
    msg.data.data = (int32_t*) malloc(msg.data.capacity * sizeof(int32_t));
    msg.data.size = 0;

    rclc_executor_add_subscription(executor, &subscriber, &msg, &pwm_set_duty_callback, ON_NEW_DATA);
}