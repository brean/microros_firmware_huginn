#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <rclc/rclc.h>
#include <rclc/executor.h>

void pwm_driver_init(uint8_t gpio_pin);
void pwm_driver_init_ros(rcl_node_t *node, rclc_support_t *support, rclc_executor_t *executor);

#endif // PWM_DRIVER_H