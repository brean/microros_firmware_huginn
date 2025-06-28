#ifndef ESC_DRIVER_H
#define ESC_DRIVER_H

#include <rclc/rclc.h>
#include <rclc/executor.h>

void esc_driver_init(uint8_t gpio_pin);
void esc_driver_init_ros(rcl_node_t *node, rclc_support_t *support, rclc_executor_t *executor);

#endif // ESC_DRIVER_H