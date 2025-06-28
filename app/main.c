#define PIN_ESC 15
#define PIN_SERVO 17

#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "include/esc_driver.h"
#include "include/pwm_driver.h"


int main()
{
    esc_driver_init(PIN_ESC);
    pwm_driver_init(PIN_SERVO);

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );


    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    // 0 timer
    // 2 services
    // 2 subscriber
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    
    esc_driver_init_ros(&node, &support, &executor);
    pwm_driver_init_ros(&node, &support, &executor);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
