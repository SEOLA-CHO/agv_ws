#include "microros_app.h"

#include "cmsis_os.h"
#include "main.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <uxr/client/transport.h>

extern UART_HandleTypeDef huart2;

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error_code);
size_t cubemx_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout_ms,
    uint8_t *error_code);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(
    size_t number_of_elements,
    size_t size_of_element,
    void *state);

static void fatal_error(void)
{
    for (;;) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        osDelay(500U);
    }
}

static bool configure_freertos_allocator(void)
{
    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate;
    allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;
    return rcutils_set_default_allocator(&allocator);
}

static void cmd_vel_callback(const void *message_input)
{
    const geometry_msgs__msg__Twist *message =
        (const geometry_msgs__msg__Twist *)message_input;

    if (message != NULL) {
        motor_control_set_cmd_vel(
            (float)message->linear.x,
            (float)message->linear.y,
            (float)message->angular.z);
    }
}

void microros_app_run(void)
{
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    rmw_uros_set_custom_transport(
        true,
        (void *)&huart2,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    if (!configure_freertos_allocator()) {
        fatal_error();
    }

    while (rmw_uros_ping_agent(100U, 1U) != RMW_RET_OK) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        osDelay(100U);
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    geometry_msgs__msg__Twist message;

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        fatal_error();
    }
    if (rclc_node_init_default(
            &node, "nucleo_f446re", "", &support) != RCL_RET_OK) {
        fatal_error();
    }
    if (rclc_subscription_init_best_effort(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel") != RCL_RET_OK) {
        fatal_error();
    }
    if (!geometry_msgs__msg__Twist__init(&message)) {
        fatal_error();
    }
    if (rclc_executor_init(&executor, &support.context, 1U, &allocator) !=
        RCL_RET_OK) {
        fatal_error();
    }
    if (rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &message,
            cmd_vel_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        fatal_error();
    }

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    for (;;) {
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10U));
        osDelay(1U);
    }
}
