#include "microros_app.h"

#include "app_config.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor_control.h"
#include "relay_control.h"
#include "vesc_can_telemetry.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <agv_msgs/msg/wheel_commands.h>
#include <agv_msgs/msg/wheel_states.h>
#include <rcl/context.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
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

typedef enum {
    WAITING_AGENT = 0,
    CREATE_ENTITIES,
    RUNNING,
    DESTROY_ENTITIES
} microros_state_t;

typedef struct {
    bool support;
    bool node;
    bool wheel_command_subscriber;
    bool relay_subscriber;
    bool wheel_states_publisher;
    bool wheel_states_timer;
    bool executor;
} entity_flags_t;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t wheel_command_subscriber;
static rcl_subscription_t relay_subscriber;
static rcl_publisher_t wheel_states_publisher;
static rcl_timer_t wheel_states_timer;
static rclc_executor_t executor;
static entity_flags_t entity_flags;

static agv_msgs__msg__WheelCommands wheel_command_message;
static agv_msgs__msg__WheelStates wheel_states_message;
static std_msgs__msg__Bool relay_command_message;

static bool time_synchronized;
static bool ever_connected;
static volatile uint32_t agent_reconnect_count;
static volatile uint32_t publish_error_count;
static uint32_t consecutive_publish_errors;

static void fatal_error(void)
{
    motor_control_set_agent_connected(false);
    for (;;) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        osDelay(500U);
    }
}

static bool configure_freertos_allocator(void)
{
    rcl_allocator_t freertos_allocator =
        rcutils_get_zero_initialized_allocator();
    freertos_allocator.allocate = microros_allocate;
    freertos_allocator.deallocate = microros_deallocate;
    freertos_allocator.reallocate = microros_reallocate;
    freertos_allocator.zero_allocate = microros_zero_allocate;
    return rcutils_set_default_allocator(&freertos_allocator);
}

static void wheel_command_callback(const void *message_input)
{
    const agv_msgs__msg__WheelCommands *message =
        (const agv_msgs__msg__WheelCommands *)message_input;
    float command[WHEEL_COUNT] = {0};
    bool valid = message != NULL;
    uint32_t wheel;

    if (message != NULL) {
        for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
            command[wheel] = message->velocity_rad_s[wheel];
            if (!isfinite(command[wheel])) {
                valid = false;
            }
        }
    }
    motor_control_set_wheel_command(command, valid);
}

static void relay_command_callback(const void *message_input)
{
    const std_msgs__msg__Bool *message =
        (const std_msgs__msg__Bool *)message_input;

    if (message == NULL) {
        return;
    }

    relay_set_enabled(message->data);
    motor_control_invalidate_command();
}

static void update_header_stamp(void)
{
    int64_t epoch_ms = 0;

    if (time_synchronized) {
        epoch_ms = rmw_uros_epoch_millis();
    }
    if (epoch_ms > 0) {
        wheel_states_message.header.stamp.sec =
            (int32_t)(epoch_ms / 1000LL);
        wheel_states_message.header.stamp.nanosec =
            (uint32_t)((epoch_ms % 1000LL) * 1000000LL);
    } else {
        wheel_states_message.header.stamp.sec = 0;
        wheel_states_message.header.stamp.nanosec = 0U;
    }
}

static void wheel_states_timer_callback(
    rcl_timer_t *timer,
    int64_t last_call_time)
{
    vesc_telemetry_snapshot_t snapshot;
    uint32_t wheel;

    (void)last_call_time;
    if ((timer == NULL) || !entity_flags.wheel_states_publisher) {
        return;
    }

    vesc_can_telemetry_get_snapshot(&snapshot);
    update_header_stamp();
    for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
        wheel_states_message.velocity_rad_s[wheel] =
            snapshot.velocity_rad_s[wheel];
        wheel_states_message.erpm[wheel] = snapshot.erpm[wheel];
        wheel_states_message.tachometer_counts[wheel] =
            snapshot.tachometer_counts[wheel];
        wheel_states_message.online[wheel] = snapshot.online[wheel];
    }

    if (rcl_publish(
            &wheel_states_publisher,
            &wheel_states_message,
            NULL) == RCL_RET_OK) {
        consecutive_publish_errors = 0U;
    } else {
        ++publish_error_count;
        ++consecutive_publish_errors;
    }
}

static void reset_entity_handles(void)
{
    support = (rclc_support_t){0};
    node = rcl_get_zero_initialized_node();
    wheel_command_subscriber =
        rcl_get_zero_initialized_subscription();
    relay_subscriber = rcl_get_zero_initialized_subscription();
    wheel_states_publisher = rcl_get_zero_initialized_publisher();
    wheel_states_timer = rcl_get_zero_initialized_timer();
    executor = rclc_executor_get_zero_initialized_executor();
    memset(&entity_flags, 0, sizeof(entity_flags));
}

static bool create_entities(void)
{
    reset_entity_handles();
    consecutive_publish_errors = 0U;

    if (rclc_support_init(
            &support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    entity_flags.support = true;

    if (rclc_node_init_default(
            &node, "base_controller", "", &support) != RCL_RET_OK) {
        return false;
    }
    entity_flags.node = true;

    if (rclc_subscription_init_best_effort(
            &wheel_command_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                agv_msgs, msg, WheelCommands),
            "/wheel_commands") != RCL_RET_OK) {
        return false;
    }
    entity_flags.wheel_command_subscriber = true;

    if (rclc_subscription_init_default(
            &relay_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/relay_cmd") != RCL_RET_OK) {
        return false;
    }
    entity_flags.relay_subscriber = true;

    if (rclc_publisher_init_best_effort(
            &wheel_states_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                agv_msgs, msg, WheelStates),
            "/wheel_states") != RCL_RET_OK) {
        return false;
    }
    entity_flags.wheel_states_publisher = true;

    if (rclc_timer_init_default(
            &wheel_states_timer,
            &support,
            RCL_MS_TO_NS(WHEEL_STATE_PUBLISH_PERIOD_MS),
            wheel_states_timer_callback) != RCL_RET_OK) {
        return false;
    }
    entity_flags.wheel_states_timer = true;

    if (rclc_executor_init(
            &executor,
            &support.context,
            3U,
            &allocator) != RCL_RET_OK) {
        return false;
    }
    entity_flags.executor = true;

    if (rclc_executor_add_subscription(
            &executor,
            &wheel_command_subscriber,
            &wheel_command_message,
            wheel_command_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_subscription(
            &executor,
            &relay_subscriber,
            &relay_command_message,
            relay_command_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_timer(
            &executor, &wheel_states_timer) != RCL_RET_OK) {
        return false;
    }

    time_synchronized =
        rmw_uros_sync_session(1000) == RMW_RET_OK;
    return true;
}

static void destroy_entities(void)
{
    if (entity_flags.support) {
        rmw_context_t *rmw_context =
            rcl_context_get_rmw_context(&support.context);
        if (rmw_context != NULL) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(
                rmw_context, 0);
        }
    }

    if (entity_flags.wheel_states_timer) {
        (void)rcl_timer_fini(&wheel_states_timer);
        entity_flags.wheel_states_timer = false;
    }
    if (entity_flags.executor) {
        (void)rclc_executor_fini(&executor);
        entity_flags.executor = false;
    }
    if (entity_flags.relay_subscriber) {
        (void)rcl_subscription_fini(&relay_subscriber, &node);
        entity_flags.relay_subscriber = false;
    }
    if (entity_flags.wheel_command_subscriber) {
        (void)rcl_subscription_fini(
            &wheel_command_subscriber, &node);
        entity_flags.wheel_command_subscriber = false;
    }
    if (entity_flags.wheel_states_publisher) {
        (void)rcl_publisher_fini(&wheel_states_publisher, &node);
        entity_flags.wheel_states_publisher = false;
    }
    if (entity_flags.node) {
        (void)rcl_node_fini(&node);
        entity_flags.node = false;
    }
    if (entity_flags.support) {
        (void)rclc_support_fini(&support);
        entity_flags.support = false;
    }

    time_synchronized = false;
    consecutive_publish_errors = 0U;
    reset_entity_handles();
}

void microros_app_run(void)
{
    microros_state_t state = WAITING_AGENT;
    uint32_t next_ping_ms = 0U;
    uint32_t consecutive_executor_errors = 0U;

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    motor_control_set_agent_connected(false);

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
    allocator = rcl_get_default_allocator();

    if (!agv_msgs__msg__WheelCommands__init(
            &wheel_command_message) ||
        !agv_msgs__msg__WheelStates__init(
            &wheel_states_message) ||
        !std_msgs__msg__Bool__init(&relay_command_message)) {
        fatal_error();
    }
    reset_entity_handles();

    for (;;) {
        uint32_t now_ms = HAL_GetTick();

        switch (state) {
        case WAITING_AGENT:
            motor_control_set_agent_connected(false);
            if ((int32_t)(now_ms - next_ping_ms) >= 0) {
                if (rmw_uros_ping_agent(
                        AGENT_PING_TIMEOUT_MS, 1U) == RMW_RET_OK) {
                    state = CREATE_ENTITIES;
                } else {
                    next_ping_ms = now_ms + AGENT_WAIT_RETRY_MS;
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                }
            }
            break;

        case CREATE_ENTITIES:
            motor_control_set_agent_connected(false);
            if (create_entities()) {
                if (ever_connected) {
                    ++agent_reconnect_count;
                }
                ever_connected = true;
                motor_control_set_agent_connected(true);
                consecutive_executor_errors = 0U;
                next_ping_ms = now_ms + AGENT_PING_PERIOD_MS;
                HAL_GPIO_WritePin(
                    LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                state = RUNNING;
            } else {
                state = DESTROY_ENTITIES;
            }
            break;

        case RUNNING:
        {
            rcl_ret_t result = rclc_executor_spin_some(
                &executor, RCL_MS_TO_NS(10U));
            if ((result == RCL_RET_OK) ||
                (result == RCL_RET_TIMEOUT)) {
                consecutive_executor_errors = 0U;
            } else {
                ++consecutive_executor_errors;
            }

            if ((consecutive_executor_errors >= 3U) ||
                (consecutive_publish_errors >= 3U)) {
                motor_control_set_agent_connected(false);
                state = DESTROY_ENTITIES;
                break;
            }

            now_ms = HAL_GetTick();
            if ((int32_t)(now_ms - next_ping_ms) >= 0) {
                next_ping_ms = now_ms + AGENT_PING_PERIOD_MS;
                if (rmw_uros_ping_agent(
                        AGENT_PING_TIMEOUT_MS, 1U) != RMW_RET_OK) {
                    motor_control_set_agent_connected(false);
                    state = DESTROY_ENTITIES;
                }
            }
            break;
        }

        case DESTROY_ENTITIES:
            motor_control_set_agent_connected(false);
            destroy_entities();
            HAL_GPIO_WritePin(
                LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            next_ping_ms = HAL_GetTick() + AGENT_WAIT_RETRY_MS;
            state = WAITING_AGENT;
            break;

        default:
            state = DESTROY_ENTITIES;
            break;
        }

        osDelay(5U);
    }
}
