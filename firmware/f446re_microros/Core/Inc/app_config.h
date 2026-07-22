#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_COUNT = 4
} wheel_index_t;

/* ROS array order is always [FL, FR, RL, RR]. */
static const uint8_t wheel_can_ids[WHEEL_COUNT] = {2U, 1U, 4U, 3U};
static const int8_t wheel_directions[WHEEL_COUNT] = {1, -1, 1, -1};

#define APP_PI_F                         3.14159265359f
#define WHEEL_DIAMETER_M                 0.1524f
#define WHEEL_RADIUS_M                   0.0762f
#define GEAR_RATIO                       24.0f
#define MOTOR_POLE_PAIRS                 4.0f
#define ERPM_PER_RAD_S                   ((60.0f * GEAR_RATIO * MOTOR_POLE_PAIRS) / (2.0f * APP_PI_F))
#define MAX_ABS_ERPM                     15000

#define MOTOR_TASK_PERIOD_MS             20U
/* Leave margin for the 20 ms control task, 50 ms VESC telemetry period,
   and physical deceleration while guaranteeing a stop within 300 ms. */
#define WHEEL_COMMAND_TIMEOUT_MS         100U
#define VESC_TELEMETRY_TIMEOUT_MS        500U
#define WHEEL_STATE_PUBLISH_PERIOD_MS    50U

#define CAN_PACKET_SET_RPM               3U
#define CAN_PACKET_STATUS                9U
#define CAN_PACKET_STATUS_5              27U
#define CAN_TX_TIMEOUT_MS                2U

#define AGENT_PING_PERIOD_MS             1000U
#define AGENT_WAIT_RETRY_MS              500U
#define AGENT_PING_TIMEOUT_MS            100U

#define UART_DMA_BUFFER_SIZE             2048U
#define UART_TX_TIMEOUT_MS               100U

#define RELAY_ACTIVE_LOW                 1U
#define RELAY_DEFAULT_ENABLED            0U
#define RELAY_STOP_MIN_CYCLES            3U
#define RELAY_STOP_DELAY_MS              60U

#endif /* APP_CONFIG_H */
