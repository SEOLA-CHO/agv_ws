#include "motor_control.h"

#include "cmsis_os.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define CAN_PACKET_SET_RPM       3U
#define CAN_TX_TIMEOUT_MS        2U

#define MOTOR_TASK_PERIOD_MS     20U
#define CMD_VEL_TIMEOUT_MS       500U

#define PI_F                     3.14159265359f
#define WHEEL_DIAMETER_M         0.1524f
#define GEAR_RATIO               24.0f
#define MOTOR_POLE_PAIRS         4.0f
#define WHEELBASE_M              0.445f
#define TRACK_WIDTH_M            0.400f
#define ROTATION_ARM_M           ((WHEELBASE_M * 0.5f) + (TRACK_WIDTH_M * 0.5f))
#define ERPM_PER_MPS             ((60.0f * GEAR_RATIO * MOTOR_POLE_PAIRS) / \
                                  (PI_F * WHEEL_DIAMETER_M))
#define MAX_ERPM                 15000

#define VESC_ID_FR               1U
#define VESC_ID_FL               2U
#define VESC_ID_RR               3U
#define VESC_ID_RL               4U

#define MOTOR_FR_DIR             (-1)
#define MOTOR_FL_DIR             1
#define MOTOR_RR_DIR             (-1)
#define MOTOR_RL_DIR             1

typedef struct {
    float vx;
    float vy;
    float wz;
    uint32_t last_update_ms;
    bool received;
} cmd_vel_state_t;

static cmd_vel_state_t command_state;
static volatile uint32_t can_tx_error_count;

static int32_t clamp_int32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static HAL_StatusTypeDef vesc_send_erpm(uint8_t vesc_id, int32_t erpm)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[4];
    uint32_t mailbox;
    uint32_t started_at = HAL_GetTick();

    header.ExtId = ((uint32_t)CAN_PACKET_SET_RPM << 8) |
                   ((uint32_t)vesc_id & 0xFFU);
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 4U;
    header.TransmitGlobalTime = DISABLE;

    data[0] = (uint8_t)(((uint32_t)erpm >> 24) & 0xFFU);
    data[1] = (uint8_t)(((uint32_t)erpm >> 16) & 0xFFU);
    data[2] = (uint8_t)(((uint32_t)erpm >> 8) & 0xFFU);
    data[3] = (uint8_t)((uint32_t)erpm & 0xFFU);

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((HAL_GetTick() - started_at) >= CAN_TX_TIMEOUT_MS) {
            return HAL_TIMEOUT;
        }
        taskYIELD();
    }

    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

static void calculate_erpm(
    float vx,
    float vy,
    float wz,
    int32_t *fr,
    int32_t *fl,
    int32_t *rr,
    int32_t *rl)
{
    float rotation_velocity = ROTATION_ARM_M * wz;
    float commands[4] = {
        (vx + vy + rotation_velocity) * ERPM_PER_MPS,
        (vx - vy - rotation_velocity) * ERPM_PER_MPS,
        (vx - vy + rotation_velocity) * ERPM_PER_MPS,
        (vx + vy - rotation_velocity) * ERPM_PER_MPS
    };
    float max_abs = 0.0f;
    float scale = 1.0f;

    for (uint32_t i = 0U; i < 4U; ++i) {
        float magnitude = fabsf(commands[i]);
        if (magnitude > max_abs) {
            max_abs = magnitude;
        }
    }

    if (max_abs > (float)MAX_ERPM) {
        scale = (float)MAX_ERPM / max_abs;
    }

    *fr = clamp_int32((int32_t)(commands[0] * scale), -MAX_ERPM, MAX_ERPM);
    *fl = clamp_int32((int32_t)(commands[1] * scale), -MAX_ERPM, MAX_ERPM);
    *rr = clamp_int32((int32_t)(commands[2] * scale), -MAX_ERPM, MAX_ERPM);
    *rl = clamp_int32((int32_t)(commands[3] * scale), -MAX_ERPM, MAX_ERPM);
}

void motor_control_set_cmd_vel(float vx, float vy, float wz)
{
    taskENTER_CRITICAL();
    command_state.vx = vx;
    command_state.vy = vy;
    command_state.wz = wz;
    command_state.last_update_ms = HAL_GetTick();
    command_state.received = true;
    taskEXIT_CRITICAL();
}

void motor_control_stop(void)
{
    motor_control_set_cmd_vel(0.0f, 0.0f, 0.0f);
}

void motor_control_run(void)
{
    uint32_t next_wake = osKernelGetTickCount();

    for (;;) {
        cmd_vel_state_t snapshot;
        int32_t erpm_fr = 0;
        int32_t erpm_fl = 0;
        int32_t erpm_rr = 0;
        int32_t erpm_rl = 0;

        taskENTER_CRITICAL();
        snapshot = command_state;
        taskEXIT_CRITICAL();

        if (snapshot.received &&
            ((HAL_GetTick() - snapshot.last_update_ms) <= CMD_VEL_TIMEOUT_MS)) {
            calculate_erpm(
                snapshot.vx, snapshot.vy, snapshot.wz,
                &erpm_fr, &erpm_fl, &erpm_rr, &erpm_rl);
        }

        if (vesc_send_erpm(VESC_ID_FR, erpm_fr * MOTOR_FR_DIR) != HAL_OK) {
            ++can_tx_error_count;
        }
        if (vesc_send_erpm(VESC_ID_FL, erpm_fl * MOTOR_FL_DIR) != HAL_OK) {
            ++can_tx_error_count;
        }
        if (vesc_send_erpm(VESC_ID_RR, erpm_rr * MOTOR_RR_DIR) != HAL_OK) {
            ++can_tx_error_count;
        }
        if (vesc_send_erpm(VESC_ID_RL, erpm_rl * MOTOR_RL_DIR) != HAL_OK) {
            ++can_tx_error_count;
        }

        next_wake += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake);
    }
}
