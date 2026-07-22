#include "motor_control.h"

#include "app_config.h"
#include "main.h"
#include "relay_control.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    float velocity_rad_s[WHEEL_COUNT];
    uint32_t last_update_ms;
    bool received;
    bool valid;
} wheel_command_state_t;

static wheel_command_state_t command_state;
static bool agent_connected;
static volatile uint32_t can_tx_error_count;
static volatile uint32_t wheel_command_invalid_count;

static int32_t clamp_erpm(int32_t erpm)
{
    if (erpm > MAX_ABS_ERPM) {
        return MAX_ABS_ERPM;
    }
    if (erpm < -MAX_ABS_ERPM) {
        return -MAX_ABS_ERPM;
    }
    return erpm;
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
        if ((uint32_t)(HAL_GetTick() - started_at) >=
            CAN_TX_TIMEOUT_MS) {
            return HAL_TIMEOUT;
        }
        taskYIELD();
    }

    return HAL_CAN_AddTxMessage(
        &hcan1, &header, data, &mailbox);
}

void motor_control_set_wheel_command(
    const float velocity_rad_s[WHEEL_COUNT],
    bool valid)
{
    uint32_t wheel;

    if (velocity_rad_s == NULL) {
        valid = false;
    }

    taskENTER_CRITICAL();
    if (velocity_rad_s != NULL) {
        for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
            command_state.velocity_rad_s[wheel] =
                velocity_rad_s[wheel];
        }
    } else {
        memset(command_state.velocity_rad_s, 0,
               sizeof(command_state.velocity_rad_s));
    }
    command_state.received = true;
    command_state.valid = valid;
    if (valid) {
        command_state.last_update_ms = HAL_GetTick();
    } else {
        ++wheel_command_invalid_count;
    }
    taskEXIT_CRITICAL();
}

void motor_control_invalidate_command(void)
{
    taskENTER_CRITICAL();
    memset(command_state.velocity_rad_s, 0,
           sizeof(command_state.velocity_rad_s));
    command_state.received = false;
    command_state.valid = false;
    command_state.last_update_ms = 0U;
    taskEXIT_CRITICAL();
}

void motor_control_set_agent_connected(bool connected)
{
    taskENTER_CRITICAL();
    agent_connected = connected;
    if (!connected) {
        memset(command_state.velocity_rad_s, 0,
               sizeof(command_state.velocity_rad_s));
        command_state.received = false;
        command_state.valid = false;
        command_state.last_update_ms = 0U;
    }
    taskEXIT_CRITICAL();
}

void motor_control_run(void)
{
    uint32_t next_wake = osKernelGetTickCount();

    for (;;) {
        wheel_command_state_t snapshot;
        bool connected;
        bool command_usable;
        int32_t target_erpm[WHEEL_COUNT] = {0};
        uint32_t wheel;
        uint32_t now_ms = HAL_GetTick();

        taskENTER_CRITICAL();
        snapshot = command_state;
        connected = agent_connected;
        taskEXIT_CRITICAL();

        command_usable = connected &&
                         relay_motion_allowed() &&
                         snapshot.received &&
                         snapshot.valid &&
                         ((uint32_t)(now_ms - snapshot.last_update_ms) <=
                          WHEEL_COMMAND_TIMEOUT_MS);

        if (command_usable) {
            for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
                float requested_erpm =
                    snapshot.velocity_rad_s[wheel] *
                    ERPM_PER_RAD_S *
                    (float)wheel_directions[wheel];
                target_erpm[wheel] = clamp_erpm(
                    (int32_t)lroundf(requested_erpm));
            }
        }

        for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
            if (vesc_send_erpm(
                    wheel_can_ids[wheel],
                    target_erpm[wheel]) != HAL_OK) {
                ++can_tx_error_count;
            }
        }

        if (relay_is_stopping()) {
            relay_note_zero_command_cycle();
        }

        next_wake += MOTOR_TASK_PERIOD_MS;
        (void)osDelayUntil(next_wake);
    }
}
