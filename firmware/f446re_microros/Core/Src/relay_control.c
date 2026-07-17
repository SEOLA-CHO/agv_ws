#include "relay_control.h"

#include "app_config.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

typedef struct {
    relay_state_t state;
    uint32_t stopping_started_ms;
    uint32_t zero_command_cycles;
} relay_control_state_t;

static relay_control_state_t relay_state;

static void relay_write_physical(bool enabled)
{
    GPIO_PinState pin_state;

#if RELAY_ACTIVE_LOW
    pin_state = enabled ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    pin_state = enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, pin_state);
}

void relay_control_init(void)
{
    relay_state.state = RELAY_DEFAULT_ENABLED ? RELAY_ON : RELAY_OFF;
    relay_state.stopping_started_ms = 0U;
    relay_state.zero_command_cycles = 0U;
    relay_write_physical(RELAY_DEFAULT_ENABLED != 0U);
}

void relay_set_enabled(bool enabled)
{
    bool turn_on = false;

    taskENTER_CRITICAL();
    if (enabled) {
        if (relay_state.state != RELAY_ON) {
            relay_state.state = RELAY_ON;
            relay_state.stopping_started_ms = 0U;
            relay_state.zero_command_cycles = 0U;
            turn_on = true;
        }
    } else if (relay_state.state == RELAY_ON) {
        relay_state.state = RELAY_STOPPING;
        relay_state.stopping_started_ms = HAL_GetTick();
        relay_state.zero_command_cycles = 0U;
    }
    taskEXIT_CRITICAL();

    if (turn_on) {
        relay_write_physical(true);
    }
}

bool relay_is_enabled(void)
{
    bool enabled;

    taskENTER_CRITICAL();
    enabled = relay_state.state != RELAY_OFF;
    taskEXIT_CRITICAL();
    return enabled;
}

bool relay_motion_allowed(void)
{
    bool allowed;

    taskENTER_CRITICAL();
    allowed = relay_state.state == RELAY_ON;
    taskEXIT_CRITICAL();
    return allowed;
}

bool relay_is_stopping(void)
{
    bool stopping;

    taskENTER_CRITICAL();
    stopping = relay_state.state == RELAY_STOPPING;
    taskEXIT_CRITICAL();
    return stopping;
}

void relay_note_zero_command_cycle(void)
{
    bool turn_off = false;
    uint32_t now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    if (relay_state.state == RELAY_STOPPING) {
        ++relay_state.zero_command_cycles;
        if ((relay_state.zero_command_cycles >= RELAY_STOP_MIN_CYCLES) &&
            ((uint32_t)(now_ms - relay_state.stopping_started_ms) >=
             RELAY_STOP_DELAY_MS)) {
            relay_state.state = RELAY_OFF;
            turn_off = true;
        }
    }
    taskEXIT_CRITICAL();

    if (turn_off) {
        relay_write_physical(false);
    }
}
