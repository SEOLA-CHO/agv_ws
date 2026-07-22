#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "app_config.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void motor_control_set_wheel_command(
    const float velocity_rad_s[WHEEL_COUNT],
    bool valid);
void motor_control_invalidate_command(void);
void motor_control_set_agent_connected(bool connected);
void motor_control_run(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
