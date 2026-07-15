#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_control_set_cmd_vel(float vx, float vy, float wz);
void motor_control_stop(void);
void motor_control_run(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
