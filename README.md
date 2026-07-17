# AGV workspace

Team repository for the ROS 2 AGV workspace and STM32 motor-controller firmware.

## Repository layout

- `src/agv_msgs/`: fixed-size wheel command and state interfaces
- `firmware/f446re_microros/`: NUCLEO-F446RE FreeRTOS + micro-ROS firmware

The PC-side mecanum controller converts `/cmd_vel` to `/wheel_commands`. The
STM32 consumes only wheel-level rad/s commands, sends 50 Hz VESC CAN commands,
and publishes combined telemetry on `/wheel_states`.

See [`firmware/f446re_microros/README.md`](firmware/f446re_microros/README.md)
for firmware setup, build, flashing, and test instructions.
