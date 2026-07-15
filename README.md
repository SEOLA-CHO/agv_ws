# AGV workspace

Team repository for the ROS 2 AGV workspace and STM32 motor-controller firmware.

## Repository layout

- `src/`: ROS 2 packages (when added)
- `firmware/f446re_microros/`: NUCLEO-F446RE FreeRTOS + micro-ROS firmware

The STM32 project subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`) and sends
50 Hz VESC CAN commands for a four-wheel mecanum platform.

See [`firmware/f446re_microros/README.md`](firmware/f446re_microros/README.md)
for firmware setup, build, flashing, and test instructions.
