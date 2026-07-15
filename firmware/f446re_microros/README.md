# STM32F446RE micro-ROS mecanum controller

## Hardware and transport

- Board: NUCLEO-F446RE
- ROS 2: Humble
- RTOS: FreeRTOS / CMSIS-RTOS v2
- micro-ROS transport: USART2 at 115200 baud using DMA
- CAN1: PB8 RX, PB9 TX, 500 kbit/s
- VESC IDs: FR=1, FL=2, RR=3, RL=4
- Direction correction: right=-1, left=+1

## Control behavior

- Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Sends VESC ERPM commands at 50 Hz
- Stops all motors if no command is received for 500 ms
- Limits commands proportionally to 15,000 ERPM

## Windows build and flash

1. Install STM32CubeIDE. The project was verified with STM32CubeIDE 1.11.0 and
   STM32CubeF4 firmware package 1.27.1.
2. Import this directory using **File > Import > Existing Projects into Workspace**.
3. Run **Project > Clean**, then **Project > Build Project**.
4. Flash through the onboard ST-LINK.

The prebuilt micro-ROS headers and `libmicroros.a` are included, so Docker is
not required for a normal firmware build.

## WSL micro-ROS Agent

Use Ubuntu 22.04 and ROS 2 Humble. After building the Humble micro-ROS Agent:

```bash
source /opt/ros/humble/setup.bash
source ~/microros_agent_ws/install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v6
```

If `/dev/ttyACM0` is missing, attach the ST-LINK/VCP USB device to WSL using
`usbipd-win`. Windows cannot flash through ST-LINK while that USB device is
attached exclusively to WSL, so flash first and attach it to WSL afterward.

## Low-speed test

Raise the wheels off the ground before the first motor test.

```bash
ros2 topic pub -r 10 --qos-reliability best_effort \
  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Rebuilding `libmicroros.a`

The included library already contains `geometry_msgs/msg/Twist`. Rebuild it
only when changing the micro-ROS configuration or message set:

```bash
./build_microros_library.sh
```

See `WSL_SETUP_RESULT.md` for the original Ubuntu, Docker, Agent, and library
verification details.
