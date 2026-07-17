# STM32F446RE micro-ROS wheel controller

This project is the safety boundary between the ROS 2 wheel controller and four
VESCs. The STM32 consumes wheel-level commands; it does not subscribe to
`/cmd_vel` and does not perform mecanum kinematics.

## System data flow

```text
Teleop / Nav2
  -> /cmd_vel
  -> PC mecanum_controller
  -> /wheel_commands
  -> STM32 micro-ROS
  -> VESC CAN

VESC STATUS + STATUS_5
  -> STM32
  -> /wheel_states
  -> PC mecanum_odometry
  -> /odom + TF
  -> SLAM
```

## Hardware and transport

- Board: NUCLEO-F446RE
- ROS 2 / micro-ROS: Humble
- RTOS: FreeRTOS / CMSIS-RTOS v2
- micro-ROS transport: USART2, 115200 baud
- USART2 RX: DMA1 Stream 5, circular, 2048-byte buffer
- USART2 TX: DMA1 Stream 6, normal mode, 100 ms completion timeout
- CAN1: PB8 RX, PB9 TX, 500 kbit/s
- Relay: PA8, active-low, initially ON to preserve the existing hardware behavior
- Wheel diameter: 0.1524 m; radius: 0.0762 m
- Gear ratio: 24.0; motor pole pairs: 4.0

## Wheel order, VESC IDs, and direction

Every ROS array and every firmware table uses `[FL, FR, RL, RR]`.

| Index | Wheel | VESC CAN ID | Direction |
|---:|---|---:|---:|
| 0 | FL | 2 | +1 |
| 1 | FR | 1 | -1 |
| 2 | RL | 4 | +1 |
| 3 | RR | 3 | -1 |

The rear mapping is deliberately `RL=4`, `RR=3`.

## ROS interface

### `/wheel_commands`

- Type: `agv_msgs/msg/WheelCommands`
- Direction: PC to STM32
- QoS: Best Effort
- Unit: rad/s
- Order: `[FL, FR, RL, RR]`

The 50 Hz motor task converts each wheel command with:

```text
ERPM = velocity_rad_s * 60 * gear_ratio * pole_pairs / (2*pi)
```

`ERPM_PER_RAD_S` is calculated from the configured constants (about 916.73),
then the wheel direction is applied and the result is limited to ±15,000 ERPM.

All four targets are forced to zero and continuously transmitted every 20 ms
when any of these conditions is true:

- no valid command has been received;
- the last valid command is older than 300 ms;
- a value contains NaN or infinity;
- the micro-ROS Agent is disconnected or entities are being changed;
- the relay is stopping or OFF.

### `/wheel_states`

- Type: `agv_msgs/msg/WheelStates`
- Direction: STM32 to PC
- QoS: Best Effort
- Publish rate: 20 Hz
- Order: `[FL, FR, RL, RR]`

`erpm` is the original signed value received from each VESC. The
`velocity_rad_s` field is converted back to the robot wheel convention and
therefore includes the configured direction correction. Tachometer counts are
extended from the VESC signed 32-bit counter to a signed 64-bit accumulator.

A wheel is online only when both `STATUS` and `STATUS_5` have been received and
both are at most 500 ms old. When offline, `online=false`, `erpm=0`, and
`velocity_rad_s=0`; the last accumulated tachometer value is retained.

The firmware attempts Agent time synchronization after entity creation. The
header stamp is zero if synchronization fails, but state publication continues.
`header.frame_id` is intentionally empty because the message contains four
wheel measurements rather than a single spatial frame.

### `/relay_cmd`

- Type: `std_msgs/msg/Bool`
- QoS: Reliable/default
- `true`: turn the relay ON, invalidate the previous wheel command, and wait for
  a new `/wheel_commands` sample while continuing to send zero ERPM
- `false`: enter `RELAY_STOPPING`, send zero ERPM for at least three motor-task
  cycles and at least 60 ms, then physically turn the relay OFF

The relay callback never blocks. Agent disconnection stops motor commands but
does not change the relay state.

## VESC telemetry configuration

Each VESC must periodically emit these extended CAN status packets:

- `CAN_PACKET_STATUS` for ERPM
- `CAN_PACKET_STATUS_5` for the tachometer counter

The 500 ms online policy assumes both messages are configured at a rate faster
than 2 Hz; 10–50 Hz is recommended.

## Build `agv_msgs`

From the repository root on Ubuntu 22.04 with ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_msgs
source install/setup.bash
```

## Rebuild the micro-ROS static library

Docker is required only when the message set or micro-ROS configuration changes.
The script stages `src/agv_msgs` into the builder and removes the staging copy
afterward.

```bash
cd firmware/f446re_microros
./build_microros_library.sh --force
```

Expected outputs:

- `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a`
- `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/`
- entries for `agv_msgs/WheelCommands.msg` and `agv_msgs/WheelStates.msg` in
  `libmicroros/available_ros2_types`

## STM32CubeIDE build and flash

1. Install STM32CubeIDE and STM32CubeF4 firmware package 1.27.1.
2. Import this directory with **File > Import > Existing Projects into Workspace**.
3. Confirm the regenerated micro-ROS library and headers exist at the paths above.
4. Run **Project > Clean**, then **Project > Build Project**.
5. Confirm there are no unresolved `agv_msgs` typesupport symbols.
6. Flash through the onboard ST-LINK.

## Run the Agent

```bash
source /opt/ros/humble/setup.bash
source ~/microros_agent_ws/install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v6
```

## Test commands

Relay ON:

```bash
ros2 topic pub --once /relay_cmd std_msgs/msg/Bool "{data: true}"
```

Relay OFF with the non-blocking zero-command safety sequence:

```bash
ros2 topic pub --once /relay_cmd std_msgs/msg/Bool "{data: false}"
```

Low-speed wheel command at 20 Hz:

```bash
ros2 topic pub --rate 20 --qos-reliability best_effort \
  /wheel_commands agv_msgs/msg/WheelCommands \
  "{velocity_rad_s: [1.0, 1.0, 1.0, 1.0]}"
```

Stop that publisher and verify zero ERPM transmission begins no later than
300 ms after the last valid sample.

Wheel states:

```bash
ros2 topic echo --qos-reliability best_effort /wheel_states
```

Agent reconnect test:

1. Keep the STM32 powered and publish a low-speed command.
2. Stop the Agent and verify all targets become zero.
3. Restart the Agent without resetting the STM32.
4. Confirm `/wheel_commands`, `/relay_cmd`, and `/wheel_states` reappear.
5. Publish a new wheel command; old commands are never resumed automatically.

## Hardware verification sequence

1. Power OFF the STM32 and all VESCs.
2. Verify VESC IDs: FL=2, FR=1, RL=4, RR=3.
3. Raise all wheels clear of the floor.
4. Verify CAN termination and wiring.
5. Start the micro-ROS Agent.
6. Power the STM32 and VESCs.
7. Confirm the three ROS topics and node are present.
8. Publish `/relay_cmd true`.
9. Publish a small positive command for one wheel at a time.
10. Verify wheel identity and direction for FL, FR, RL, RR.
11. Stop command publication and verify the 300 ms stop behavior.
12. Check `/wheel_states`, including STATUS/STATUS_5 online transitions.
13. Stop the Agent and verify zero ERPM behavior.
14. Restart the Agent and verify automatic entity recovery without MCU reset.
15. Publish `/relay_cmd false` and verify zero commands precede relay cutoff.

Do not test full-speed motion until every item above passes with the wheels raised.
