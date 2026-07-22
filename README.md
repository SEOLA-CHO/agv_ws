# AGV workspace

Team repository for the ROS 2 AGV workspace and STM32 motor-controller firmware.

## Repository layout

- `src/agv_msgs/`: fixed-size wheel command and state interfaces
- `src/agv_control/`: `/cmd_vel` to `/wheel_commands` mecanum controller
- `src/agv_odometry/`: `/wheel_states` to `/odom` and `odom` TF
- `firmware/f446re_microros/`: NUCLEO-F446RE FreeRTOS + micro-ROS firmware

## Runtime data flow

```text
Teleop / Nav2
  -> /cmd_vel (geometry_msgs/Twist)
  -> agv_control/mecanum_controller
  -> /wheel_commands (agv_msgs/WheelCommands)
  -> STM32 micro-ROS
  -> VESC CAN

VESC CAN STATUS + STATUS_5
  -> STM32 micro-ROS
  -> /wheel_states (agv_msgs/WheelStates)
  -> agv_odometry/mecanum_odometry
  -> /odom + odom -> base_footprint TF
```

For measured relative moves, `agv_control/relative_motion_controller` accepts
`agv_msgs/action/MoveRelative` goals on `/move_relative`, closes the loop with
`/odom`, and publishes bounded `/cmd_vel` commands. The low-level mecanum
controller remains velocity-only. See `src/agv_bringup/README.md` for the
integrated launch and command examples.

All wheel arrays use `[FL, FR, RL, RR]`. The controller and odometry packages
use the same geometry: wheel radius 0.0762 m, wheel base 0.445 m, and wheel
track 0.400 m. Positive angular velocity follows the ROS convention and turns
the physical platform counter-clockwise; command and odometry use the same
sign.

The controller publishes at 50 Hz with Best Effort QoS. It replaces stale or
invalid `/cmd_vel` input with zeros and proportionally limits all wheel speeds
to 16.36 rad/s, matching the firmware's 15,000 ERPM limit without distorting
the requested motion ratios.

Odometry consumes only robot-coordinate `velocity_rad_s` from
`WheelStates`; it never derives motion from raw ERPM. Integration pauses if a
wheel is offline or a sample is invalid. A synchronized STM32 header timestamp
is used when available, with ROS receipt time as the zero-stamp fallback.

## PC dependencies, including the micro-ROS Agent

The PC running hardware mode must have the `micro_ros_agent` package. Check
for it after sourcing ROS 2:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix micro_ros_agent
```

If the package is available from the configured ROS apt repository, install it
with:

```bash
sudo apt update
apt-cache policy ros-humble-micro-ros-agent
sudo apt install ros-humble-micro-ros-agent
```

If apt does not provide that package, build the official Humble Agent in the
workspace-local dependency directory used by this repository:

```bash
cd ~/agv_ws
mkdir -p _deps/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git \
  _deps/microros_ws/src/micro_ros_setup

cd _deps/microros_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 pkg prefix micro_ros_agent
```

Only one of the apt or source-build methods is required. The `_deps/`
directory is intentionally not stored in Git, so every new PC must install the
Agent dependency locally.

## Build

On Ubuntu 22.04 with ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select agv_msgs agv_control agv_odometry
source install/setup.bash
```

## Included map

The repository contains only the approved v4 map under `maps/`:

- `agv_map_v4.yaml` and `agv_map_v4.pgm` for static map display/navigation
- `agv_map_v4.posegraph` and `agv_map_v4.data` for SLAM Toolbox map reload

The large ZIP archives and older map versions are intentionally excluded.

## Run the PC nodes

In separate terminals after sourcing the workspace:

```bash
ros2 launch agv_control mecanum_controller.launch.py
ros2 launch agv_odometry odometry.launch.py
```

The odometry launch follows the integration-branch contract and accepts frame
overrides:

```bash
ros2 launch agv_odometry odometry.launch.py \
  odom_frame:=odom base_frame:=base_footprint publish_tf:=true
```

Low-speed test:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}"

ros2 topic echo --qos-reliability best_effort /wheel_commands
ros2 topic echo --qos-reliability best_effort /wheel_states
ros2 topic echo /odom
```

Stop the `/cmd_vel` publisher and verify that `/wheel_commands` changes to four
zeros after 0.5 seconds. The STM32 independently applies a 100 ms
wheel-command watchdog so the 20 ms control loop, VESC telemetry latency, and
physical deceleration still fit inside the 300 ms stop-safety requirement.

See [`firmware/f446re_microros/README.md`](firmware/f446re_microros/README.md)
for firmware setup, build, flashing, and test instructions.
