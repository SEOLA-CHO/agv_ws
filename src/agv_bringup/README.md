# AGV Bringup

`agv_bringup` composes the upper-controller simulation or hardware mapping
pipeline while enforcing a single `/wheel_states` source.

Source the runtime overlays before launching:

```bash
source /opt/ros/humble/setup.bash
source _deps/sllidar_ws/install/setup.bash
source _deps/microros_ws/install/local_setup.bash
source install/setup.bash
```

## Simulation

```bash
ros2 launch agv_bringup mapping.launch.py wheel_source:=sim
```

## Hardware mapping

Use stable `/dev/serial/by-id` paths discovered from the attached devices:

```bash
ros2 launch agv_bringup mapping.launch.py \
  wheel_source:=hardware \
  agent_device:=/dev/serial/by-id/<stm32> \
  use_lidar:=true \
  lidar_device:=/dev/serial/by-id/<lidar> \
  use_slam:=true use_rviz:=true
```

The fixed frames are `map`, `odom`, `base_footprint`, `base_link`, and
`laser_frame`. The default LiDAR baudrate is `460800` for the RPLIDAR C1.
Hardware mode rejects a missing Agent device, and simulation mode never starts
the micro-ROS Agent.

## Odometry-closed-loop relative motion

Whenever `wheel_source` is `sim` or `hardware`, the launch also starts the
`/move_relative` action server. Relay power remains an explicit, separate
operator command. After odometry and motor telemetry are healthy, send a
bounded one-metre forward goal with:

```bash
ros2 action send_goal --feedback /move_relative \
  agv_msgs/action/MoveRelative \
  "{x: 1.0, y: 0.0, yaw: 0.0, max_linear_speed: 0.2, \
  max_angular_speed: 0.2, timeout: 30.0}"
```

Targets use the robot frame captured when the goal starts: positive `x` is
forward, positive `y` is left, and positive `yaw` is counter-clockwise. A zero
speed or timeout field selects the configured default. The server stops and
aborts if odometry becomes stale or the timeout expires, and publishes zero
velocity on cancellation and shutdown. Translation holds the requested linear
speed until it enters the position tolerance; pure rotation similarly holds the
requested angular speed. Straight-line yaw correction remains proportional. It
does not reverse after first entering a target tolerance or passing a target
between odometry samples. Final yaw correction uses constant angular speed with
zero linear velocity, then latches complete instead of reversing after an
overshoot. It does not provide obstacle avoidance.

Pure rotation uses the configured `yaw_stop_lead` to publish zero angular
velocity before the requested angle, compensating for drivetrain coast without
commanding a speed below the hardware's usable closed-loop threshold. The
default five-degree lead is calibrated for the default 0.4 rad/s rotation and
should be adjusted from repeated hardware measurements if the final angle has a
consistent bias.

For an operator stop that does not depend on the action client remaining open:

```bash
ros2 service call /move_relative_stop std_srvs/srv/Trigger '{}'
```

The service publishes zero velocity immediately and aborts any active relative
goal. Relay OFF remains the independent hardware-power stop.
