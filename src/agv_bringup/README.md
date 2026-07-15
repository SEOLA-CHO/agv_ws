# AGV Bringup

`agv_bringup` is the integration launch skeleton for the mecanum AGV. Every
mapping component is disabled by default so that the package can be built and
inspected in WSL without LiDAR, STM32 hardware, or unfinished team packages.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agv_bringup
source install/setup.bash
```

## Launch examples

The default command performs no optional package lookup and exits normally:

```bash
ros2 launch agv_bringup mapping.launch.py
```

Enable only components that are available:

```bash
ros2 launch agv_bringup mapping.launch.py \
  use_lidar:=true use_description:=true use_odometry:=true \
  use_slam:=true use_rviz:=true
```

For native Ubuntu integration tests, make selected missing dependencies fatal:

```bash
ros2 launch agv_bringup mapping.launch.py \
  use_lidar:=true use_slam:=true strict_dependencies:=true
```

LiDAR-only and software-stack tests are available as separate launches:

```bash
ros2 launch agv_bringup lidar_test.launch.py \
  lidar_serial_port:=/dev/ttyUSB0 lidar_serial_baudrate:=460800 \
  laser_frame:=laser scan_topic:=/scan
ros2 launch agv_bringup simulator_test.launch.py
```

Boolean launch arguments accept only `true` or `false` (case-insensitive).

## Mapping arguments

All component switches default to `false`: `use_lidar`, `use_description`,
`use_sim_wheels`, `use_odometry`, `use_slam`, and `use_rviz`.
`strict_dependencies` also defaults to `false`. Frame defaults are `map`,
`odom`, `base_footprint`, and `laser`. The LiDAR defaults are `/dev/ttyUSB0`,
`460800`, and `/scan`.

The upstream C1 launch does not expose a scan topic argument. This package
passes serial and frame parameters to it and remaps its `scan` topic instead.

## Team launch contract

These packages are optional runtime integrations and are deliberately not
required package dependencies yet. When delivered, each team package must
install the following launch file:

- `agv_sim/launch/simulator.launch.py`
- `agv_odometry/launch/odometry.launch.py`
- `agv_description/launch/description.launch.py`

The odometry launch must accept `odom_frame` and `base_frame`. The description
launch must accept `base_frame` and `laser_frame`. The simulator uses the shared
topic contract in `docs/interface_spec.md`.

The STM32 micro-ROS wheel source is not connected because its host-side package
and launch contract have not been defined. Until then, `use_sim_wheels:=false`
means that this bringup launch starts no wheel source.
