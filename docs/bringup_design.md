# AGV Bringup Design

## Purpose

`agv_bringup` provides the integration boundary while hardware and team-owned
ROS packages are still under development. It does not implement simulation,
odometry, robot description, or the LiDAR driver.

All mapping components are opt-in. Running `mapping.launch.py` without
arguments performs no optional package lookup, logs that the skeleton is ready,
and exits normally. This is the supported WSL development behavior.

## Runtime composition

The intended data and TF flow is:

```text
agv_sim or STM32 micro-ROS -> /wheel_states -> agv_odometry -> /odom
sllidar_ros2 -----------------------------------------------> /scan
agv_description -> robot_state_publisher -------------------> base/laser TF
slam_toolbox: /scan + /odom + TF -> /map and map -> odom TF
RViz2: /map + /scan + TF
```

`mapping.launch.py` exposes independent switches for LiDAR, description, wheel
simulation, odometry, SLAM, and RViz. Every switch defaults to `false`; enabling
SLAM does not implicitly enable description or odometry.

The hardware wheel path remains a placeholder. No micro-ROS process is started
when `use_sim_wheels` is false because the host package and launch contract are
not yet defined.

## Deferred dependency resolution

Launch configurations are parsed inside an `OpaqueFunction`. A package lookup
occurs only after its corresponding `use_*` switch evaluates to true. Accepted
boolean values are case-insensitive `true` and `false`; all other values are
rejected with the argument name and received value.

For a selected component, `strict_dependencies:=false` emits a warning and
skips a missing package or launch file. `strict_dependencies:=true` raises a
`RuntimeError` containing the component, package, and expected launch file.
Native Ubuntu integration tests should use strict mode; WSL development defaults
to non-strict mode.

## LiDAR integration

`lidar_test.launch.py` includes the installed upstream
`sllidar_ros2/launch/sllidar_c1_launch.py`. It passes `serial_port`,
`serial_baudrate`, and `frame_id`. Because the upstream launch has no scan topic
argument, the driver's `scan` topic is remapped to `scan_topic` by the enclosing
launch group.

Defaults are `/dev/ttyUSB0`, `460800`, `laser`, and `/scan`. The upstream driver
package is not modified.

## Team launch contract

Bringup expects these installed files:

- `agv_sim/launch/simulator.launch.py`
- `agv_odometry/launch/odometry.launch.py`
- `agv_description/launch/description.launch.py`

Odometry accepts `odom_frame` and `base_frame`; description accepts `base_frame`
and `laser_frame`. The three packages remain optional runtime integrations and
are not mandatory `exec_depend` entries in `agv_bringup`.

## Integration order

1. Install and verify RPLIDAR C1 output and scan remapping.
2. Add `agv_description` and verify the complete fixed TF chain.
3. Integrate `agv_sim`, then replace it with micro-ROS for hardware tests.
4. Add `agv_odometry` and verify `odom -> base_footprint` plus `/odom`.
5. Install and tune `slam_toolbox` using verified scan, odometry, and TF data.
6. Validate the complete system in RViz2.
7. Run native Ubuntu acceptance tests with `strict_dependencies:=true`.
