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
