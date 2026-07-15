# AGV ROS 2 Interface Specification

## 1. Vehicle Type

The vehicle is a four-wheel mecanum AGV.

It is not an Ackermann vehicle.

Coordinate convention:

- +X: forward
- +Y: left
- +Z: upward
- positive angular.z: counterclockwise rotation

## 2. Vehicle Dimensions

- wheel_base_length: 0.445 m
- wheel_base_width: 0.400 m
- wheel_radius: TBD

Definitions:

- L = wheel_base_length / 2 = 0.2225 m
- W = wheel_base_width / 2 = 0.2000 m
- K = L + W = 0.4225 m

## 3. Wheel Naming and Order

The ROS wheel order is always:

1. front_left_wheel
2. front_right_wheel
3. rear_left_wheel
4. rear_right_wheel

Array order:

[FL, FR, RL, RR]

Do not use CAN ID order as the ROS array order.

## 4. VESC CAN IDs

- front_left_wheel: CAN ID 2
- front_right_wheel: CAN ID 1
- rear_left_wheel: CAN ID 4
- rear_right_wheel: CAN ID 3

## 5. ROS Topics

### /cmd_vel

- Message type: geometry_msgs/msg/Twist
- Publisher: teleop, autonomous controller
- Subscriber: micro-ROS STM32 node or mecanum controller

Fields:

- linear.x: forward/backward velocity [m/s]
- linear.y: lateral velocity [m/s]
- angular.z: yaw angular velocity [rad/s]

### /wheel_commands

- Message type: sensor_msgs/msg/JointState
- Publisher: mecanum controller
- Subscriber: wheel simulator or hardware interface
- velocity unit: wheel-axis rad/s

Required names:

- front_left_wheel
- front_right_wheel
- rear_left_wheel
- rear_right_wheel

### /wheel_states

- Message type: sensor_msgs/msg/JointState
- Publisher: STM32 micro-ROS node or wheel simulator
- Subscriber: mecanum odometry node
- velocity unit: wheel-axis rad/s

Required names:

- front_left_wheel
- front_right_wheel
- rear_left_wheel
- rear_right_wheel

The values must already be normalized to the physical robot convention.

When a wheel physically rotates in the forward-driving direction,
its wheel state should have the expected positive sign.

The odometry node must not directly process raw ERPM.

### /odom

- Message type: nav_msgs/msg/Odometry
- Publisher: mecanum odometry node
- header.frame_id: odom
- child_frame_id: base_footprint

### /scan

- Message type: sensor_msgs/msg/LaserScan
- Publisher: RPLIDAR C1 driver
- frame_id: laser
- current frequency: approximately 10 Hz

## 6. TF Tree

Final TF tree:

map
└── odom
    └── base_footprint
        └── base_link
            └── laser

Publishers:

- map -> odom: slam_toolbox
- odom -> base_footprint: mecanum odometry node
- base_footprint -> base_link: URDF fixed joint
- base_link -> laser: URDF fixed joint

## 7. Mecanum Inverse Kinematics

Inputs:

- vx: linear.x
- vy: linear.y
- wz: angular.z

Equations:

w_fl = (vx - vy - K*wz) / R

w_fr = (vx + vy + K*wz) / R

w_rl = (vx + vy - K*wz) / R

w_rr = (vx - vy + K*wz) / R

## 8. Mecanum Forward Kinematics

vx = R/4 * (w_fl + w_fr + w_rl + w_rr)

vy = R/4 * (-w_fl + w_fr + w_rl - w_rr)

wz = R/(4*K) * (-w_fl + w_fr - w_rl + w_rr)

## 9. Timing Requirements

Recommended rates:

- /scan: approximately 10 Hz
- /wheel_commands: 20-50 Hz
- /wheel_states: 20-50 Hz
- /odom: 20-50 Hz

Recommended timeout:

- command timeout: 0.5 s

## 10. Safety Requirements

The motor command must become zero when:

- /cmd_vel timeout occurs
- micro-ROS Agent disconnects
- invalid or non-finite values are received
- the STM32 node starts
- an emergency stop occurs

## 11. Unconfirmed Parameters

The following values must not be guessed:

- wheel_radius
- actual motor direction multipliers
- LiDAR mounting position
- LiDAR roll, pitch and yaw
- actual VESC Status transmission rates
- encoder or tachometer scaling
