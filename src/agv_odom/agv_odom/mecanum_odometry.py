#!/usr/bin/env python3

import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MecanumOdometry(Node):
    """Calculate mecanum wheel odometry from four wheel velocities."""

    WHEEL_NAMES = [
        'front_left_wheel',
        'front_right_wheel',
        'rear_left_wheel',
        'rear_right_wheel',
    ]

    def __init__(self) -> None:
        super().__init__('mecanum_odometry')

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('wheel_base', 0.40)
        self.declare_parameter('wheel_track', 0.35)

        # Frame and topic settings
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        # Direction corrections
        self.declare_parameter('vx_direction', 1.0)
        self.declare_parameter('vy_direction', 1.0)
        self.declare_parameter('wz_direction', -1.0)

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.wheel_base = float(
            self.get_parameter('wheel_base').value
        )
        self.wheel_track = float(
            self.get_parameter('wheel_track').value
        )

        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )
        self.base_frame = str(
            self.get_parameter('base_frame').value
        )
        self.publish_tf = bool(
            self.get_parameter('publish_tf').value
        )

        self.vx_direction = float(
            self.get_parameter('vx_direction').value
        )
        self.vy_direction = float(
            self.get_parameter('vy_direction').value
        )
        self.wz_direction = float(
            self.get_parameter('wz_direction').value
        )

        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be greater than zero')

        if self.wheel_base <= 0.0:
            raise ValueError('wheel_base must be greater than zero')

        if self.wheel_track <= 0.0:
            raise ValueError('wheel_track must be greater than zero')

        # Integrated robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time: Optional[rclpy.time.Time] = None

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10,
        )

        self.wheel_subscriber = self.create_subscription(
            JointState,
            '/wheel_states',
            self.wheel_state_callback,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Mecanum odometry started: '
            f'r={self.wheel_radius:.4f} m, '
            f'wheel_base={self.wheel_base:.3f} m, '
            f'wheel_track={self.wheel_track:.3f} m, '
            f'wz_direction={self.wz_direction:.1f}'
        )

    def reorder_wheel_velocities(
        self,
        msg: JointState,
    ) -> Optional[List[float]]:
        """Reorder JointState velocities using wheel names."""
        if len(msg.name) != len(msg.velocity):
            self.get_logger().warning(
                'JointState name and velocity lengths do not match.'
            )
            return None

        wheel_map: Dict[str, float] = dict(
            zip(msg.name, msg.velocity)
        )

        missing = [
            name
            for name in self.WHEEL_NAMES
            if name not in wheel_map
        ]

        if missing:
            self.get_logger().warning(
                f'Missing wheel names: {missing}'
            )
            return None

        velocities = [
            float(wheel_map[name])
            for name in self.WHEEL_NAMES
        ]

        if not all(
            math.isfinite(value)
            for value in velocities
        ):
            self.get_logger().warning(
                'Non-finite wheel velocity received.'
            )
            return None

        return velocities

    def calculate_body_velocity(
        self,
        wheel_velocities: List[float],
    ) -> tuple[float, float, float]:
        """Convert wheel angular velocities to robot body velocity."""
        fl, fr, rl, rr = wheel_velocities

        lever_arm = (
            self.wheel_base / 2.0
            + self.wheel_track / 2.0
        )

        vx = (
            self.wheel_radius
            / 4.0
            * (fl + fr + rl + rr)
        )

        vy = (
            self.wheel_radius
            / 4.0
            * (-fl + fr + rl - rr)
        )

        wz = (
            self.wheel_radius
            / (4.0 * lever_arm)
            * (-fl + fr - rl + rr)
        )

        vx *= self.vx_direction
        vy *= self.vy_direction
        wz *= self.wz_direction

        return vx, vy, wz

    def wheel_state_callback(
        self,
        msg: JointState,
    ) -> None:
        """Update pose from wheel velocity measurements."""
        wheel_velocities = self.reorder_wheel_velocities(msg)

        if wheel_velocities is None:
            return

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        self.last_time = current_time

        if dt <= 0.0 or dt > 1.0:
            self.get_logger().warning(
                f'Invalid odometry time step: {dt:.4f} s'
            )
            return

        vx, vy, wz = self.calculate_body_velocity(
            wheel_velocities
        )

        # Transform body velocity into odom/world coordinates.
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        global_vx = vx * cos_yaw - vy * sin_yaw
        global_vy = vx * sin_yaw + vy * cos_yaw

        self.x += global_vx * dt
        self.y += global_vy * dt
        self.yaw += wz * dt

        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw),
        )

        self.publish_odometry(
            current_time,
            vx,
            vy,
            wz,
        )

    def publish_odometry(
        self,
        stamp,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        """Publish Odometry message and odom-to-base TF."""
        half_yaw = self.yaw / 2.0

        quat_z = math.sin(half_yaw)
        quat_w = math.cos(half_yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = quat_z
        odom_msg.pose.pose.orientation.w = quat_w

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = wz

        # Temporary covariance values for wheel-only odometry.
        odom_msg.pose.covariance[0] = 0.02
        odom_msg.pose.covariance[7] = 0.02
        odom_msg.pose.covariance[35] = 0.05

        odom_msg.twist.covariance[0] = 0.02
        odom_msg.twist.covariance[7] = 0.02
        odom_msg.twist.covariance[35] = 0.05

        self.odom_publisher.publish(odom_msg)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp.to_msg()
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame

            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = quat_z
            transform.transform.rotation.w = quat_w

            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MecanumOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
