#!/usr/bin/env python3
"""Integrate fixed-order wheel states into planar odometry and TF."""

import math
from typing import Dict, Optional, Tuple

from agv_msgs.msg import WheelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from agv_odometry.kinematics import forward_mecanum


class MecanumOdometry(Node):
    """Calculate planar odometry from [FL, FR, RL, RR] wheel velocities."""

    def __init__(self) -> None:
        super().__init__('mecanum_odometry')

        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('wheel_base', 0.445)
        self.declare_parameter('wheel_track', 0.400)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('require_all_wheels_online', True)
        self.declare_parameter('max_update_interval', 0.5)
        self.declare_parameter('vx_direction', 1.0)
        self.declare_parameter('vy_direction', 1.0)
        self.declare_parameter('wz_direction', -1.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.require_all_wheels_online = bool(
            self.get_parameter('require_all_wheels_online').value
        )
        self.max_update_interval = float(
            self.get_parameter('max_update_interval').value
        )
        self.vx_direction = float(self.get_parameter('vx_direction').value)
        self.vy_direction = float(self.get_parameter('vy_direction').value)
        self.wz_direction = float(self.get_parameter('wz_direction').value)
        self._validate_parameters()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_sample_ns: Optional[int] = None
        self.warning_times: Dict[str, int] = {}

        wheel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.wheel_subscriber = self.create_subscription(
            WheelStates,
            '/wheel_states',
            self.wheel_state_callback,
            wheel_qos,
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        self.get_logger().info(
            'Mecanum odometry started: input order=[FL, FR, RL, RR], '
            f'frames={self.odom_frame}->{self.base_frame}, '
            f'r={self.wheel_radius:.4f}m, base={self.wheel_base:.3f}m, '
            f'track={self.wheel_track:.3f}m'
        )

    def _validate_parameters(self) -> None:
        values = (
            self.wheel_radius,
            self.wheel_base,
            self.wheel_track,
            self.max_update_interval,
            self.vx_direction,
            self.vy_direction,
            self.wz_direction,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('All odometry parameters must be finite')
        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be greater than zero')
        if self.wheel_base <= 0.0 or self.wheel_track <= 0.0:
            raise ValueError('wheel_base and wheel_track must be positive')
        if self.max_update_interval <= 0.0:
            raise ValueError('max_update_interval must be greater than zero')
        if 0.0 in (self.vx_direction, self.vy_direction, self.wz_direction):
            raise ValueError('Direction multipliers must be non-zero')
        if not self.odom_frame or not self.base_frame:
            raise ValueError('odom_frame and base_frame must not be empty')

    def _warn_throttled(self, key: str, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.warning_times.get(key, now_ns - 2_000_000_000)
        if now_ns - last_ns >= 2_000_000_000:
            self.get_logger().warning(message)
            self.warning_times[key] = now_ns

    def _sample_time(self, message: WheelStates):
        now = self.get_clock().now()
        seconds = int(message.header.stamp.sec)
        nanoseconds = int(message.header.stamp.nanosec)
        if seconds == 0 and nanoseconds == 0:
            return now.nanoseconds, now.to_msg()
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            self._warn_throttled(
                'stamp',
                'Invalid wheel-state timestamp; using receipt time.',
            )
            return now.nanoseconds, now.to_msg()
        return seconds * 1_000_000_000 + nanoseconds, message.header.stamp

    def wheel_state_callback(self, message: WheelStates) -> None:
        """Validate one wheel-state sample and update odometry."""
        wheel_velocities = [float(value) for value in message.velocity_rad_s]
        if len(wheel_velocities) != 4 or not all(
            math.isfinite(value) for value in wheel_velocities
        ):
            self.last_sample_ns = None
            self._warn_throttled(
                'velocity',
                'Invalid /wheel_states velocity array; '
                'odometry sample skipped.',
            )
            return

        if self.require_all_wheels_online and not all(message.online):
            self.last_sample_ns = None
            self._warn_throttled(
                'offline',
                'At least one wheel is offline; odometry integration paused.',
            )
            return

        vx, vy, wz = forward_mecanum(
            wheel_velocities,
            self.wheel_radius,
            self.wheel_base,
            self.wheel_track,
            self.vx_direction,
            self.vy_direction,
            self.wz_direction,
        )
        sample_ns, stamp = self._sample_time(message)

        if self.last_sample_ns is None:
            self.last_sample_ns = sample_ns
            self.publish_odometry(stamp, vx, vy, wz)
            return

        dt = (sample_ns - self.last_sample_ns) / 1e9
        self.last_sample_ns = sample_ns
        if dt <= 0.0:
            self._warn_throttled(
                'nonpositive_dt',
                f'Non-positive odometry time step ({dt:.6f}s); '
                'sample skipped.',
            )
            return
        if dt > self.max_update_interval:
            self._warn_throttled(
                'large_dt',
                f'Odometry gap {dt:.3f}s exceeds limit; pose not integrated.',
            )
            self.publish_odometry(stamp, vx, vy, wz)
            return

        midpoint_yaw = self.yaw + 0.5 * wz * dt
        cos_yaw = math.cos(midpoint_yaw)
        sin_yaw = math.sin(midpoint_yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + wz * dt),
            math.cos(self.yaw + wz * dt),
        )
        self.publish_odometry(stamp, vx, vy, wz)

    def publish_odometry(self, stamp, vx: float, vy: float, wz: float) -> None:
        """Publish nav_msgs/Odometry and optionally odom-to-base TF."""
        half_yaw = 0.5 * self.yaw
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = quaternion_z
        message.pose.pose.orientation.w = quaternion_w
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz

        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[14] = 1e6
        message.pose.covariance[21] = 1e6
        message.pose.covariance[28] = 1e6
        message.pose.covariance[35] = 0.05
        message.twist.covariance[0] = 0.02
        message.twist.covariance[7] = 0.02
        message.twist.covariance[14] = 1e6
        message.twist.covariance[21] = 1e6
        message.twist.covariance[28] = 1e6
        message.twist.covariance[35] = 0.05
        self.odom_publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = quaternion_z
            transform.transform.rotation.w = quaternion_w
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
