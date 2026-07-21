#!/usr/bin/env python3
"""Publish mecanum odometry from the F446 wheel-state contract."""

import math

from agv_msgs.msg import WheelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from agv_odom.kinematics import forward_mecanum


WHEEL_COUNT = 4


def wheel_qos() -> QoSProfile:
    """Match the Best Effort micro-ROS wheel-topic contract."""

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class MecanumOdometry(Node):
    """Integrate WheelStates into odom and odom-to-base TF."""

    def __init__(self):
        super().__init__('mecanum_odometry')

        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('wheelbase_x', 0.445)
        self.declare_parameter('wheelbase_y', 0.400)

        self.declare_parameter('wheel_state_topic', '/wheel_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('front_left_sign', 1.0)
        self.declare_parameter('front_right_sign', 1.0)
        self.declare_parameter('rear_left_sign', 1.0)
        self.declare_parameter('rear_right_sign', 1.0)
        self.declare_parameter('require_all_wheels_online', True)

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('wheel_data_timeout', 0.2)
        self.declare_parameter('wheel_velocity_deadband', 0.02)
        self.declare_parameter('minimum_dt', 0.001)
        self.declare_parameter('maximum_dt', 0.1)

        self.declare_parameter('linear_x_scale', 1.0)
        self.declare_parameter('linear_y_scale', 1.0)
        self.declare_parameter('angular_z_scale', -1.0)

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.wheelbase_x = float(
            self.get_parameter('wheelbase_x').value
        )
        self.wheelbase_y = float(
            self.get_parameter('wheelbase_y').value
        )

        if not math.isfinite(self.wheel_radius) or self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be finite and greater than zero')
        if (
            not math.isfinite(self.wheelbase_x)
            or not math.isfinite(self.wheelbase_y)
            or self.wheelbase_x <= 0.0
            or self.wheelbase_y <= 0.0
        ):
            raise ValueError(
                'wheelbase_x and wheelbase_y must be finite and positive'
            )

        self.wheel_state_topic = str(
            self.get_parameter('wheel_state_topic').value
        )
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.signs = [
            float(self.get_parameter('front_left_sign').value),
            float(self.get_parameter('front_right_sign').value),
            float(self.get_parameter('rear_left_sign').value),
            float(self.get_parameter('rear_right_sign').value),
        ]
        if not all(math.isfinite(value) and value != 0.0 for value in self.signs):
            raise ValueError('Wheel sign multipliers must be finite and non-zero')

        self.require_all_wheels_online = bool(
            self.get_parameter('require_all_wheels_online').value
        )
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.timeout = float(
            self.get_parameter('wheel_data_timeout').value
        )
        self.deadband = float(
            self.get_parameter('wheel_velocity_deadband').value
        )
        self.minimum_dt = float(self.get_parameter('minimum_dt').value)
        self.maximum_dt = float(self.get_parameter('maximum_dt').value)

        timing_values = [
            self.update_rate,
            self.timeout,
            self.deadband,
            self.minimum_dt,
            self.maximum_dt,
        ]
        if not all(math.isfinite(value) for value in timing_values):
            raise ValueError('Timing and deadband parameters must be finite')
        if self.update_rate <= 0.0:
            raise ValueError('update_rate must be greater than zero')
        if self.timeout < 0.0 or self.deadband < 0.0:
            raise ValueError('Timeout and deadband must not be negative')
        if self.minimum_dt <= 0.0 or self.maximum_dt < self.minimum_dt:
            raise ValueError('Require 0 < minimum_dt <= maximum_dt')

        self.linear_x_scale = float(
            self.get_parameter('linear_x_scale').value
        )
        self.linear_y_scale = float(
            self.get_parameter('linear_y_scale').value
        )
        self.angular_z_scale = float(
            self.get_parameter('angular_z_scale').value
        )
        scales = [
            self.linear_x_scale,
            self.linear_y_scale,
            self.angular_z_scale,
        ]
        if not all(math.isfinite(value) and value != 0.0 for value in scales):
            raise ValueError('Velocity scale multipliers must be finite and non-zero')

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        if not all(math.isfinite(value) for value in [self.x, self.y, self.yaw]):
            raise ValueError('Initial pose must be finite')

        self.raw_wheels = [0.0] * WHEEL_COUNT
        self.last_wheel_time = None
        self.last_input_fault = None
        self.last_update_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )
        self.subscription = self.create_subscription(
            WheelStates,
            self.wheel_state_topic,
            self.wheel_callback,
            wheel_qos(),
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_service(Empty, 'reset_odom', self.reset_callback)
        self.create_timer(1.0 / self.update_rate, self.update)

        self.get_logger().info(
            'Mecanum odometry started: '
            f'r={self.wheel_radius}, '
            f'wheelbase_x={self.wheelbase_x}, '
            f'wheelbase_y={self.wheelbase_y}, '
            'wheel_order=[FL, FR, RL, RR]'
        )

    def _invalidate_wheel_input(self, reason):
        """Fail closed on an invalid or incomplete WheelStates sample."""

        self.raw_wheels = [0.0] * WHEEL_COUNT
        self.last_wheel_time = None

        if reason != self.last_input_fault:
            self.get_logger().warning(reason)
        self.last_input_fault = reason

    def wheel_callback(self, msg: WheelStates):
        """Accept a valid fixed-order WheelStates sample."""

        if len(msg.velocity_rad_s) != WHEEL_COUNT:
            self._invalidate_wheel_input(
                '/wheel_states requires exactly [FL, FR, RL, RR]'
            )
            return

        velocities = [float(value) for value in msg.velocity_rad_s]
        if not all(math.isfinite(value) for value in velocities):
            self._invalidate_wheel_input(
                '/wheel_states contains a non-finite velocity'
            )
            return

        if self.require_all_wheels_online and not all(msg.online):
            offline = [
                index
                for index, is_online in enumerate(msg.online)
                if not is_online
            ]
            self._invalidate_wheel_input(
                f'/wheel_states reports offline wheel indexes: {offline}'
            )
            return

        if self.last_input_fault is not None:
            self.get_logger().info('Valid wheel states recovered')

        self.raw_wheels = velocities
        self.last_wheel_time = self.get_clock().now()
        self.last_input_fault = None

    def reset_callback(self, request, response):
        """Reset the integrated pose to the origin."""

        del request
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_time = self.get_clock().now()
        self.get_logger().info('Odometry reset')
        return response

    def corrected_wheels(self, now):
        """Return fresh, calibrated wheel speeds or a safe zero vector."""

        if self.last_wheel_time is None:
            return [0.0] * WHEEL_COUNT

        age = (now - self.last_wheel_time).nanoseconds * 1.0e-9
        if age > self.timeout:
            return [0.0] * WHEEL_COUNT

        result = []
        for raw_value, sign in zip(self.raw_wheels, self.signs):
            value = raw_value * sign
            if abs(value) < self.deadband:
                value = 0.0
            result.append(value)

        return result

    def update(self):
        """Integrate one odometry update and publish it."""

        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now

        if dt < self.minimum_dt:
            return
        if dt > self.maximum_dt:
            self.publish(now, 0.0, 0.0, 0.0)
            return

        vx, vy, wz = forward_mecanum(
            self.corrected_wheels(now),
            self.wheel_radius,
            self.wheelbase_x,
            self.wheelbase_y,
            self.linear_x_scale,
            self.linear_y_scale,
            self.angular_z_scale,
        )

        yaw_mid = self.yaw + 0.5 * wz * dt
        self.x += (
            vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)
        ) * dt
        self.y += (
            vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)
        ) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + wz * dt),
            math.cos(self.yaw + wz * dt),
        )

        self.publish(now, vx, vy, wz)

    def publish(self, now, vx, vy, wz):
        """Publish odometry and, when enabled, odom-to-base TF."""

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz

        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[14] = 1000000.0
        msg.pose.covariance[21] = 1000000.0
        msg.pose.covariance[28] = 1000000.0
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[14] = 1000000.0
        msg.twist.covariance[21] = 1000000.0
        msg.twist.covariance[28] = 1000000.0
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        if not self.publish_tf:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    """Run the odometry node."""

    rclpy.init(args=args)
    node = MecanumOdometry()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
