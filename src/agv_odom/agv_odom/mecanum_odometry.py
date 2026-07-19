#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


class MecanumOdometry(Node):

    def __init__(self):
        super().__init__('mecanum_odometry')

        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('wheelbase_x', 0.380)
        self.declare_parameter('wheelbase_y', 0.330)

        self.declare_parameter('wheel_state_topic', '/wheel_states')
        self.declare_parameter('odom_topic', '/odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('front_left_sign', 1.0)
        self.declare_parameter('front_right_sign', -1.0)
        self.declare_parameter('rear_left_sign', 1.0)
        self.declare_parameter('rear_right_sign', -1.0)

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('wheel_data_timeout', 0.2)
        self.declare_parameter('wheel_velocity_deadband', 0.02)

        self.declare_parameter('minimum_dt', 0.001)
        self.declare_parameter('maximum_dt', 0.1)

        self.declare_parameter('linear_x_scale', 1.0)
        self.declare_parameter('linear_y_scale', 1.0)
        self.declare_parameter('angular_z_scale', 1.0)

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

        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be greater than zero')

        if self.wheelbase_x <= 0.0 or self.wheelbase_y <= 0.0:
            raise ValueError(
                'wheelbase_x and wheelbase_y must be greater than zero'
            )

        self.lx = self.wheelbase_x / 2.0
        self.ly = self.wheelbase_y / 2.0
        self.rotation_radius = self.lx + self.ly

        self.wheel_state_topic = str(
            self.get_parameter('wheel_state_topic').value
        )
        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )
        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.signs = [
            float(self.get_parameter('front_left_sign').value),
            float(self.get_parameter('front_right_sign').value),
            float(self.get_parameter('rear_left_sign').value),
            float(self.get_parameter('rear_right_sign').value),
        ]

        self.publish_tf = bool(
            self.get_parameter('publish_tf').value
        )
        self.update_rate = float(
            self.get_parameter('update_rate').value
        )
        self.timeout = float(
            self.get_parameter('wheel_data_timeout').value
        )
        self.deadband = float(
            self.get_parameter('wheel_velocity_deadband').value
        )

        self.minimum_dt = float(
            self.get_parameter('minimum_dt').value
        )
        self.maximum_dt = float(
            self.get_parameter('maximum_dt').value
        )

        self.linear_x_scale = float(
            self.get_parameter('linear_x_scale').value
        )
        self.linear_y_scale = float(
            self.get_parameter('linear_y_scale').value
        )
        self.angular_z_scale = float(
            self.get_parameter('angular_z_scale').value
        )

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        self.raw_wheels = [0.0, 0.0, 0.0, 0.0]
        self.last_wheel_time = None
        self.last_update_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.create_subscription(
            JointState,
            self.wheel_state_topic,
            self.wheel_callback,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_service(
            Empty,
            'reset_odom',
            self.reset_callback,
        )

        self.create_timer(
            1.0 / self.update_rate,
            self.update,
        )

        self.get_logger().info(
            f'Mecanum odometry started: '
            f'r={self.wheel_radius}, '
            f'wheelbase_x={self.wheelbase_x}, '
            f'wheelbase_y={self.wheelbase_y}'
        )

    def wheel_callback(self, msg):
        if len(msg.velocity) < 4:
            self.get_logger().error(
                '/wheel_states requires [FL, FR, RL, RR]'
            )
            return

        self.raw_wheels = [
            float(msg.velocity[0]),
            float(msg.velocity[1]),
            float(msg.velocity[2]),
            float(msg.velocity[3]),
        ]

        self.last_wheel_time = self.get_clock().now()

    def reset_callback(self, request, response):
        del request

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_time = self.get_clock().now()

        self.get_logger().info('Odometry reset')

        return response

    def corrected_wheels(self, now):
        if self.last_wheel_time is None:
            return [0.0, 0.0, 0.0, 0.0]

        age = (
            now - self.last_wheel_time
        ).nanoseconds * 1.0e-9

        if age > self.timeout:
            return [0.0, 0.0, 0.0, 0.0]

        result = []

        for raw_value, sign in zip(self.raw_wheels, self.signs):
            value = raw_value * sign

            if abs(value) < self.deadband:
                value = 0.0

            result.append(value)

        return result

    def update(self):
        now = self.get_clock().now()

        dt = (
            now - self.last_update_time
        ).nanoseconds * 1.0e-9

        self.last_update_time = now

        if dt < self.minimum_dt:
            return

        if dt > self.maximum_dt:
            self.publish(now, 0.0, 0.0, 0.0)
            return

        omega_fl, omega_fr, omega_rl, omega_rr = (
            self.corrected_wheels(now)
        )

        v_fl = omega_fl * self.wheel_radius
        v_fr = omega_fr * self.wheel_radius
        v_rl = omega_rl * self.wheel_radius
        v_rr = omega_rr * self.wheel_radius

        vx = (
            v_fl + v_fr + v_rl + v_rr
        ) / 4.0

        vy = (
            -v_fl + v_fr + v_rl - v_rr
        ) / 4.0

        wz = (
            -v_fl + v_fr - v_rl + v_rr
        ) / (4.0 * self.rotation_radius)

        vx *= self.linear_x_scale
        vy *= self.linear_y_scale
        wz *= self.angular_z_scale

        yaw_mid = self.yaw + 0.5 * wz * dt

        self.x += (
            vx * math.cos(yaw_mid)
            - vy * math.sin(yaw_mid)
        ) * dt

        self.y += (
            vx * math.sin(yaw_mid)
            + vy * math.cos(yaw_mid)
        ) * dt

        self.yaw = math.atan2(
            math.sin(self.yaw + wz * dt),
            math.cos(self.yaw + wz * dt),
        )

        self.publish(now, vx, vy, wz)

    def publish(self, now, vx, vy, wz):
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
    rclpy.init(args=args)

    node = MecanumOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()