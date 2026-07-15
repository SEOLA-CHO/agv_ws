#!/usr/bin/env python3

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MecanumController(Node):
    """Convert robot velocity commands into four wheel angular velocities."""

    WHEEL_NAMES = [
        'front_left_wheel',
        'front_right_wheel',
        'rear_left_wheel',
        'rear_right_wheel',
    ]

    def __init__(self) -> None:
        super().__init__('mecanum_controller')

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('wheel_base', 0.40)
        self.declare_parameter('wheel_track', 0.35)

        # Command settings
        self.declare_parameter('max_wheel_speed', 30.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        # Direction correction
        self.declare_parameter('rotation_direction', -1.0)

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.wheel_base = float(
            self.get_parameter('wheel_base').value
        )
        self.wheel_track = float(
            self.get_parameter('wheel_track').value
        )
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )
        self.rotation_direction = float(
            self.get_parameter('rotation_direction').value
        )

        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be greater than zero')

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.latest_cmd = Twist()
        self.last_command_time = self.get_clock().now()
        self.command_received = False

        self.command_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.wheel_publisher = self.create_publisher(
            JointState,
            '/wheel_commands',
            10,
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.publish_wheel_commands,
        )

        self.get_logger().info(
            'Mecanum controller started: '
            f'r={self.wheel_radius:.4f} m, '
            f'wheel_base={self.wheel_base:.3f} m, '
            f'wheel_track={self.wheel_track:.3f} m, '
            f'rotation_direction={self.rotation_direction:.1f}'
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Store the most recently received velocity command."""
        values = [
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        ]

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                'Invalid cmd_vel received. Publishing zero wheel speeds.'
            )
            self.latest_cmd = Twist()
        else:
            self.latest_cmd = msg

        self.last_command_time = self.get_clock().now()
        self.command_received = True

    def calculate_wheel_speeds(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> List[float]:
        """Calculate wheel angular velocities in rad/s."""
        lever_arm = (
            self.wheel_base / 2.0
            + self.wheel_track / 2.0
        )

        # The tested vehicle rotated opposite to the original assumption.
        corrected_wz = self.rotation_direction * wz

        front_left = (
            vx - vy - lever_arm * corrected_wz
        ) / self.wheel_radius

        front_right = (
            vx + vy + lever_arm * corrected_wz
        ) / self.wheel_radius

        rear_left = (
            vx + vy - lever_arm * corrected_wz
        ) / self.wheel_radius

        rear_right = (
            vx - vy + lever_arm * corrected_wz
        ) / self.wheel_radius

        wheel_speeds = [
            front_left,
            front_right,
            rear_left,
            rear_right,
        ]

        return [
            max(
                -self.max_wheel_speed,
                min(self.max_wheel_speed, speed),
            )
            for speed in wheel_speeds
        ]

    def publish_wheel_commands(self) -> None:
        """Publish wheel commands, or stop when cmd_vel times out."""
        now = self.get_clock().now()

        timed_out = False

        if self.command_received:
            elapsed = (
                now - self.last_command_time
            ).nanoseconds / 1e9

            timed_out = elapsed > self.command_timeout

        if not self.command_received or timed_out:
            vx = 0.0
            vy = 0.0
            wz = 0.0
        else:
            vx = self.latest_cmd.linear.x
            vy = self.latest_cmd.linear.y
            wz = self.latest_cmd.angular.z

        wheel_speeds = self.calculate_wheel_speeds(
            vx,
            vy,
            wz,
        )

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.WHEEL_NAMES
        msg.velocity = wheel_speeds

        self.wheel_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MecanumController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
