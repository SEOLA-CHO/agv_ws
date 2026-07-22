#!/usr/bin/env python3
"""Convert velocity commands to fixed-order mecanum wheel commands."""

import math
from typing import Tuple

from agv_control.kinematics import inverse_mecanum
from agv_msgs.msg import WheelCommands
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy


class MecanumController(Node):
    """Publish wheel angular velocities from robot body velocity commands."""

    def __init__(self) -> None:
        super().__init__('mecanum_controller')

        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('wheel_base', 0.445)
        self.declare_parameter('wheel_track', 0.400)
        self.declare_parameter('max_wheel_speed', 16.36)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('rotation_direction', 1.0)
        self.declare_parameter('front_left_scale', 1.0)
        self.declare_parameter('front_right_scale', 1.0)
        self.declare_parameter('rear_left_scale', 1.0)
        self.declare_parameter('rear_right_scale', 1.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.rotation_direction = float(
            self.get_parameter('rotation_direction').value
        )
        self.wheel_scales = [
            float(self.get_parameter('front_left_scale').value),
            float(self.get_parameter('front_right_scale').value),
            float(self.get_parameter('rear_left_scale').value),
            float(self.get_parameter('rear_right_scale').value),
        ]
        self._validate_parameters()

        self.latest_command: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_command_time = self.get_clock().now()
        self.command_received = False

        wheel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )
        self.wheel_publisher = self.create_publisher(
            WheelCommands,
            '/wheel_commands',
            wheel_qos,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_wheel_commands,
        )

        self.get_logger().info(
            'Mecanum controller started: order=[FL, FR, RL, RR], '
            f'r={self.wheel_radius:.4f}m, base={self.wheel_base:.3f}m, '
            f'track={self.wheel_track:.3f}m, '
            f'limit={self.max_wheel_speed:.2f}rad/s'
        )

    def _validate_parameters(self) -> None:
        values = (
            self.wheel_radius,
            self.wheel_base,
            self.wheel_track,
            self.max_wheel_speed,
            self.command_timeout,
            self.publish_rate,
            self.rotation_direction,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('All controller parameters must be finite')
        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be greater than zero')
        if self.wheel_base <= 0.0 or self.wheel_track <= 0.0:
            raise ValueError('wheel_base and wheel_track must be positive')
        if self.max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed must be greater than zero')
        if self.command_timeout <= 0.0 or self.publish_rate <= 0.0:
            raise ValueError(
                'command_timeout and publish_rate must be positive'
            )
        if self.rotation_direction == 0.0:
            raise ValueError('rotation_direction must be non-zero')
        if not all(
            math.isfinite(scale) and scale > 0.0
            for scale in self.wheel_scales
        ):
            raise ValueError('Wheel scales must be finite and positive')

    def cmd_vel_callback(self, message: Twist) -> None:
        """Store only finite body-velocity commands."""
        command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        if all(math.isfinite(value) for value in command):
            self.latest_command = command
        else:
            self.latest_command = (0.0, 0.0, 0.0)
            self.get_logger().warning(
                'Non-finite /cmd_vel received; commanding zero wheel speed.'
            )

        self.last_command_time = self.get_clock().now()
        self.command_received = True

    def publish_wheel_commands(self) -> None:
        """Publish at a fixed rate, including zeros after command timeout."""
        if not rclpy.ok():
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        stale = (
            not self.command_received
            or elapsed < 0.0
            or elapsed > self.command_timeout
        )
        vx, vy, wz = (0.0, 0.0, 0.0) if stale else self.latest_command

        wheel_speeds = inverse_mecanum(
            vx=vx,
            vy=vy,
            wz=wz,
            wheel_radius=self.wheel_radius,
            wheel_base=self.wheel_base,
            wheel_track=self.wheel_track,
            rotation_direction=self.rotation_direction,
            max_wheel_speed=self.max_wheel_speed,
            wheel_scales=self.wheel_scales,
        )

        message = WheelCommands()
        message.velocity_rad_s = wheel_speeds
        self.wheel_publisher.publish(message)

    def publish_stop(self) -> None:
        """Best-effort final stop; the STM32 timeout remains authoritative."""
        if not rclpy.ok():
            return

        message = WheelCommands()
        message.velocity_rad_s = [0.0, 0.0, 0.0, 0.0]
        self.wheel_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
