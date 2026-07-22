#!/usr/bin/env python3
"""Execute odometry-closed-loop relative planar motion goals."""

from dataclasses import dataclass
import math
import threading
import time
from typing import Optional

from agv_control.relative_motion import (
    body_frame_error,
    constant_angular_command,
    constant_linear_command,
    normalize_angle,
    proportional_angular_command,
    quaternion_to_yaw,
    relative_pose,
    rotation_target_reached,
    translation_target_reached,
)
from agv_msgs.action import MoveRelative
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class PoseSample:
    """Latest planar odometry sample and monotonic receipt time."""

    x: float
    y: float
    yaw: float
    received_at: float


class RelativeMotionController(Node):
    """Translate relative pose goals into safe, bounded velocity commands."""

    def __init__(self) -> None:
        super().__init__('relative_motion_controller')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_timeout', 0.2)
        self.declare_parameter('default_goal_timeout', 30.0)
        self.declare_parameter('default_max_linear_speed', 0.20)
        self.declare_parameter('default_max_angular_speed', 0.40)
        self.declare_parameter('angular_gain', 1.8)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('yaw_tolerance', math.radians(1.0))
        self.declare_parameter('yaw_stop_lead', math.radians(5.0))
        self.declare_parameter('settle_cycles', 5)

        self.control_rate = self._float_parameter('control_rate')
        self.odom_timeout = self._float_parameter('odom_timeout')
        self.default_goal_timeout = self._float_parameter(
            'default_goal_timeout'
        )
        self.default_max_linear_speed = self._float_parameter(
            'default_max_linear_speed'
        )
        self.default_max_angular_speed = self._float_parameter(
            'default_max_angular_speed'
        )
        self.angular_gain = self._float_parameter('angular_gain')
        self.position_tolerance = self._float_parameter(
            'position_tolerance'
        )
        self.yaw_tolerance = self._float_parameter('yaw_tolerance')
        self.yaw_stop_lead = self._float_parameter('yaw_stop_lead')
        self.settle_cycles = int(self.get_parameter('settle_cycles').value)
        self._validate_parameters()

        self._lock = threading.Lock()
        self._latest_pose: Optional[PoseSample] = None
        self._goal_reserved = False
        self._stop_requested = threading.Event()
        self._shutdown_requested = threading.Event()
        callback_group = ReentrantCallbackGroup()

        self._command_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self._odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
            callback_group=callback_group,
        )
        self._stop_service = self.create_service(
            Trigger,
            '/move_relative_stop',
            self._stop_callback,
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            MoveRelative,
            '/move_relative',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self.get_logger().info(
            'Relative motion controller started: '
            f'linear_limit={self.default_max_linear_speed:.3f}m/s, '
            f'angular_limit={self.default_max_angular_speed:.3f}rad/s, '
            f'position_tolerance={self.position_tolerance:.3f}m, '
            f'yaw_tolerance={math.degrees(self.yaw_tolerance):.2f}deg, '
            f'yaw_stop_lead={math.degrees(self.yaw_stop_lead):.2f}deg'
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _validate_parameters(self) -> None:
        values = (
            self.control_rate,
            self.odom_timeout,
            self.default_goal_timeout,
            self.default_max_linear_speed,
            self.default_max_angular_speed,
            self.angular_gain,
            self.position_tolerance,
            self.yaw_tolerance,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('All relative motion parameters must be positive')
        if not math.isfinite(self.yaw_stop_lead) or self.yaw_stop_lead < 0.0:
            raise ValueError('yaw_stop_lead must be finite and non-negative')
        if self.settle_cycles <= 0:
            raise ValueError('settle_cycles must be positive')

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        sample = PoseSample(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=quaternion_to_yaw(
                float(pose.orientation.z), float(pose.orientation.w)
            ),
            received_at=time.monotonic(),
        )
        if not all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw)):
            self.get_logger().warning('Ignoring non-finite /odom pose')
            return
        with self._lock:
            self._latest_pose = sample

    def _goal_callback(self, goal_request) -> GoalResponse:
        values = (
            goal_request.x,
            goal_request.y,
            goal_request.yaw,
            goal_request.max_linear_speed,
            goal_request.max_angular_speed,
            goal_request.timeout,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Rejecting non-finite relative goal')
            return GoalResponse.REJECT
        if (
            goal_request.max_linear_speed < 0.0
            or goal_request.max_angular_speed < 0.0
            or goal_request.timeout < 0.0
        ):
            self.get_logger().warning('Rejecting goal with negative limits')
            return GoalResponse.REJECT
        if goal_request.max_linear_speed > self.default_max_linear_speed:
            self.get_logger().warning('Requested linear speed exceeds safety limit')
            return GoalResponse.REJECT
        if goal_request.max_angular_speed > self.default_max_angular_speed:
            self.get_logger().warning('Requested angular speed exceeds safety limit')
            return GoalResponse.REJECT

        with self._lock:
            if self._goal_reserved:
                self.get_logger().warning('Rejecting goal while another is active')
                return GoalResponse.REJECT
            self._stop_requested.clear()
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _stop_callback(self, _request, response):
        with self._lock:
            goal_active = self._goal_reserved
        self._stop_requested.set()
        self.publish_stop()
        response.success = True
        response.message = (
            'Active relative goal stop requested'
            if goal_active
            else 'No active goal; zero velocity published'
        )
        return response

    def _pose_sample(self) -> Optional[PoseSample]:
        with self._lock:
            return self._latest_pose

    def _publish_command(self, vx: float, vy: float, wz: float) -> None:
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self._command_publisher.publish(message)

    def publish_stop(self) -> None:
        """Publish an explicit zero velocity command."""
        if rclpy.ok():
            self._publish_command(0.0, 0.0, 0.0)

    @staticmethod
    def _result(
        success: bool,
        message: str,
        position_error: float,
        yaw_error: float,
    ) -> MoveRelative.Result:
        result = MoveRelative.Result()
        result.success = success
        result.message = message
        result.final_position_error = position_error
        result.final_yaw_error = yaw_error
        return result

    def _execute_callback(self, goal_handle) -> MoveRelative.Result:
        request = goal_handle.request
        start_time = time.monotonic()
        period = 1.0 / self.control_rate
        timeout = request.timeout or self.default_goal_timeout
        max_linear_speed = (
            request.max_linear_speed or self.default_max_linear_speed
        )
        max_angular_speed = (
            request.max_angular_speed or self.default_max_angular_speed
        )
        position_error = math.hypot(request.x, request.y)
        yaw_error = normalize_angle(request.yaw)
        translation_complete = position_error <= self.position_tolerance
        rotation_complete = (
            translation_complete and abs(yaw_error) <= self.yaw_tolerance
        )
        previous_error_x = request.x
        previous_error_y = request.y
        previous_yaw_error = yaw_error

        try:
            start_pose = self._pose_sample()
            if (
                start_pose is None
                or start_time - start_pose.received_at > self.odom_timeout
            ):
                goal_handle.abort()
                return self._result(
                    False, 'No fresh odometry at goal start',
                    position_error, yaw_error,
                )

            settled = 0
            while rclpy.ok() and not self._shutdown_requested.is_set():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False, 'Goal canceled', position_error, yaw_error
                    )
                if self._stop_requested.is_set():
                    goal_handle.abort()
                    return self._result(
                        False, 'Stop requested', position_error, yaw_error
                    )

                now = time.monotonic()
                if now - start_time > timeout:
                    goal_handle.abort()
                    return self._result(
                        False, 'Goal timeout', position_error, yaw_error
                    )

                current_pose = self._pose_sample()
                if (
                    current_pose is None
                    or now - current_pose.received_at > self.odom_timeout
                ):
                    goal_handle.abort()
                    return self._result(
                        False, 'Odometry became stale',
                        position_error, yaw_error,
                    )

                relative_x, relative_y, relative_yaw = relative_pose(
                    start_pose.x,
                    start_pose.y,
                    start_pose.yaw,
                    current_pose.x,
                    current_pose.y,
                    current_pose.yaw,
                )
                error_x_start = request.x - relative_x
                error_y_start = request.y - relative_y
                position_error = math.hypot(error_x_start, error_y_start)
                yaw_error = normalize_angle(request.yaw - relative_yaw)

                if not translation_complete:
                    translation_complete = translation_target_reached(
                        previous_error_x,
                        previous_error_y,
                        error_x_start,
                        error_y_start,
                        self.position_tolerance,
                    )
                previous_error_x = error_x_start
                previous_error_y = error_y_start
                if translation_complete and not rotation_complete:
                    rotation_complete = rotation_target_reached(
                        previous_yaw_error,
                        yaw_error,
                        self.yaw_tolerance,
                        self.yaw_stop_lead,
                    )
                previous_yaw_error = yaw_error

                feedback = MoveRelative.Feedback()
                feedback.remaining_distance = position_error
                feedback.remaining_yaw = yaw_error
                goal_handle.publish_feedback(feedback)

                if (
                    translation_complete
                    and rotation_complete
                ):
                    settled += 1
                    self.publish_stop()
                    if settled >= self.settle_cycles:
                        goal_handle.succeed()
                        return self._result(
                            True,
                            'Relative target reached',
                            position_error,
                            yaw_error,
                        )
                else:
                    settled = 0
                    if translation_complete:
                        vx, vy = 0.0, 0.0
                    else:
                        error_x_body, error_y_body = body_frame_error(
                            error_x_start, error_y_start, relative_yaw
                        )
                        vx, vy = constant_linear_command(
                            error_x_body,
                            error_y_body,
                            max_linear_speed,
                            self.position_tolerance,
                        )
                    if rotation_complete:
                        wz = 0.0
                    elif translation_complete:
                        wz = constant_angular_command(
                            yaw_error,
                            max_angular_speed,
                            self.yaw_tolerance,
                        )
                    else:
                        wz = proportional_angular_command(
                            yaw_error,
                            self.angular_gain,
                            max_angular_speed,
                            self.yaw_tolerance,
                        )
                    self._publish_command(vx, vy, wz)

                time.sleep(period)

            goal_handle.abort()
            return self._result(
                False, 'ROS shutdown', position_error, yaw_error
            )
        finally:
            self.publish_stop()
            with self._lock:
                self._goal_reserved = False

    def destroy_node(self):
        self._shutdown_requested.set()
        self._stop_requested.set()
        self.publish_stop()
        self._action_server.destroy()
        return super().destroy_node()

    def request_shutdown(self) -> None:
        """Stop an active worker before shutting down the executor."""
        self._shutdown_requested.set()
        self._stop_requested.set()
        self.publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeMotionController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
