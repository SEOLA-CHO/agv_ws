"""ROS 2 node for the mock wheel simulator."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agv_sim.wheel_simulator_model import (
    NonFiniteWheelCommandError,
    WHEEL_NAMES,
    WheelCommandError,
    WheelSimulatorModel,
)


class WheelSimulatorNode(Node):
    """Receive wheel commands and publish simulated wheel states."""

    def __init__(self) -> None:
        super().__init__("wheel_simulator")

        # ROS 2 파라미터 선언
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("response_alpha", 0.1)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_wheel_speed", 30.0)
        self.declare_parameter("hard_stop_on_timeout", False)
        self.declare_parameter("noise_stddev", 0.0)

        # 파라미터를 읽고 유효성을 검사
        self.publish_rate = self._read_float_parameter(
            name="publish_rate",
            default=50.0,
            minimum=0.0,
            minimum_inclusive=False,
        )

        self.response_alpha = self._read_response_alpha()

        self.command_timeout = self._read_float_parameter(
            name="command_timeout",
            default=0.5,
            minimum=0.0,
            minimum_inclusive=True,
        )

        self.max_wheel_speed = self._read_float_parameter(
            name="max_wheel_speed",
            default=30.0,
            minimum=0.0,
            minimum_inclusive=False,
        )

        self.noise_stddev = self._read_float_parameter(
            name="noise_stddev",
            default=0.0,
            minimum=0.0,
            minimum_inclusive=True,
        )

        self.hard_stop_on_timeout = bool(
            self.get_parameter("hard_stop_on_timeout").value
        )

        # 앞에서 만든 계산 모델 생성
        self.model = WheelSimulatorModel(
            response_alpha=self.response_alpha,
            max_wheel_speed=self.max_wheel_speed,
        )

        # /wheel_states 발행자
        self.publisher = self.create_publisher(
            JointState,
            "/wheel_states",
            10,
        )

        # /wheel_commands 구독자
        self.subscription = self.create_subscription(
            JointState,
            "/wheel_commands",
            self.wheel_command_callback,
            10,
        )

        # Timeout 상태 관리
        self.last_command_time = self.get_clock().now()
        self.has_received_command = False
        self.timeout_active = False

        # 지정된 주기로 Timer 실행
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
        )

        self.get_logger().info(
            "Wheel simulator started: "
            f"publish_rate={self.publish_rate:.1f} Hz, "
            f"response_alpha={self.response_alpha}, "
            f"command_timeout={self.command_timeout} s, "
            f"max_wheel_speed={self.max_wheel_speed} rad/s"
        )

    def _read_float_parameter(
        self,
        name: str,
        default: float,
        minimum: float,
        minimum_inclusive: bool,
    ) -> float:
        """Read and validate a floating-point ROS parameter."""

        value = float(self.get_parameter(name).value)

        if not math.isfinite(value):
            self.get_logger().error(
                f"Parameter '{name}' is not finite. "
                f"Using default value {default}."
            )
            return default

        if minimum_inclusive:
            valid = value >= minimum
        else:
            valid = value > minimum

        if not valid:
            operator = ">=" if minimum_inclusive else ">"

            self.get_logger().error(
                f"Parameter '{name}' must satisfy "
                f"{name} {operator} {minimum}. "
                f"Using default value {default}."
            )
            return default

        return value

    def _read_response_alpha(self) -> float:
        """Read and validate response_alpha."""

        value = float(
            self.get_parameter("response_alpha").value
        )

        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            self.get_logger().error(
                "Parameter 'response_alpha' must satisfy "
                "0 < response_alpha <= 1. "
                "Using default value 0.1."
            )
            return 0.1

        return value

    def wheel_command_callback(
        self,
        message: JointState,
    ) -> None:
        """Process a received /wheel_commands message."""

        try:
            self.model.set_command(
                message.name,
                message.velocity,
            )

        except NonFiniteWheelCommandError as error:
            # 모델 내부에서 목표 명령이 0으로 변경됨
            self.get_logger().warning(
                f"Rejected non-finite wheel command: {error}. "
                "Target wheel speeds were reset to zero."
            )
            return

        except WheelCommandError as error:
            # 잘못된 메시지는 적용하지 않고 기존 명령 유지
            self.get_logger().warning(
                f"Rejected invalid wheel command: {error}"
            )
            return

        # 정상 명령을 받은 시간 기록
        self.last_command_time = self.get_clock().now()
        self.has_received_command = True

        # Timeout 상태였다면 정상 상태로 복귀
        if self.timeout_active:
            self.get_logger().info(
                "Valid wheel command received. "
                "Timeout state cleared."
            )

        self.timeout_active = False

    def timer_callback(self) -> None:
        """Update simulated speeds and publish /wheel_states."""

        current_time = self.get_clock().now()

        self._handle_timeout(current_time)

        # 1차 지연 모델을 한 번 계산
        simulated_velocity = self.model.update()

        # JointState 메시지 생성
        message = JointState()
        message.header.stamp = current_time.to_msg()

        # 출력 순서는 항상 FL, FR, RL, RR
        message.name = list(WHEEL_NAMES)
        message.velocity = simulated_velocity

        # position과 effort는 사용하지 않으므로 빈 배열
        message.position = []
        message.effort = []

        self.publisher.publish(message)

    def _handle_timeout(self, current_time) -> None:
        """Stop the command if no new command arrives in time."""

        # 아직 정상 명령을 한 번도 받지 않았다면 Timeout 검사 안 함
        if not self.has_received_command:
            return

        # 이미 Timeout 처리된 상태라면 반복 처리하지 않음
        if self.timeout_active:
            return

        elapsed_seconds = (
            current_time.nanoseconds
            - self.last_command_time.nanoseconds
        ) / 1_000_000_000.0

        if elapsed_seconds <= self.command_timeout:
            return

        if self.hard_stop_on_timeout:
            self.model.hard_stop()
            stop_mode = "hard stop"
        else:
            self.model.stop_command()
            stop_mode = "gradual stop"

        self.timeout_active = True

        # Timeout 상태로 처음 바뀔 때 한 번만 출력
        self.get_logger().warning(
            "Wheel command timeout: "
            f"no valid command for {elapsed_seconds:.3f} seconds. "
            f"Applying {stop_mode}."
        )


def main(args=None) -> None:
    """Run the wheel simulator node."""

    rclpy.init(args=args)

    node = WheelSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

