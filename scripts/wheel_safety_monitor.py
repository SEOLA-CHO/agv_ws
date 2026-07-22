#!/usr/bin/env python3
"""Measure single-wheel motion and watchdog stop latency on ROS topics."""

import json
import time

import rclpy
from agv_msgs.msg import WheelCommands, WheelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class WheelSafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("wheel_safety_monitor")
        self.last_nonzero_command_ns = None
        self.motion_seen = False
        self.result_reported = False
        self.max_abs_erpm = [0, 0, 0, 0]
        self.create_subscription(
            WheelCommands,
            "/wheel_commands",
            self.command_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WheelStates,
            "/wheel_states",
            self.state_callback,
            qos_profile_sensor_data,
        )

    @staticmethod
    def emit(event: str, **fields: object) -> None:
        print(
            json.dumps(
                {"monotonic_ns": time.monotonic_ns(), "event": event, **fields},
                separators=(",", ":"),
            ),
            flush=True,
        )

    def command_callback(self, message: WheelCommands) -> None:
        velocity = [float(value) for value in message.velocity_rad_s]
        if any(abs(value) > 0.01 for value in velocity):
            self.last_nonzero_command_ns = time.monotonic_ns()
            self.emit("nonzero_command", velocity_rad_s=velocity)

    def state_callback(self, message: WheelStates) -> None:
        now_ns = time.monotonic_ns()
        erpm = [int(value) for value in message.erpm]
        online = [bool(value) for value in message.online]
        for index, value in enumerate(erpm):
            self.max_abs_erpm[index] = max(self.max_abs_erpm[index], abs(value))

        if abs(erpm[0]) >= 100:
            self.motion_seen = True

        if (
            not self.result_reported
            and self.motion_seen
            and self.last_nonzero_command_ns is not None
            and now_ns - self.last_nonzero_command_ns >= 100_000_000
            and abs(erpm[0]) <= 50
        ):
            self.result_reported = True
            self.emit(
                "stop_result",
                stop_latency_ms=round(
                    (now_ns - self.last_nonzero_command_ns) / 1_000_000.0, 3
                ),
                erpm=erpm,
                online=online,
                max_abs_erpm=self.max_abs_erpm,
                other_wheels_safe=all(value < 100 for value in self.max_abs_erpm[1:]),
            )


def main() -> None:
    rclpy.init()
    node = WheelSafetyMonitor()
    WheelSafetyMonitor.emit("monitor_started")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
