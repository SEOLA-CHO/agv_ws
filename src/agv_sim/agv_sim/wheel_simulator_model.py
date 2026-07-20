"""Core calculation and validation logic for the wheel simulator."""

from __future__ import annotations

import math
from typing import Sequence


WHEEL_NAMES = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)
WHEEL_COUNT = len(WHEEL_NAMES)


class WheelCommandError(ValueError):
    """Raised when a wheel command message is invalid."""


class NonFiniteWheelCommandError(WheelCommandError):
    """Raised when a command contains NaN or infinity."""


class WheelSimulatorModel:
    """Simulate four wheel velocities using a first-order lag model."""

    def __init__(
        self,
        response_alpha: float = 0.1,
        max_wheel_speed: float = 16.36,
    ) -> None:
        self._validate_parameters(response_alpha, max_wheel_speed)

        self.response_alpha = float(response_alpha)
        self.max_wheel_speed = float(max_wheel_speed)

        self.command = [0.0] * WHEEL_COUNT
        self.state = [0.0] * WHEEL_COUNT

    @staticmethod
    def _validate_parameters(
        response_alpha: float,
        max_wheel_speed: float,
    ) -> None:
        """Validate model parameters."""

        if not math.isfinite(response_alpha):
            raise ValueError("response_alpha must be finite")

        if not 0.0 < response_alpha <= 1.0:
            raise ValueError("response_alpha must satisfy 0 < alpha <= 1")

        if not math.isfinite(max_wheel_speed):
            raise ValueError("max_wheel_speed must be finite")

        if max_wheel_speed <= 0.0:
            raise ValueError("max_wheel_speed must be greater than 0")

    def set_command(
        self,
        velocities: Sequence[float],
    ) -> list[float]:
        """
        Validate and store a fixed-order wheel command.

        The F446 contract is always [FL, FR, RL, RR], so there is no wheel-name
        lookup or reordering at this boundary.
        """

        if len(velocities) != WHEEL_COUNT:
            raise WheelCommandError(
                "the command must contain exactly four wheel velocities"
            )

        command = []

        for wheel_name, velocity in zip(WHEEL_NAMES, velocities):
            value = float(velocity)

            if not math.isfinite(value):
                self.stop_command()
                raise NonFiniteWheelCommandError(
                    f"non-finite velocity received for {wheel_name}"
                )

            command.append(self._clamp(value))

        self.command = command
        return self.command.copy()

    def _clamp(self, velocity: float) -> float:
        """Limit velocity to the configured maximum speed."""

        return max(
            -self.max_wheel_speed,
            min(velocity, self.max_wheel_speed),
        )

    def update(self) -> list[float]:
        """Advance the four simulated wheel states by one timer cycle."""

        for index in range(len(self.state)):
            current = self.state[index]
            target = self.command[index]

            self.state[index] = (
                current
                + self.response_alpha * (target - current)
            )

        return self.state.copy()

    def stop_command(self) -> None:
        """Set only the target command to zero for gradual stopping."""

        self.command = [0.0] * WHEEL_COUNT

    def hard_stop(self) -> None:
        """Immediately reset both target and current state to zero."""

        self.command = [0.0] * WHEEL_COUNT
        self.state = [0.0] * WHEEL_COUNT
