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


class WheelCommandError(ValueError):
    """Raised when a wheel command message is invalid."""


class NonFiniteWheelCommandError(WheelCommandError):
    """Raised when a command contains NaN or infinity."""


class WheelSimulatorModel:
    """Simulates four wheel velocities using a first-order lag model."""

    def __init__(
        self,
        response_alpha: float = 0.1,
        max_wheel_speed: float = 30.0,
    ) -> None:
        self._validate_parameters(response_alpha, max_wheel_speed)

        self.response_alpha = float(response_alpha)
        self.max_wheel_speed = float(max_wheel_speed)

        self.command = [0.0, 0.0, 0.0, 0.0]
        self.state = [0.0, 0.0, 0.0, 0.0]

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
        names: Sequence[str],
        velocities: Sequence[float],
    ) -> list[float]:
        """
        Validate and store a wheel command.

        The received array order is ignored. Values are reordered using
        JointState.name into [FL, FR, RL, RR].
        """

        if len(names) != len(velocities):
            raise WheelCommandError(
                "name and velocity arrays must have the same length"
            )

        if len(names) != len(WHEEL_NAMES):
            raise WheelCommandError(
                "the command must contain exactly four wheel names"
            )

        if len(set(names)) != len(names):
            raise WheelCommandError("duplicate wheel names are not allowed")

        unknown_names = set(names) - set(WHEEL_NAMES)
        if unknown_names:
            raise WheelCommandError(
                f"unknown wheel names: {sorted(unknown_names)}"
            )

        missing_names = set(WHEEL_NAMES) - set(names)
        if missing_names:
            raise WheelCommandError(
                f"missing wheel names: {sorted(missing_names)}"
            )

        velocity_by_name: dict[str, float] = {}

        for name, velocity in zip(names, velocities):
            value = float(velocity)

            if not math.isfinite(value):
                self.stop_command()
                raise NonFiniteWheelCommandError(
                    f"non-finite velocity received for {name}"
                )

            velocity_by_name[name] = value

        ordered_command = [
            self._clamp(velocity_by_name[name])
            for name in WHEEL_NAMES
        ]

        self.command = ordered_command
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

        self.command = [0.0, 0.0, 0.0, 0.0]

    def hard_stop(self) -> None:
        """Immediately reset both target and current state to zero."""

        self.command = [0.0, 0.0, 0.0, 0.0]
        self.state = [0.0, 0.0, 0.0, 0.0]