"""Pure mecanum forward-kinematics helpers."""

import math
from typing import List, Sequence


def forward_mecanum(
    wheel_velocities: Sequence[float],
    wheel_radius: float,
    wheelbase_x: float,
    wheelbase_y: float,
    linear_x_scale: float = 1.0,
    linear_y_scale: float = 1.0,
    angular_z_scale: float = 1.0,
) -> List[float]:
    """Return [vx, vy, wz] from wheels ordered [FL, FR, RL, RR]."""

    if len(wheel_velocities) != 4:
        raise ValueError('Exactly four wheel velocities are required')

    wheels = [float(value) for value in wheel_velocities]
    values = wheels + [
        wheel_radius,
        wheelbase_x,
        wheelbase_y,
        linear_x_scale,
        linear_y_scale,
        angular_z_scale,
    ]

    if not all(math.isfinite(value) for value in values):
        raise ValueError('Wheel velocities and geometry must be finite')
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be greater than zero')
    if wheelbase_x <= 0.0 or wheelbase_y <= 0.0:
        raise ValueError(
            'wheelbase_x and wheelbase_y must be greater than zero'
        )
    if (
        linear_x_scale == 0.0
        or linear_y_scale == 0.0
        or angular_z_scale == 0.0
    ):
        raise ValueError('Scale multipliers must be non-zero')

    fl, fr, rl, rr = wheels
    rotation_radius = 0.5 * (wheelbase_x + wheelbase_y)
    vx = wheel_radius * (fl + fr + rl + rr) / 4.0
    vy = wheel_radius * (-fl + fr + rl - rr) / 4.0
    wz = (
        wheel_radius * (-fl + fr - rl + rr)
        / (4.0 * rotation_radius)
    )

    return [
        vx * linear_x_scale,
        vy * linear_y_scale,
        wz * angular_z_scale,
    ]
