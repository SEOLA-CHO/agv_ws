"""Pure mecanum forward-kinematics helpers."""

import math
from typing import List, Sequence


def forward_mecanum(
    wheel_velocities: Sequence[float],
    wheel_radius: float,
    wheel_base: float,
    wheel_track: float,
    vx_direction: float = 1.0,
    vy_direction: float = 1.0,
    wz_direction: float = 1.0,
) -> List[float]:
    """Return [vx, vy, wz] from wheels ordered [FL, FR, RL, RR]."""
    if len(wheel_velocities) != 4:
        raise ValueError('Exactly four wheel velocities are required')

    wheels = [float(value) for value in wheel_velocities]
    values = wheels + [
        wheel_radius,
        wheel_base,
        wheel_track,
        vx_direction,
        vy_direction,
        wz_direction,
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Wheel velocities and geometry must be finite')
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be greater than zero')
    if wheel_base <= 0.0 or wheel_track <= 0.0:
        raise ValueError(
            'wheel_base and wheel_track must be greater than zero'
        )
    if vx_direction == 0.0 or vy_direction == 0.0 or wz_direction == 0.0:
        raise ValueError('Direction multipliers must be non-zero')

    fl, fr, rl, rr = wheels
    lever_arm = 0.5 * (wheel_base + wheel_track)
    vx = wheel_radius * (fl + fr + rl + rr) / 4.0
    vy = wheel_radius * (-fl + fr + rl - rr) / 4.0
    wz = wheel_radius * (-fl + fr - rl + rr) / (4.0 * lever_arm)
    return [
        vx * vx_direction,
        vy * vy_direction,
        wz * wz_direction,
    ]
