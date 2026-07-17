"""Pure mecanum inverse-kinematics helpers."""

import math
from typing import List, Optional


def inverse_mecanum(
    vx: float,
    vy: float,
    wz: float,
    wheel_radius: float,
    wheel_base: float,
    wheel_track: float,
    rotation_direction: float = 1.0,
    max_wheel_speed: Optional[float] = None,
) -> List[float]:
    """Return wheel rad/s in the fixed order [FL, FR, RL, RR]."""
    values = (
        vx,
        vy,
        wz,
        wheel_radius,
        wheel_base,
        wheel_track,
        rotation_direction,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Mecanum inputs and geometry must be finite')
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be greater than zero')
    if wheel_base <= 0.0 or wheel_track <= 0.0:
        raise ValueError(
            'wheel_base and wheel_track must be greater than zero'
        )
    if rotation_direction == 0.0:
        raise ValueError('rotation_direction must be non-zero')
    if max_wheel_speed is not None:
        if not math.isfinite(max_wheel_speed) or max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed must be finite and positive')

    lever_arm = 0.5 * (wheel_base + wheel_track)
    corrected_wz = rotation_direction * wz
    wheel_speeds = [
        (vx - vy - lever_arm * corrected_wz) / wheel_radius,
        (vx + vy + lever_arm * corrected_wz) / wheel_radius,
        (vx + vy - lever_arm * corrected_wz) / wheel_radius,
        (vx - vy + lever_arm * corrected_wz) / wheel_radius,
    ]

    if max_wheel_speed is not None:
        peak_speed = max(abs(speed) for speed in wheel_speeds)
        if peak_speed > max_wheel_speed:
            scale = max_wheel_speed / peak_speed
            wheel_speeds = [speed * scale for speed in wheel_speeds]

    return wheel_speeds
