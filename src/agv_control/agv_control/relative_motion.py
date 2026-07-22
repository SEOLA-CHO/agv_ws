"""Pure relative-pose control helpers."""

import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(z: float, w: float) -> float:
    """Return planar yaw for a quaternion with zero roll and pitch."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def relative_pose(
    start_x: float,
    start_y: float,
    start_yaw: float,
    current_x: float,
    current_y: float,
    current_yaw: float,
) -> Tuple[float, float, float]:
    """Express the current pose in the captured start frame."""
    dx = current_x - start_x
    dy = current_y - start_y
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize_angle(current_yaw - start_yaw),
    )


def body_frame_error(
    error_x_start: float,
    error_y_start: float,
    relative_yaw: float,
) -> Tuple[float, float]:
    """Rotate a start-frame translation error into the current body frame."""
    cosine = math.cos(relative_yaw)
    sine = math.sin(relative_yaw)
    return (
        cosine * error_x_start + sine * error_y_start,
        -sine * error_x_start + cosine * error_y_start,
    )


def constant_linear_command(
    error_x: float,
    error_y: float,
    speed: float,
    tolerance: float,
) -> Tuple[float, float]:
    """Generate a constant-magnitude planar command toward the target."""
    distance = math.hypot(error_x, error_y)
    if distance <= tolerance:
        return 0.0, 0.0

    return speed * error_x / distance, speed * error_y / distance


def translation_target_reached(
    previous_error_x: float,
    previous_error_y: float,
    error_x: float,
    error_y: float,
    tolerance: float,
) -> bool:
    """Return whether a translation should stop without reversing direction.

    A constant-speed platform can move from one side of a small tolerance
    circle to the other between odometry samples.  A non-positive dot product
    between consecutive error vectors means that the target was passed between
    those samples, so commanding back toward it would only start an oscillation.
    """
    if math.hypot(error_x, error_y) <= tolerance:
        return True

    return (
        previous_error_x * error_x + previous_error_y * error_y <= 0.0
    )


def constant_angular_command(
    error: float,
    speed: float,
    tolerance: float,
) -> float:
    """Generate a constant-magnitude pure-rotation command."""
    if abs(error) <= tolerance:
        return 0.0
    return math.copysign(speed, error)


def rotation_target_reached(
    previous_error: float,
    error: float,
    tolerance: float,
    stop_lead: float = 0.0,
) -> bool:
    """Return whether a rotation should stop without reversing direction."""
    if abs(error) <= tolerance:
        return True

    if previous_error * error <= 0.0:
        return True

    stop_threshold = tolerance + stop_lead
    return (
        stop_lead > 0.0
        and abs(previous_error) > stop_threshold
        and abs(error) <= stop_threshold
    )


def proportional_angular_command(
    error: float,
    gain: float,
    maximum_speed: float,
    tolerance: float,
) -> float:
    """Generate a bounded yaw correction during translation."""
    if abs(error) <= tolerance:
        return 0.0

    magnitude = min(gain * abs(error), maximum_speed)
    return math.copysign(magnitude, error)
