import math
import unittest

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


class TestRelativeMotion(unittest.TestCase):

    def test_normalize_angle_wraps_both_directions(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-1.5 * math.pi), 0.5 * math.pi)

    def test_quaternion_to_yaw(self):
        yaw = math.radians(90.0)
        self.assertAlmostEqual(
            quaternion_to_yaw(math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
            yaw,
        )

    def test_relative_pose_uses_start_heading(self):
        relative = relative_pose(
            start_x=2.0,
            start_y=3.0,
            start_yaw=math.pi / 2.0,
            current_x=2.0,
            current_y=4.0,
            current_yaw=math.pi / 2.0,
        )
        self.assertAlmostEqual(relative[0], 1.0)
        self.assertAlmostEqual(relative[1], 0.0)
        self.assertAlmostEqual(relative[2], 0.0)

    def test_body_frame_error_accounts_for_current_yaw(self):
        error = body_frame_error(1.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(error[0], 0.0, places=7)
        self.assertAlmostEqual(error[1], -1.0)

    def test_linear_command_is_constant_and_preserves_direction(self):
        command = constant_linear_command(
            error_x=3.0,
            error_y=4.0,
            speed=0.2,
            tolerance=0.01,
        )
        self.assertAlmostEqual(command[0], 0.12)
        self.assertAlmostEqual(command[1], 0.16)

    def test_linear_command_stops_inside_tolerance(self):
        self.assertEqual(
            constant_linear_command(0.005, 0.0, 0.2, 0.01),
            (0.0, 0.0),
        )

    def test_translation_reached_inside_tolerance(self):
        self.assertTrue(
            translation_target_reached(0.02, 0.0, 0.005, 0.0, 0.01)
        )

    def test_translation_reached_when_sample_passes_target(self):
        self.assertTrue(
            translation_target_reached(0.01, 0.001, -0.01, 0.001, 0.005)
        )

    def test_translation_not_reached_while_approaching(self):
        self.assertFalse(
            translation_target_reached(0.10, 0.01, 0.05, 0.01, 0.01)
        )

    def test_translation_not_reached_after_lateral_miss(self):
        self.assertFalse(
            translation_target_reached(0.01, 0.02, -0.01, 0.02, 0.005)
        )

    def test_rotation_command_uses_constant_speed(self):
        command = constant_angular_command(
            error=0.02,
            speed=0.2,
            tolerance=0.01,
        )
        self.assertAlmostEqual(command, 0.2)

    def test_rotation_reached_inside_tolerance(self):
        self.assertTrue(rotation_target_reached(0.02, 0.005, 0.01))

    def test_rotation_reached_when_sample_passes_target(self):
        self.assertTrue(rotation_target_reached(0.01, -0.01, 0.005))

    def test_rotation_not_reached_while_approaching(self):
        self.assertFalse(rotation_target_reached(0.10, 0.05, 0.01))

    def test_rotation_reached_at_configured_stop_lead(self):
        self.assertTrue(
            rotation_target_reached(
                math.radians(7.0),
                math.radians(5.0),
                math.radians(1.0),
                math.radians(5.0),
            )
        )

    def test_small_rotation_is_not_skipped_by_stop_lead(self):
        error = math.radians(5.0)
        self.assertFalse(
            rotation_target_reached(
                error,
                error,
                math.radians(1.0),
                math.radians(5.0),
            )
        )

    def test_heading_correction_remains_proportional(self):
        command = proportional_angular_command(
            error=0.02,
            gain=1.0,
            maximum_speed=0.3,
            tolerance=0.01,
        )
        self.assertAlmostEqual(command, 0.02)


if __name__ == '__main__':
    unittest.main()
