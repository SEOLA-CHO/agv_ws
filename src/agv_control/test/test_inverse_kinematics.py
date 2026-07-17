import math
import unittest

from agv_control.kinematics import inverse_mecanum


RADIUS = 0.0762
BASE = 0.445
TRACK = 0.400


def calculate(vx=0.0, vy=0.0, wz=0.0, **kwargs):
    return inverse_mecanum(
        vx,
        vy,
        wz,
        RADIUS,
        BASE,
        TRACK,
        **kwargs,
    )


class TestInverseKinematics(unittest.TestCase):

    def assert_wheels_almost_equal(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value)

    def test_forward_order_is_fl_fr_rl_rr(self):
        self.assert_wheels_almost_equal(
            calculate(vx=1.0),
            [1.0 / RADIUS] * 4,
        )

    def test_lateral_pattern(self):
        speed = 1.0 / RADIUS
        self.assert_wheels_almost_equal(
            calculate(vy=1.0),
            [-speed, speed, speed, -speed],
        )

    def test_rotation_direction_matches_existing_robot_configuration(self):
        lever_arm = 0.5 * (BASE + TRACK)
        speed = lever_arm / RADIUS
        self.assert_wheels_almost_equal(
            calculate(wz=1.0, rotation_direction=-1.0),
            [speed, -speed, speed, -speed],
        )

    def test_limit_scales_all_wheels_proportionally(self):
        unrestricted = calculate(vx=2.0, vy=1.0, wz=0.5)
        limited = calculate(
            vx=2.0,
            vy=1.0,
            wz=0.5,
            max_wheel_speed=10.0,
        )
        scale = 10.0 / max(abs(value) for value in unrestricted)
        self.assert_wheels_almost_equal(
            limited,
            [value * scale for value in unrestricted],
        )
        self.assertAlmostEqual(max(abs(value) for value in limited), 10.0)

    def test_non_finite_command_is_rejected(self):
        with self.assertRaises(ValueError):
            calculate(vx=math.nan)


if __name__ == '__main__':
    unittest.main()
