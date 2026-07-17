import math
import unittest

from agv_odometry.kinematics import forward_mecanum


RADIUS = 0.0762
BASE = 0.445
TRACK = 0.400


def calculate(wheels, **kwargs):
    return forward_mecanum(
        wheels,
        RADIUS,
        BASE,
        TRACK,
        **kwargs,
    )


class TestForwardKinematics(unittest.TestCase):

    def assert_velocity_almost_equal(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value)

    def test_forward_velocity(self):
        self.assert_velocity_almost_equal(
            calculate([1.0, 1.0, 1.0, 1.0]),
            [RADIUS, 0.0, 0.0],
        )

    def test_lateral_velocity(self):
        self.assert_velocity_almost_equal(
            calculate([-1.0, 1.0, 1.0, -1.0]),
            [0.0, RADIUS, 0.0],
        )

    def test_rotation_velocity_and_configured_sign(self):
        lever_arm = 0.5 * (BASE + TRACK)
        wheels = [-1.0, 1.0, -1.0, 1.0]
        self.assert_velocity_almost_equal(
            calculate(wheels),
            [0.0, 0.0, RADIUS / lever_arm],
        )
        self.assert_velocity_almost_equal(
            calculate(wheels, wz_direction=-1.0),
            [0.0, 0.0, -RADIUS / lever_arm],
        )

    def test_invalid_wheel_count_and_non_finite_value_are_rejected(self):
        with self.assertRaises(ValueError):
            calculate([1.0, 1.0, 1.0])
        with self.assertRaises(ValueError):
            calculate([1.0, 1.0, math.inf, 1.0])


if __name__ == '__main__':
    unittest.main()
