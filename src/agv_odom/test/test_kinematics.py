import math
import unittest

from agv_odom.kinematics import forward_mecanum


RADIUS = 0.0762
WHEELBASE_X = 0.445
WHEELBASE_Y = 0.400


def calculate(wheels, **kwargs):
    return forward_mecanum(
        wheels,
        RADIUS,
        WHEELBASE_X,
        WHEELBASE_Y,
        **kwargs,
    )


class TestKinematics(unittest.TestCase):
    def assert_velocity_almost_equal(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value)

    def test_all_forward_positive_wheels_move_forward(self):
        self.assert_velocity_almost_equal(
            calculate([1.0, 1.0, 1.0, 1.0]),
            [RADIUS, 0.0, 0.0],
        )

    def test_fixed_order_lateral_pattern(self):
        self.assert_velocity_almost_equal(
            calculate([-1.0, 1.0, 1.0, -1.0]),
            [0.0, RADIUS, 0.0],
        )

    def test_positive_rotation_uses_ros_counter_clockwise_pattern(self):
        rotation_radius = 0.5 * (WHEELBASE_X + WHEELBASE_Y)
        wheels = [-1.0, 1.0, -1.0, 1.0]
        self.assert_velocity_almost_equal(
            calculate(wheels, angular_z_scale=1.0),
            [0.0, 0.0, RADIUS / rotation_radius],
        )

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            calculate([1.0, 1.0, 1.0])
        with self.assertRaises(ValueError):
            calculate([1.0, math.inf, 1.0, 1.0])


if __name__ == '__main__':
    unittest.main()
