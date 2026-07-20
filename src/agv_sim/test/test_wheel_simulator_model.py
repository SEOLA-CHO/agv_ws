import math
import unittest

from agv_sim.wheel_simulator_model import (
    NonFiniteWheelCommandError,
    WheelCommandError,
    WheelSimulatorModel,
)


class TestWheelSimulatorModel(unittest.TestCase):
    def assert_vector_almost_equal(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value)

    def test_initial_state_is_zero(self):
        model = WheelSimulatorModel()

        self.assertEqual(model.command, [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(model.state, [0.0, 0.0, 0.0, 0.0])

    def test_first_order_lag_response(self):
        model = WheelSimulatorModel(
            response_alpha=0.1,
            max_wheel_speed=16.36,
        )
        model.set_command([10.0, 10.0, 10.0, 10.0])

        self.assert_vector_almost_equal(
            model.update(), [1.0, 1.0, 1.0, 1.0]
        )
        self.assert_vector_almost_equal(
            model.update(), [1.9, 1.9, 1.9, 1.9]
        )
        self.assert_vector_almost_equal(
            model.update(), [2.71, 2.71, 2.71, 2.71]
        )

    def test_preserves_fixed_fl_fr_rl_rr_order(self):
        model = WheelSimulatorModel()

        command = model.set_command([1.0, 2.0, 3.0, 4.0])

        self.assertEqual(command, [1.0, 2.0, 3.0, 4.0])

    def test_clamps_f446_maximum_wheel_speed(self):
        model = WheelSimulatorModel(max_wheel_speed=16.36)

        command = model.set_command([50.0, -50.0, 20.0, -20.0])

        self.assertEqual(command, [16.36, -16.36, 16.36, -16.36])

    def test_rejects_wrong_wheel_count(self):
        model = WheelSimulatorModel()

        with self.assertRaises(WheelCommandError):
            model.set_command([10.0])

    def test_nan_command_sets_target_to_zero(self):
        model = WheelSimulatorModel()
        model.set_command([10.0, 10.0, 10.0, 10.0])

        with self.assertRaises(NonFiniteWheelCommandError):
            model.set_command([math.nan, 10.0, 10.0, 10.0])

        self.assertEqual(model.command, [0.0, 0.0, 0.0, 0.0])

    def test_stop_command_allows_gradual_stop(self):
        model = WheelSimulatorModel(response_alpha=0.1)
        model.command = [10.0, 10.0, 10.0, 10.0]
        model.state = [10.0, 10.0, 10.0, 10.0]

        model.stop_command()
        result = model.update()

        self.assert_vector_almost_equal(result, [9.0, 9.0, 9.0, 9.0])

    def test_hard_stop_sets_command_and_state_to_zero(self):
        model = WheelSimulatorModel()
        model.command = [10.0, 10.0, 10.0, 10.0]
        model.state = [8.0, 8.0, 8.0, 8.0]

        model.hard_stop()

        self.assertEqual(model.command, [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(model.state, [0.0, 0.0, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
