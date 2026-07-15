import math

import pytest

from agv_sim.wheel_simulator_model import (
    NonFiniteWheelCommandError,
    WheelCommandError,
    WheelSimulatorModel,
)


WHEEL_NAMES = [
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
]


def test_initial_state_is_zero():
    model = WheelSimulatorModel()

    assert model.command == [0.0, 0.0, 0.0, 0.0]
    assert model.state == [0.0, 0.0, 0.0, 0.0]


def test_first_order_lag_response():
    model = WheelSimulatorModel(
        response_alpha=0.1,
        max_wheel_speed=30.0,
    )

    model.set_command(
        WHEEL_NAMES,
        [10.0, 10.0, 10.0, 10.0],
    )

    assert model.update() == pytest.approx([1.0, 1.0, 1.0, 1.0])
    assert model.update() == pytest.approx([1.9, 1.9, 1.9, 1.9])
    assert model.update() == pytest.approx([2.71, 2.71, 2.71, 2.71])


def test_reorders_commands_using_wheel_names():
    model = WheelSimulatorModel()

    command = model.set_command(
        [
            "rear_right_wheel",
            "front_left_wheel",
            "rear_left_wheel",
            "front_right_wheel",
        ],
        [4.0, 1.0, 3.0, 2.0],
    )

    assert command == [1.0, 2.0, 3.0, 4.0]


def test_clamps_maximum_wheel_speed():
    model = WheelSimulatorModel(max_wheel_speed=30.0)

    command = model.set_command(
        WHEEL_NAMES,
        [50.0, -50.0, 40.0, -40.0],
    )

    assert command == [30.0, -30.0, 30.0, -30.0]


def test_rejects_mismatched_array_lengths():
    model = WheelSimulatorModel()

    with pytest.raises(WheelCommandError):
        model.set_command(
            WHEEL_NAMES,
            [10.0],
        )


def test_rejects_duplicate_wheel_names():
    model = WheelSimulatorModel()

    with pytest.raises(WheelCommandError):
        model.set_command(
            [
                "front_left_wheel",
                "front_left_wheel",
                "rear_left_wheel",
                "rear_right_wheel",
            ],
            [1.0, 2.0, 3.0, 4.0],
        )


def test_nan_command_sets_target_to_zero():
    model = WheelSimulatorModel()

    model.set_command(
        WHEEL_NAMES,
        [10.0, 10.0, 10.0, 10.0],
    )

    with pytest.raises(NonFiniteWheelCommandError):
        model.set_command(
            WHEEL_NAMES,
            [math.nan, 10.0, 10.0, 10.0],
        )

    assert model.command == [0.0, 0.0, 0.0, 0.0]


def test_stop_command_allows_gradual_stop():
    model = WheelSimulatorModel(response_alpha=0.1)

    model.command = [10.0, 10.0, 10.0, 10.0]
    model.state = [10.0, 10.0, 10.0, 10.0]

    model.stop_command()
    result = model.update()

    assert result == pytest.approx([9.0, 9.0, 9.0, 9.0])


def test_hard_stop_sets_command_and_state_to_zero():
    model = WheelSimulatorModel()

    model.command = [10.0, 10.0, 10.0, 10.0]
    model.state = [8.0, 8.0, 8.0, 8.0]

    model.hard_stop()

    assert model.command == [0.0, 0.0, 0.0, 0.0]
    assert model.state == [0.0, 0.0, 0.0, 0.0]
