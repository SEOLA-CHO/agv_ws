"""Launch file for the wheel simulator node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share_directory = get_package_share_directory("agv_sim")

    parameter_file = os.path.join(
        package_share_directory,
        "config",
        "wheel_simulator.yaml",
    )

    wheel_simulator_node = Node(
        package="agv_sim",
        executable="wheel_simulator",
        name="wheel_simulator",
        output="screen",
        parameters=[parameter_file],
    )

    return LaunchDescription([
        wheel_simulator_node,
    ])
