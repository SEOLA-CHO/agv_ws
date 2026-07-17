"""Launch the cmd_vel-to-wheel-command controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('agv_control'),
        'config',
        'mecanum_controller.yaml',
    )
    return LaunchDescription([
        Node(
            package='agv_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen',
            parameters=[config_file],
        ),
    ])
