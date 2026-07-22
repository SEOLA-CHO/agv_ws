"""Launch wheel odometry with the integration-branch launch contract."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('agv_odometry'),
        'config',
        'mecanum_odometry.yaml',
    )
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    publish_tf = LaunchConfiguration('publish_tf')
    return LaunchDescription([
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='agv_odometry',
            executable='mecanum_odometry',
            name='mecanum_odometry',
            output='screen',
            parameters=[
                config_file,
                {
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'publish_tf': ParameterValue(publish_tf, value_type=bool),
                },
            ],
        ),
    ])
