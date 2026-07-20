import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('agv_odom'),
        'config',
        'mecanum_odometry.yaml',
    )

    odometry_node = Node(
        package='agv_odom',
        executable='mecanum_odometry',
        name='mecanum_odometry',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        odometry_node,
    ])
