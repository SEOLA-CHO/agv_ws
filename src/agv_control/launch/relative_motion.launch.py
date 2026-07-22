from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agv_control'),
        'config',
        'relative_motion_controller.yaml',
    )
    return LaunchDescription([
        Node(
            package='agv_control',
            executable='relative_motion_controller',
            name='relative_motion_controller',
            output='screen',
            parameters=[config],
        ),
    ])
