import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    odom_share = get_package_share_directory('agv_odom')
    description_share = get_package_share_directory(
        'agv_description'
    )

    odom_config = os.path.join(
        odom_share,
        'config',
        'odom.yaml',
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'agv.urdf.xacro',
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                }
            ],
        ),

        Node(
            package='agv_odom',
            executable='mecanum_odometry',
            name='mecanum_odometry',
            output='screen',
            parameters=[odom_config],
        ),
    ])
