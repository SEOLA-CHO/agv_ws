import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory(
        'agv_description'
    )
    simulator_share = get_package_share_directory(
        'agv_sim'
    )
    control_share = get_package_share_directory(
        'agv_control'
    )
    odometry_share = get_package_share_directory(
        'agv_odom'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'agv.urdf.xacro',
    )

    simulator_config = os.path.join(
        simulator_share,
        'config',
        'wheel_simulator.yaml',
    )
    control_config = os.path.join(
        control_share,
        'config',
        'mecanum_controller.yaml',
    )
    odometry_config = os.path.join(
        odometry_share,
        'config',
        'mecanum_odometry.yaml',
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
            parameters=[{
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='agv_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen',
            parameters=[control_config],
        ),

        Node(
            package='agv_sim',
            executable='wheel_simulator',
            name='wheel_simulator',
            output='screen',
            parameters=[simulator_config],
        ),

        Node(
            package='agv_odom',
            executable='mecanum_odometry',
            name='mecanum_odometry',
            output='screen',
            parameters=[odometry_config],
        ),
    ])
