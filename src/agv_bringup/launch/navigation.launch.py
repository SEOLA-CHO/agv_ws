from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_file(package_name, *parts):
    return str(Path(get_package_share_directory(package_name)).joinpath(*parts))


def _workspace_file(*parts):
    prefix = Path(get_package_prefix('agv_bringup'))
    workspace = prefix.parent.parent
    return str(workspace.joinpath(*parts))


def _argument(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    mapping_launch = _package_file(
        'agv_bringup', 'launch', 'mapping.launch.py'
    )
    nav2_launch = _package_file(
        'nav2_bringup', 'launch', 'bringup_launch.py'
    )

    return LaunchDescription([
        _argument(
            'wheel_source', 'hardware',
            "Wheel-state source: 'sim' or 'hardware'.",
        ),
        _argument('agent_device', '', 'STM32 /dev/serial/by-id path.'),
        _argument('agent_baudrate', '115200', 'micro-ROS serial baudrate.'),
        _argument('lidar_device', '', 'LiDAR /dev/serial/by-id path.'),
        _argument('lidar_baudrate', '460800', 'RPLIDAR C1 baudrate.'),
        _argument('scan_topic', '/scan', 'LaserScan topic.'),
        _argument('use_lidar', 'true', 'Start the RPLIDAR C1 driver.'),
        _argument('use_rviz', 'true', 'Start RViz2 with the Nav2 view.'),
        _argument('use_sim_time', 'false', 'Use ROS simulation time.'),
        _argument(
            'map', _workspace_file('maps', 'agv_map_v4.yaml'),
            'Absolute path to the static map YAML file.',
        ),
        _argument(
            'params_file',
            _package_file('agv_bringup', 'config', 'nav2_params.yaml'),
            'Nav2 parameter file.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'wheel_source': LaunchConfiguration('wheel_source'),
                'agent_device': LaunchConfiguration('agent_device'),
                'agent_baudrate': LaunchConfiguration('agent_baudrate'),
                'use_lidar': LaunchConfiguration('use_lidar'),
                'lidar_device': LaunchConfiguration('lidar_device'),
                'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_slam': 'false',
                'use_rviz': 'false',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
                'use_composition': 'True',
            }.items(),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', _package_file(
                    'nav2_bringup', 'rviz', 'nav2_default_view.rviz'
                )
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
