from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_SHARE = Path(__file__).resolve().parent.parent


def _parse_boolean(context, name):
    value = LaunchConfiguration(name).perform(context)
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be 'true' or 'false'; received '{value}'."
    )


def _resolve_optional_launch(package_name, launch_name, component, strict):
    expected = f"{package_name}/launch/{launch_name}"
    try:
        package_share = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        message = (
            f"{component} dependency '{package_name}' is not installed; expected {expected}."
        )
        if strict:
            raise RuntimeError(message)
        get_logger('agv_bringup').warning(message + f' Skipping {component}.')
        return None

    launch_path = package_share / 'launch' / launch_name
    if not launch_path.is_file():
        message = f"{component} launch file is missing: {launch_path}."
        if strict:
            raise RuntimeError(message)
        get_logger('agv_bringup').warning(message + f' Skipping {component}.')
        return None
    return launch_path


def _package_available(package_name, component, strict):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError:
        message = f"{component} dependency '{package_name}' is not installed."
        if strict:
            raise RuntimeError(message)
        get_logger('agv_bringup').warning(message + f' Skipping {component}.')
        return False
    return True


def _include_team_launch(package_name, launch_name, component, strict, arguments=None):
    launch_path = _resolve_optional_launch(
        package_name, launch_name, component, strict
    )
    if launch_path is None:
        return None
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(arguments or {}).items(),
    )


def _launch_selected_components(context):
    option_names = (
        'use_lidar',
        'use_description',
        'use_sim_wheels',
        'use_odometry',
        'use_slam',
        'use_rviz',
    )
    enabled = {name: _parse_boolean(context, name) for name in option_names}
    strict = _parse_boolean(context, 'strict_dependencies')

    if not any(enabled.values()):
        return [LogInfo(msg='No mapping components enabled; bringup skeleton is ready.')]

    actions = []
    if enabled['use_lidar']:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(PACKAGE_SHARE / 'launch' / 'lidar_test.launch.py')
                ),
                launch_arguments={
                    'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
                    'lidar_serial_baudrate': LaunchConfiguration(
                        'lidar_serial_baudrate'
                    ),
                    'laser_frame': LaunchConfiguration('laser_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'strict_dependencies': LaunchConfiguration('strict_dependencies'),
                }.items(),
            )
        )

    if enabled['use_description']:
        action = _include_team_launch(
            'agv_description',
            'description.launch.py',
            'robot description',
            strict,
            {
                'base_frame': LaunchConfiguration('base_frame'),
                'laser_frame': LaunchConfiguration('laser_frame'),
            },
        )
        if action is not None:
            actions.append(action)

    if enabled['use_sim_wheels']:
        action = _include_team_launch(
            'agv_sim',
            'simulator.launch.py',
            'wheel simulator',
            strict,
        )
        if action is not None:
            actions.append(action)

    if enabled['use_odometry']:
        action = _include_team_launch(
            'agv_odometry',
            'odometry.launch.py',
            'odometry',
            strict,
            {
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            },
        )
        if action is not None:
            actions.append(action)

    if enabled['use_slam']:
        available = _package_available('slam_toolbox', 'SLAM', strict)
        if available:
            actions.append(
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        str(PACKAGE_SHARE / 'config' / 'slam_toolbox.yaml'),
                        {
                            'map_frame': LaunchConfiguration('map_frame'),
                            'odom_frame': LaunchConfiguration('odom_frame'),
                            'base_frame': LaunchConfiguration('base_frame'),
                            'scan_topic': LaunchConfiguration('scan_topic'),
                        },
                    ],
                    remappings=[('scan', LaunchConfiguration('scan_topic'))],
                )
            )

    if enabled['use_rviz']:
        available = _package_available('rviz2', 'RViz', strict)
        if available:
            actions.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=[
                        '-d',
                        str(PACKAGE_SHARE / 'rviz' / 'mapping.rviz'),
                        '-f',
                        LaunchConfiguration('map_frame'),
                    ],
                    remappings=[('/scan', LaunchConfiguration('scan_topic'))],
                )
            )
    return actions


def _argument(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    return LaunchDescription([
        _argument('use_lidar', 'false', 'Start the RPLIDAR C1 driver.'),
        _argument('use_description', 'false', 'Start the robot description launch.'),
        _argument('use_sim_wheels', 'false', 'Start the software wheel simulator.'),
        _argument('use_odometry', 'false', 'Start mecanum odometry.'),
        _argument('use_slam', 'false', 'Start asynchronous slam_toolbox mapping.'),
        _argument('use_rviz', 'false', 'Start RViz2 with the mapping configuration.'),
        _argument(
            'strict_dependencies',
            'false',
            'Fail when a selected package or launch file is unavailable.',
        ),
        _argument('lidar_serial_port', '/dev/ttyUSB0', 'RPLIDAR serial device.'),
        _argument('lidar_serial_baudrate', '460800', 'RPLIDAR C1 baudrate.'),
        _argument('scan_topic', '/scan', 'LaserScan topic used by LiDAR and SLAM.'),
        _argument('map_frame', 'map', 'Global map frame.'),
        _argument('odom_frame', 'odom', 'Local odometry frame.'),
        _argument('base_frame', 'base_footprint', 'Robot planar base frame.'),
        _argument('laser_frame', 'laser', 'LiDAR frame.'),
        OpaqueFunction(function=_launch_selected_components),
    ])
