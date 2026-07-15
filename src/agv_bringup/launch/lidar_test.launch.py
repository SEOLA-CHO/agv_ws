from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


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


def _missing_dependency(message, strict):
    if strict:
        raise RuntimeError(message)
    get_logger('agv_bringup').warning(message + ' Skipping LiDAR.')
    return []


def _launch_lidar(context):
    strict = _parse_boolean(context, 'strict_dependencies')
    package_name = 'sllidar_ros2'
    launch_name = 'sllidar_c1_launch.py'

    try:
        package_share = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        return _missing_dependency(
            f"LiDAR dependency '{package_name}' is not installed; expected "
            f"launch/{launch_name}.",
            strict,
        )

    launch_path = package_share / 'launch' / launch_name
    if not launch_path.is_file():
        return _missing_dependency(
            f"LiDAR launch file is missing: {launch_path}.",
            strict,
        )

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
            'frame_id': LaunchConfiguration('laser_frame'),
        }.items(),
    )
    return [
        GroupAction(
            actions=[
                SetRemap(src='scan', dst=LaunchConfiguration('scan_topic')),
                include,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device connected to the RPLIDAR C1.',
        ),
        DeclareLaunchArgument(
            'lidar_serial_baudrate',
            default_value='460800',
            description='Serial baudrate used by the RPLIDAR C1.',
        ),
        DeclareLaunchArgument(
            'laser_frame',
            default_value='laser',
            description='Frame ID assigned to LaserScan messages.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Destination topic for the driver scan topic remapping.',
        ),
        DeclareLaunchArgument(
            'strict_dependencies',
            default_value='false',
            description='Fail instead of warning when sllidar_ros2 is unavailable.',
        ),
        OpaqueFunction(function=_launch_lidar),
    ])
