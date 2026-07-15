from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration


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


def _optional_include(package_name, launch_name, strict, launch_arguments=None):
    expected = f"{package_name}/launch/{launch_name}"
    try:
        package_share = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        message = f"Optional integration package '{package_name}' is missing; expected {expected}."
        if strict:
            raise RuntimeError(message)
        get_logger('agv_bringup').warning(message + ' Skipping this component.')
        return None

    launch_path = package_share / 'launch' / launch_name
    if not launch_path.is_file():
        message = f"Optional integration launch file is missing: {launch_path}."
        if strict:
            raise RuntimeError(message)
        get_logger('agv_bringup').warning(message + ' Skipping this component.')
        return None

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _launch_simulator_stack(context):
    strict = _parse_boolean(context, 'strict_dependencies')
    actions = [
        _optional_include('agv_sim', 'simulator.launch.py', strict),
        _optional_include(
            'agv_odometry',
            'odometry.launch.py',
            strict,
            {
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            },
        ),
    ]
    return [action for action in actions if action is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument(
            'strict_dependencies',
            default_value='false',
            description='Fail instead of warning when a team package is unavailable.',
        ),
        OpaqueFunction(function=_launch_simulator_stack),
    ])
