from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value == 'true':
        return True
    if value == 'false':
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be 'true' or 'false'; got '{value}'."
    )


def _package_file(package_name, *parts):
    return str(Path(get_package_share_directory(package_name)).joinpath(*parts))


def _launch_components(context):
    wheel_source = LaunchConfiguration('wheel_source').perform(context)
    if wheel_source not in ('none', 'sim', 'hardware'):
        raise RuntimeError(
            "wheel_source must be exactly 'none', 'sim', or 'hardware'; "
            f"got '{wheel_source}'."
        )

    use_lidar = _boolean(context, 'use_lidar')
    use_slam = _boolean(context, 'use_slam')
    use_rviz = _boolean(context, 'use_rviz')
    use_description = _boolean(context, 'use_description')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool
    )

    actions = []

    if use_description:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _package_file(
                        'agv_description', 'launch', 'description.launch.py'
                    )
                )
            )
        )

    if wheel_source != 'none':
        actions.extend([
            Node(
                package='agv_control',
                executable='mecanum_controller',
                name='mecanum_controller',
                output='screen',
                parameters=[
                    _package_file(
                        'agv_control', 'config', 'mecanum_controller.yaml'
                    ),
                    {'use_sim_time': use_sim_time},
                ],
            ),
            Node(
                package='agv_odom',
                executable='mecanum_odometry',
                name='mecanum_odometry',
                output='screen',
                parameters=[
                    _package_file(
                        'agv_odom', 'config', 'mecanum_odometry.yaml'
                    ),
                    {
                        'odom_frame': LaunchConfiguration('odom_frame'),
                        'base_frame': LaunchConfiguration('base_frame'),
                        'use_sim_time': use_sim_time,
                    },
                ],
            ),
        ])

    if wheel_source == 'sim':
        actions.append(
            Node(
                package='agv_sim',
                executable='wheel_simulator',
                name='wheel_simulator',
                output='screen',
                parameters=[
                    _package_file(
                        'agv_sim', 'config', 'wheel_simulator.yaml'
                    ),
                    {'use_sim_time': use_sim_time},
                ],
            )
        )

    if wheel_source == 'hardware':
        agent_device = LaunchConfiguration('agent_device').perform(context)
        if not agent_device:
            raise RuntimeError(
                'agent_device must be an existing /dev/serial/by-id path in '
                'hardware mode.'
            )
        if not Path(agent_device).exists():
            raise RuntimeError(f'agent_device does not exist: {agent_device}')
        actions.append(
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                name='micro_ros_agent',
                output='screen',
                arguments=[
                    'serial',
                    '--dev',
                    LaunchConfiguration('agent_device'),
                    '-b',
                    LaunchConfiguration('agent_baudrate'),
                    '-v6',
                ],
            )
        )

    if use_lidar:
        lidar_launch = _package_file(
            'sllidar_ros2', 'launch', 'sllidar_c1_launch.py'
        )
        actions.append(
            GroupAction(actions=[
                SetRemap(src='scan', dst=LaunchConfiguration('scan_topic')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch),
                    launch_arguments={
                        'serial_port': LaunchConfiguration('lidar_device'),
                        'serial_baudrate': LaunchConfiguration(
                            'lidar_baudrate'
                        ),
                        'frame_id': LaunchConfiguration('laser_frame'),
                    }.items(),
                ),
            ])
        )

    if use_slam:
        actions.append(
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    _package_file(
                        'agv_bringup', 'config', 'slam_toolbox.yaml'
                    ),
                    {
                        'map_frame': LaunchConfiguration('map_frame'),
                        'odom_frame': LaunchConfiguration('odom_frame'),
                        'base_frame': LaunchConfiguration('base_frame'),
                        'scan_topic': LaunchConfiguration('scan_topic'),
                        'use_sim_time': use_sim_time,
                    },
                ],
                remappings=[('scan', LaunchConfiguration('scan_topic'))],
            )
        )

    if use_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-f', LaunchConfiguration('map_frame')],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

    return actions


def _argument(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    return LaunchDescription([
        _argument(
            'wheel_source',
            'none',
            "Wheel-state source: 'none', 'sim', or 'hardware'.",
        ),
        _argument('use_description', 'true', 'Start robot_state_publisher.'),
        _argument('use_lidar', 'false', 'Start the RPLIDAR C1 driver.'),
        _argument('use_slam', 'false', 'Start asynchronous slam_toolbox.'),
        _argument('use_rviz', 'false', 'Start RViz2.'),
        _argument('use_sim_time', 'false', 'Use ROS simulation time.'),
        _argument('agent_device', '', 'STM32 /dev/serial/by-id path.'),
        _argument('agent_baudrate', '115200', 'micro-ROS serial baudrate.'),
        _argument('lidar_device', '', 'LiDAR /dev/serial/by-id path.'),
        _argument('lidar_baudrate', '460800', 'RPLIDAR C1 baudrate.'),
        _argument('scan_topic', '/scan', 'LaserScan topic.'),
        _argument('map_frame', 'map', 'SLAM map frame.'),
        _argument('odom_frame', 'odom', 'Odometry frame.'),
        _argument('base_frame', 'base_footprint', 'Planar robot base frame.'),
        _argument('laser_frame', 'laser_frame', 'LiDAR frame.'),
        OpaqueFunction(function=_launch_components),
    ])
