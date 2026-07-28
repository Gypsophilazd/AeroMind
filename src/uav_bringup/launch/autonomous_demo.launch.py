import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    fc_port = LaunchConfiguration('fc_port')
    flight_height = LaunchConfiguration('flight_height')
    map_path = LaunchConfiguration('map_path')
    use_relocalization = LaunchConfiguration('use_relocalization')
    enable_flight_commands = LaunchConfiguration('enable_flight_commands')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fc_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Fancinnov_Mcontroller-v7_307838633430-if00'),
        ),
        DeclareLaunchArgument('flight_height', default_value='1.0'),
        DeclareLaunchArgument(
            'map_path', default_value='/home/nvidia/uav_ws/maps/demo/lab_map.pcd'),
        DeclareLaunchArgument('use_relocalization', default_value='true'),
        DeclareLaunchArgument(
            'enable_flight_commands', default_value='false',
            description='Explicit safety gate for /mission_001 output'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'live_lio_to_fc_bench.launch.py')),
            launch_arguments={'fc_port': fc_port}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'live_ego_planning.launch.py')),
            launch_arguments={'fixed_altitude_m': flight_height}.items(),
        ),
        Node(
            package='uav_planning',
            executable='ego_trajectory_executor.py',
            name='ego_trajectory_executor',
            output='screen',
            parameters=[
                os.path.join(planning_share, 'config', 'ego_trajectory_executor.yaml'),
                {'enable_mission_output': enable_flight_commands},
            ],
        ),
        Node(
            package='uav_planning',
            executable='kiss_relocalizer',
            name='kiss_relocalizer',
            output='screen',
            condition=IfCondition(use_relocalization),
            parameters=[
                os.path.join(planning_share, 'config', 'kiss_relocalizer.yaml'),
                {'map_path': map_path},
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            output='screen',
            condition=UnlessCondition(use_relocalization),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(bringup_share, 'config', 'uav_final.rviz')],
        ),
    ])
