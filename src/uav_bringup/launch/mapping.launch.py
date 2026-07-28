import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    map_path = LaunchConfiguration('map_path')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value='/home/nvidia/uav_ws/maps/session_map.pcd'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'live_mid360_lio.launch.py')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ),
        Node(
            package='uav_planning',
            executable='registered_cloud_map_saver',
            name='registered_cloud_map_saver',
            output='screen',
            parameters=[
                os.path.join(planning_share, 'config', 'map_saver.yaml'),
                {'output_path': map_path},
            ],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(bringup_share, 'config', 'uav_final.rviz')],
        ),
    ])
