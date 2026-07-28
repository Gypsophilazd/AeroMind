import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    map_path = LaunchConfiguration('map_path')
    source_path = LaunchConfiguration('source_path')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value='/home/nvidia/uav_ws/maps/demo/lab_map.pcd'),
        DeclareLaunchArgument(
            'source_path',
            default_value='/home/nvidia/uav_ws/maps/demo/lab_restart_scan.pcd'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_lidar_static_tf', output='screen',
            arguments=[
                '--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar_link',
            ],
        ),
        Node(
            package='uav_planning', executable='kiss_relocalizer',
            name='kiss_relocalizer', output='screen',
            parameters=[
                os.path.join(planning_share, 'config', 'kiss_relocalizer.yaml'),
                {'map_path': map_path},
            ],
        ),
        Node(
            package='uav_planning', executable='relocalization_demo_source',
            name='relocalization_demo_source', output='screen',
            parameters=[{'source_path': source_path}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(bringup_share, 'config', 'uav_final.rviz')],
        ),
    ])
