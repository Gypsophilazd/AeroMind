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
    evidence_path = LaunchConfiguration('evidence_path')
    relocalization_map = LaunchConfiguration('map_path')
    relocalization_source = LaunchConfiguration('source_path')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'evidence_path',
            default_value='/tmp/final_navigation_evidence.json'),
        DeclareLaunchArgument(
            'map_path', default_value='/home/nvidia/uav_ws/maps/demo/lab_map.pcd'),
        DeclareLaunchArgument(
            'source_path',
            default_value='/home/nvidia/uav_ws/maps/demo/lab_restart_scan.pcd'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'software_navigation_pipeline.launch.py')),
            launch_arguments={'evidence_path': evidence_path}.items(),
        ),
        Node(
            package='uav_planning', executable='kiss_relocalizer',
            name='kiss_relocalizer', output='screen',
            parameters=[
                os.path.join(planning_share, 'config', 'kiss_relocalizer.yaml'),
                {
                    'map_path': relocalization_map,
                    'cloud_topic': '/relocalization/test_cloud',
                },
            ],
        ),
        Node(
            package='uav_planning', executable='relocalization_demo_source',
            name='relocalization_test_source', output='screen',
            parameters=[{
                'source_path': relocalization_source,
                'cloud_topic': '/relocalization/test_cloud',
                'odom_topic': '/relocalization/test_odometry',
                'publish_pose': False,
                'publish_marker': False,
            }],
        ),
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
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', os.path.join(bringup_share, 'config', 'uav_final.rviz')],
        ),
    ])
