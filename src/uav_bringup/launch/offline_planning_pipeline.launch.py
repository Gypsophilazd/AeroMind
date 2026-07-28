from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    spark_share = get_package_share_directory('spark_fast_lio')

    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    play_bag = LaunchConfiguration('play_bag')
    start_ego = LaunchConfiguration('start_ego')

    spark = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        output='screen',
        remappings=[
            ('lidar', '/acl_jackal/lidar_points'),
            ('imu', '/acl_jackal/forward/imu'),
        ],
        parameters=[
            os.path.join(spark_share, 'config', 'velodyne_mit.yaml'),
            {
                'common.lidar_frame': 'acl_jackal2/velodyne_link',
                'common.imu_frame': 'acl_jackal2/forward_imu_optical_frame',
                'common.map_frame': 'odom',
                'common.base_frame': 'base',
                'common.visualization_frame': 'base',
                'gravity_alignment.enable_gravity_alignment': False,
                'verbose': False,
            },
        ],
    )

    dataset_extrinsic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='acl_jackal_lidar_extrinsic',
        output='screen',
        arguments=[
            '--x', '0.13', '--y', '0.0', '--z', '0.52',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base',
            '--child-frame-id', 'acl_jackal2/velodyne_link',
        ],
    )

    adapter = Node(
        package='uav_planning',
        executable='thin_odom_adapter',
        name='thin_odom_adapter',
        output='screen',
        parameters=[os.path.join(planning_share, 'config', 'thin_odom_adapter.yaml')],
    )

    ego = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'ego_acl_offline.yaml')],
        remappings=[
            ('odom_world', '/uav/planning/odometry'),
            ('grid_map/odom', '/uav/planning/odometry'),
            ('grid_map/cloud', '/cloud_registered'),
        ],
        condition=IfCondition(start_ego),
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--rate', bag_rate,
            '--disable-keyboard-controls',
        ],
        output='screen',
        condition=IfCondition(play_bag),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/nvidia/uav_ws/data/spark_fast_lio/10_14_acl_jackal',
        ),
        DeclareLaunchArgument('bag_rate', default_value='1.0'),
        DeclareLaunchArgument('play_bag', default_value='true'),
        DeclareLaunchArgument('start_ego', default_value='true'),
        dataset_extrinsic,
        spark,
        adapter,
        ego,
        TimerAction(period=2.0, actions=[bag_play]),
    ])
