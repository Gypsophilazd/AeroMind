import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    fixed_altitude = LaunchConfiguration('fixed_altitude_m')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fixed_altitude_m',
            default_value='1.0',
            description='Fixed odom-frame flight altitude used for every RViz 2D goal',
        ),
        Node(
            package='uav_planning',
            executable='thin_odom_adapter',
            name='thin_odom_adapter',
            output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'thin_odom_adapter.yaml')],
        ),
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            output='screen',
            parameters=[os.path.join(
                bringup_share, 'config', 'ego_acl_offline.yaml')],
            remappings=[
                ('odom_world', '/uav/planning/odometry'),
                ('grid_map/odom', '/uav/planning/odometry'),
                ('grid_map/cloud', '/cloud_registered'),
            ],
        ),
        Node(
            package='uav_planning',
            executable='ego_goal_adapter.py',
            name='ego_goal_adapter',
            output='screen',
            parameters=[os.path.join(
                planning_share, 'config', 'ego_goal_adapter.yaml'),
                {'fixed_altitude_m': fixed_altitude}],
        ),
    ])
