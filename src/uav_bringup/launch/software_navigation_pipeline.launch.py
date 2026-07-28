import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    planning_share = get_package_share_directory('uav_planning')
    evidence_path = LaunchConfiguration('evidence_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'evidence_path',
            default_value='/tmp/final_navigation_evidence.json',
        ),
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            output='screen',
            parameters=[os.path.join(
                bringup_share, 'config', 'ego_final_sim.yaml')],
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
                planning_share, 'config', 'ego_goal_adapter.yaml')],
        ),
        Node(
            package='uav_planning',
            executable='ego_trajectory_executor.py',
            name='ego_trajectory_executor',
            output='screen',
            parameters=[
                os.path.join(
                    planning_share, 'config', 'ego_trajectory_executor.yaml'),
                {'enable_mission_output': True},
            ],
        ),
        Node(
            package='uav_planning',
            executable='final_navigation_simulation.py',
            name='final_navigation_simulation',
            output='screen',
            parameters=[{'evidence_path': evidence_path}],
        ),
    ])
