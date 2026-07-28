import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('uav_bringup')
    fc_port = LaunchConfiguration('fc_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fc_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Fancinnov_Mcontroller-v7_307838633430-if00'
            ),
            description='Mcontroller serial device or MAVLink proxy PTY link',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_static_tf',
            output='screen',
            arguments=[
                '--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar_link',
            ],
        ),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'lidar_link',
                'user_config_path': os.path.join(
                    bringup_share, 'config', 'mid360_live.json'
                ),
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),
        Node(
            package='spark_fast_lio',
            executable='spark_lio_mapping',
            name='lio_mapping',
            output='screen',
            remappings=[
                ('lidar', '/livox/lidar'),
                ('imu', '/livox/imu'),
            ],
            parameters=[
                os.path.join(bringup_share, 'config', 'spark_mid360_live.yaml'),
            ],
        ),
        Node(
            package='uav_bringup',
            executable='lio_to_fc_adapter',
            name='lio_to_fc_adapter',
            output='screen',
        ),
        Node(
            package='fcu_core',
            executable='fcu_bridge_001',
            name='fcu_bridge_001',
            output='screen',
            parameters=[{
                'USB_PORT': fc_port,
                'BANDRATE': 460800,
                'channel': 0,
                'offboard': False,
                'set_goal': False,
                'simple_target': True,
            }],
        ),
    ])
