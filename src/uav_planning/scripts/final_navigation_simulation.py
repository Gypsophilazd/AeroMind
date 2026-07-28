#!/usr/bin/env python3
"""Lightweight closed-loop environment and final navigation evidence probe."""

import json
import math
import os
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import TransformBroadcaster
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker


def quaternion_from_yaw(yaw):
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return 0.0, 0.0, z, w


class FinalNavigationSimulation(Node):
    def __init__(self):
        super().__init__('final_navigation_simulation')
        self.goal_x = float(self.declare_parameter('goal_x', 5.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)
        self.goal_z = float(self.declare_parameter('goal_z', 1.0).value)
        self.goal_yaw = float(self.declare_parameter('goal_yaw', math.pi / 2.0).value)
        self.goal_delay = float(self.declare_parameter('goal_delay_sec', 3.0).value)
        self.evidence_path = self.declare_parameter(
            'evidence_path', '/tmp/final_navigation_evidence.json').value
        self.started = time.monotonic()
        self.last_update = self.started
        self.goal_sent = False
        self.position = [0.0, 0.0, self.goal_z]
        self.velocity = [0.0, 0.0, 0.0]
        self.command = None
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.tf_broadcaster = TransformBroadcaster(self)
        self.counts = {
            'odometry_published': 0, 'cloud_published': 0,
            'unified_goals_published': 0, 'normalized_goals_observed': 0,
            'ego_goals_observed': 0, 'inflated_cloud_observed': 0,
            'bsplines_observed': 0, 'trajectory_samples_observed': 0,
            'mission_setpoints_observed': 0,
        }
        self.nonfinite_samples = 0
        self.max_continuous_step_m = 0.0
        self.max_mission_mapping_error = 0.0
        self.previous_sample_position = None
        self.previous_sample_time = None
        self.latest_sample = None
        self.latest_elapsed = 0.0
        self.latest_duration = 0.0
        self.closest_obstacle_distance_xy = float('inf')
        self.max_lateral_avoidance_m = 0.0
        self.max_vertical_avoidance_m = 0.0
        self.final_setpoint_error_m = float('inf')

        self.odom_pub = self.create_publisher(
            Odometry, '/uav/planning/odometry', 10)
        self.raw_odom_pub = self.create_publisher(Odometry, '/odometry', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/cloud_registered', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.obstacle_marker_pub = self.create_publisher(
            Marker, '/uav/planning/simulation_obstacle', 1)
        self.vehicle_marker_pub = self.create_publisher(
            Marker, '/uav/planning/simulation_vehicle', 1)
        self.evidence_marker_pub = self.create_publisher(
            Marker, '/uav/planning/simulation_evidence', 1)

        self.create_subscription(
            PoseStamped, '/uav/planning/commanded_setpoint', self.command_callback, 10)
        self.create_subscription(
            PoseStamped, '/uav/planning/goal', self.normalized_goal_callback, 10)
        self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.ego_goal_callback, 10)
        self.create_subscription(
            PointCloud2, '/grid_map/occupancy_inflate', self.inflated_callback, 10)
        self.create_subscription(Bspline, '/planning/bspline', self.bspline_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/uav/planning/trajectory_sample',
            self.sample_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/mission_001', self.mission_callback, 10)
        self.create_timer(0.02, self.update)
        self.create_timer(0.10, self.publish_cloud)
        self.create_timer(0.50, self.publish_scene_markers)
        self.create_timer(1.0, self.write_evidence)
        self.get_logger().info(
            f'software-only test: goal=({self.goal_x:.1f}, {self.goal_y:.1f}, '
            f'{self.goal_z:.1f}), obstacle center=(2.5, 0.0, 1.0)')

    def command_callback(self, message):
        self.command = [
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ]

    def normalized_goal_callback(self, _message):
        self.counts['normalized_goals_observed'] += 1

    def ego_goal_callback(self, _message):
        self.counts['ego_goals_observed'] += 1

    def inflated_callback(self, message):
        if message.width * message.height > 0:
            self.counts['inflated_cloud_observed'] += 1

    def bspline_callback(self, _message):
        self.counts['bsplines_observed'] += 1

    def sample_callback(self, message):
        self.counts['trajectory_samples_observed'] += 1
        if len(message.data) != 13 or not all(math.isfinite(value) for value in message.data):
            self.nonfinite_samples += 1
            return
        elapsed, duration = message.data[0:2]
        position = list(message.data[2:5])
        if self.previous_sample_position is not None and elapsed >= self.previous_sample_time:
            step = math.dist(position, self.previous_sample_position)
            self.max_continuous_step_m = max(self.max_continuous_step_m, step)
        self.previous_sample_position = position
        self.previous_sample_time = elapsed
        self.latest_sample = list(message.data)
        self.latest_elapsed = elapsed
        self.latest_duration = duration
        distance = math.hypot(position[0] - 2.5, position[1])
        self.closest_obstacle_distance_xy = min(
            self.closest_obstacle_distance_xy, distance)
        if 1.7 <= position[0] <= 3.3:
            self.max_lateral_avoidance_m = max(
                self.max_lateral_avoidance_m, abs(position[1]))
            self.max_vertical_avoidance_m = max(
                self.max_vertical_avoidance_m, abs(position[2] - self.goal_z))
        self.final_setpoint_error_m = math.dist(
            position, [self.goal_x, self.goal_y, self.goal_z])

    def mission_callback(self, message):
        self.counts['mission_setpoints_observed'] += 1
        if self.latest_sample is None or len(message.data) != 11:
            return
        px, py, pz = self.latest_sample[2:5]
        yaw = self.latest_sample[11]
        expected = [-yaw, 0.0, px, -py, -pz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.max_mission_mapping_error = max(
            self.max_mission_mapping_error,
            max(abs(float(a) - float(b)) for a, b in zip(message.data, expected)))

    def update(self):
        now = time.monotonic()
        dt = min(0.1, now - self.last_update)
        self.last_update = now
        if self.command is not None:
            error = [self.command[i] - self.position[i] for i in range(3)]
            desired_velocity = [max(-1.5, min(1.5, 4.0 * value)) for value in error]
            for index in range(3):
                self.velocity[index] += 0.35 * (
                    desired_velocity[index] - self.velocity[index])
                self.position[index] += self.velocity[index] * dt

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, \
            odom.pose.pose.position.z = self.position
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, \
            odom.twist.twist.linear.z = self.velocity
        self.odom_pub.publish(odom)
        self.raw_odom_pub.publish(odom)
        self.counts['odometry_published'] += 1

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = self.position[2]
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

        if not self.path.poses or now - getattr(self, '_last_path', 0.0) >= 0.1:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
            self._last_path = now

        if not self.goal_sent and now - self.started >= self.goal_delay:
            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = 'odom'
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.position.z = 0.0  # RViz 2D tool semantics; adapter replaces this.
            qx, qy, qz, qw = quaternion_from_yaw(self.goal_yaw)
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.counts['unified_goals_published'] += 1
            self.get_logger().info('published representative /goal_pose')

    @staticmethod
    def obstacle_points():
        points = []
        # Dense rectangular obstacle spanning the straight-line route.
        for x_index in range(9):
            x = 2.1 + 0.1 * x_index
            for y_index in range(15):
                y = -0.7 + 0.1 * y_index
                for z_index in range(25):
                    z = 0.2 * z_index
                    if x_index in (0, 8) or y_index in (0, 14):
                        points.append((x, y, z))
        return points

    def publish_cloud(self):
        points = self.obstacle_points()
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.data = b''.join(struct.pack('<fff', *point) for point in points)
        message.is_dense = True
        self.cloud_pub.publish(message)
        self.counts['cloud_published'] += 1

    def publish_scene_markers(self):
        stamp = self.get_clock().now().to_msg()
        obstacle = Marker()
        obstacle.header.stamp = stamp
        obstacle.header.frame_id = 'odom'
        obstacle.ns = 'final_demo_obstacle'
        obstacle.id = 0
        obstacle.type = Marker.CUBE
        obstacle.action = Marker.ADD
        obstacle.pose.position.x = 2.5
        obstacle.pose.position.z = 2.4
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = 0.8
        obstacle.scale.y = 1.4
        obstacle.scale.z = 4.8
        obstacle.color.r = 0.9
        obstacle.color.g = 0.15
        obstacle.color.b = 0.1
        obstacle.color.a = 0.35
        self.obstacle_marker_pub.publish(obstacle)

        vehicle = Marker()
        vehicle.header = obstacle.header
        vehicle.ns = 'final_demo_vehicle'
        vehicle.id = 0
        vehicle.type = Marker.SPHERE
        vehicle.action = Marker.ADD
        vehicle.pose.position.x, vehicle.pose.position.y, vehicle.pose.position.z = self.position
        vehicle.pose.orientation.w = 1.0
        vehicle.scale.x = 0.35
        vehicle.scale.y = 0.35
        vehicle.scale.z = 0.18
        vehicle.color.r = 0.2
        vehicle.color.g = 0.9
        vehicle.color.b = 0.25
        vehicle.color.a = 1.0
        self.vehicle_marker_pub.publish(vehicle)

        evidence = Marker()
        evidence.header = obstacle.header
        evidence.ns = 'final_demo_evidence'
        evidence.id = 0
        evidence.type = Marker.TEXT_VIEW_FACING
        evidence.action = Marker.ADD
        evidence.pose.position.x = 2.5
        evidence.pose.position.y = -2.3
        evidence.pose.position.z = 2.3
        evidence.pose.orientation.w = 1.0
        evidence.scale.z = 0.32
        evidence.color.r = 1.0
        evidence.color.g = 1.0
        evidence.color.b = 1.0
        evidence.color.a = 1.0
        final_error = self.final_setpoint_error_m
        final_text = 'waiting' if not math.isfinite(final_error) else f'{final_error:.2f} m'
        evidence.text = (
            f'Final software-only navigation\n'
            f'B-splines: {self.counts["bsplines_observed"]}  '
            f'/mission_001: {self.counts["mission_setpoints_observed"]}\n'
            f'finite errors: {self.nonfinite_samples}  final error: {final_text}')
        self.evidence_marker_pub.publish(evidence)

    def evidence(self):
        closest = self.closest_obstacle_distance_xy
        result = {
            'test_kind': 'software_only_no_fcu_bridge',
            'goal_xyz': [self.goal_x, self.goal_y, self.goal_z],
            'obstacle_center_xyz': [2.5, 0.0, 2.4],
            'counts': self.counts,
            'nonfinite_trajectory_samples': self.nonfinite_samples,
            'max_continuous_setpoint_step_m': self.max_continuous_step_m,
            'max_mission_mapping_error': self.max_mission_mapping_error,
            'closest_sample_to_obstacle_center_xy_m': (
                closest if math.isfinite(closest) else None),
            'max_lateral_avoidance_near_obstacle_m': self.max_lateral_avoidance_m,
            'max_vertical_avoidance_near_obstacle_m': self.max_vertical_avoidance_m,
            'final_setpoint_error_m': (
                self.final_setpoint_error_m
                if math.isfinite(self.final_setpoint_error_m) else None),
            'latest_trajectory_elapsed_sec': self.latest_elapsed,
            'latest_trajectory_duration_sec': self.latest_duration,
        }
        result['checks'] = {
            'odom_stream': self.counts['odometry_published'] >= 50,
            'cloud_ingested': self.counts['inflated_cloud_observed'] > 0,
            'goal_adapter': (
                self.counts['normalized_goals_observed'] > 0 and
                self.counts['ego_goals_observed'] > 0),
            'bspline_generated': self.counts['bsplines_observed'] > 0,
            'executor_stream': self.counts['trajectory_samples_observed'] >= 20,
            'mission_stream': self.counts['mission_setpoints_observed'] >= 20,
            'all_finite': self.nonfinite_samples == 0,
            'smooth_position_steps': 0.0 < self.max_continuous_step_m < 0.20,
            'coordinate_mapping': self.max_mission_mapping_error < 1e-5,
            'final_convergence': (
                math.isfinite(self.final_setpoint_error_m) and
                self.final_setpoint_error_m < 0.20),
            'obstacle_avoidance': (
                (self.max_lateral_avoidance_m > 0.75 or
                 self.max_vertical_avoidance_m > 1.25) and
                math.isfinite(closest) and closest > 0.75),
        }
        return result

    def write_evidence(self):
        temporary = self.evidence_path + '.tmp'
        try:
            os.makedirs(os.path.dirname(self.evidence_path) or '.', exist_ok=True)
            with open(temporary, 'w', encoding='utf-8') as stream:
                json.dump(self.evidence(), stream, indent=2, sort_keys=True)
                stream.write('\n')
            os.replace(temporary, self.evidence_path)
        except OSError as error:
            self.get_logger().error(f'cannot write evidence: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = FinalNavigationSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_evidence()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
