#!/usr/bin/env python3
"""Translate the unified RViz goal API to EGO's legacy manual-goal topic."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def finite_pose(message):
    values = (
        message.pose.position.x, message.pose.position.y,
        message.pose.orientation.x, message.pose.orientation.y,
        message.pose.orientation.z, message.pose.orientation.w,
    )
    return all(math.isfinite(value) for value in values)


class EgoGoalAdapter(Node):
    def __init__(self):
        super().__init__('ego_goal_adapter')
        self.input_topic = self.declare_parameter('input_topic', '/goal_pose').value
        self.ego_topic = self.declare_parameter(
            'ego_goal_topic', '/move_base_simple/goal').value
        self.normalized_topic = self.declare_parameter(
            'normalized_goal_topic', '/uav/planning/goal').value
        self.marker_topic = self.declare_parameter(
            'goal_marker_topic', '/uav/planning/goal_marker').value
        self.altitude = float(self.declare_parameter(
            'fixed_altitude_m', 1.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.tf_timeout = float(self.declare_parameter(
            'tf_timeout_sec', 0.2).value)
        if not math.isfinite(self.altitude):
            raise ValueError('fixed_altitude_m must be finite')
        if self.tf_timeout <= 0.0:
            raise ValueError('tf_timeout_sec must be positive')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ego_pub = self.create_publisher(PoseStamped, self.ego_topic, qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.normalized_topic, qos)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, marker_qos)
        self.create_subscription(PoseStamped, self.input_topic, self.goal_callback, qos)
        self.get_logger().info(
            f'RViz goal adapter: {self.input_topic} -> {self.ego_topic}; '
            f'fixed z={self.altitude:.3f} m, frame={self.frame_id}')

    def goal_callback(self, message):
        if not finite_pose(message):
            self.get_logger().error('rejecting non-finite goal')
            return
        quaternion_norm = math.sqrt(
            message.pose.orientation.x ** 2 + message.pose.orientation.y ** 2 +
            message.pose.orientation.z ** 2 + message.pose.orientation.w ** 2)
        if quaternion_norm < 0.99 or quaternion_norm > 1.01:
            self.get_logger().error(
                f'rejecting invalid goal quaternion norm {quaternion_norm:.6f}')
            return

        source_frame = message.header.frame_id or self.frame_id
        if source_frame == self.frame_id:
            transformed = message
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id, source_frame, Time(),
                    timeout=Duration(seconds=self.tf_timeout))
                transformed = do_transform_pose_stamped(message, transform)
            except TransformException as error:
                self.get_logger().error(
                    f'rejecting goal: cannot transform {source_frame!r} -> '
                    f'{self.frame_id!r}: {error}')
                return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose = transformed.pose
        goal.pose.position.z = self.altitude
        self.goal_pub.publish(goal)
        self.ego_pub.publish(goal)

        marker = Marker()
        marker.header = goal.header
        marker.ns = 'uav_navigation_goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.8
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
        self.get_logger().info(
            f'accepted goal x={goal.pose.position.x:.3f}, '
            f'y={goal.pose.position.y:.3f}, z={goal.pose.position.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = EgoGoalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
