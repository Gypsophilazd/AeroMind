#!/usr/bin/env python3
"""Collect runtime evidence for the SPARK -> adapter -> EGO pipeline."""

import argparse
from collections import OrderedDict
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from traj_utils.msg import Bspline


def vector3(msg):
    return [msg.x, msg.y, msg.z]


def magnitude(values):
    return math.sqrt(sum(value * value for value in values))


def finite_vector(values):
    return all(math.isfinite(value) for value in values)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def de_boor(points, degree, knots, value):
    n = len(points) - 1
    m = n + degree + 1
    bounded = min(max(knots[degree], value), knots[m - degree])
    k = degree
    while knots[k + 1] < bounded:
        k += 1
    work = [list(points[k - degree + i]) for i in range(degree + 1)]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            denominator = (
                knots[index + 1 + k - level] - knots[index + k - degree]
            )
            alpha = (bounded - knots[index + k - degree]) / denominator
            work[index] = [
                (1.0 - alpha) * work[index - 1][axis]
                + alpha * work[index][axis]
                for axis in range(3)
            ]
    return work[degree]


def bspline_summary(msg):
    points = [[point.x, point.y, point.z] for point in msg.pos_pts]
    knots = list(msg.knots)
    position = de_boor(points, msg.order, knots, knots[msg.order])
    derivative_points = []
    for index in range(len(points) - 1):
        scale = msg.order / (
            knots[index + msg.order + 1] - knots[index + 1]
        )
        derivative_points.append([
            scale * (points[index + 1][axis] - points[index][axis])
            for axis in range(3)
        ])
    derivative_knots = knots[1:-1]
    velocity = de_boor(
        derivative_points,
        msg.order - 1,
        derivative_knots,
        derivative_knots[msg.order - 1],
    )
    duration = knots[len(points)] - knots[msg.order]
    return {
        'order': msg.order,
        'traj_id': msg.traj_id,
        'control_point_count': len(points),
        'knot_count': len(knots),
        'duration_sec': duration,
        'start_time': {
            'sec': msg.start_time.sec,
            'nanosec': msg.start_time.nanosec,
        },
        'initial_position': position,
        'initial_velocity': velocity,
        'initial_speed': magnitude(velocity),
    }


class StreamStats:
    def __init__(self):
        self.count = 0
        self.first_time = None
        self.last_time = None
        self.max_interval = 0.0

    def add(self):
        current = time.monotonic()
        if self.first_time is None:
            self.first_time = current
        if self.last_time is not None:
            self.max_interval = max(self.max_interval, current - self.last_time)
        self.last_time = current
        self.count += 1

    def summary(self):
        span = 0.0 if self.first_time is None else self.last_time - self.first_time
        rate = 0.0 if span <= 0.0 or self.count < 2 else (self.count - 1) / span
        return {
            'count': self.count,
            'observed_rate_hz': rate,
            'max_interval_sec': self.max_interval,
        }


class PipelineProbe(Node):
    def __init__(self, args):
        super().__init__('fast_lio_ego_pipeline_probe')
        self.args = args
        self.started = time.monotonic()
        self.source_stats = StreamStats()
        self.adapter_stats = StreamStats()
        self.cloud_stats = StreamStats()
        self.map_stats = StreamStats()
        self.bspline_stats = StreamStats()
        self.source_samples = OrderedDict()
        self.latest_source = None
        self.latest_adapter = None
        self.previous_source_stamp = None
        self.observed_source_rollbacks = 0
        self.source_speeds = []
        self.adapter_speeds = []
        self.adapter_nonfinite_count = 0
        self.pose_comparison_count = 0
        self.pose_max_abs_error = 0.0
        self.orientation_max_abs_error = 0.0
        self.frame_mismatch_count = 0
        self.map_nonempty_count = 0
        self.map_max_points = 0
        self.cloud_max_points = 0
        self.bspline_summaries = []
        self.latest_diagnostics = {}
        self.goal = None
        self.goal_context = None

        self.create_subscription(Odometry, '/odometry', self.source_callback, 10)
        self.create_subscription(
            Odometry, '/uav/planning/odometry', self.adapter_callback, 10
        )
        self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_callback, 10
        )
        self.create_subscription(
            PointCloud2, '/grid_map/occupancy_inflate', self.map_callback, 10
        )
        self.create_subscription(
            Bspline, '/planning/bspline', self.bspline_callback, 10
        )
        self.create_subscription(
            DiagnosticArray,
            '/uav/planning/odometry_adapter/diagnostics',
            self.diagnostics_callback,
            10,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, '/move_base_simple/goal', 10
        )
        self.goal_timer = self.create_timer(0.2, self.maybe_send_goal)

    def source_callback(self, msg):
        self.source_stats.add()
        stamp = stamp_ns(msg.header.stamp)
        if self.previous_source_stamp is not None and stamp < self.previous_source_stamp:
            self.observed_source_rollbacks += 1
        self.previous_source_stamp = stamp
        speed_vector = vector3(msg.twist.twist.linear)
        self.source_speeds.append(magnitude(speed_vector))
        sample = {
            'position': vector3(msg.pose.pose.position),
            'orientation': [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            'linear_velocity': speed_vector,
            'header_frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'stamp_ns': stamp,
        }
        self.latest_source = sample
        self.source_samples[stamp] = sample
        while len(self.source_samples) > 2000:
            self.source_samples.popitem(last=False)

    def adapter_callback(self, msg):
        self.adapter_stats.add()
        velocity = vector3(msg.twist.twist.linear)
        if finite_vector(velocity):
            self.adapter_speeds.append(magnitude(velocity))
        else:
            self.adapter_nonfinite_count += 1
        stamp = stamp_ns(msg.header.stamp)
        sample = {
            'position': vector3(msg.pose.pose.position),
            'orientation': [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            'linear_velocity': velocity,
            'header_frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'stamp_ns': stamp,
        }
        self.latest_adapter = sample
        source = self.source_samples.get(stamp)
        if source is not None:
            self.pose_comparison_count += 1
            self.pose_max_abs_error = max(
                self.pose_max_abs_error,
                max(abs(a - b) for a, b in zip(sample['position'], source['position'])),
            )
            self.orientation_max_abs_error = max(
                self.orientation_max_abs_error,
                max(
                    abs(a - b)
                    for a, b in zip(sample['orientation'], source['orientation'])
                ),
            )
            if (
                sample['header_frame_id'] != source['header_frame_id']
                or sample['child_frame_id'] != source['child_frame_id']
            ):
                self.frame_mismatch_count += 1

    def cloud_callback(self, msg):
        self.cloud_stats.add()
        self.cloud_max_points = max(self.cloud_max_points, msg.width * msg.height)

    def map_callback(self, msg):
        self.map_stats.add()
        points = msg.width * msg.height
        self.map_max_points = max(self.map_max_points, points)
        if points > 0:
            self.map_nonempty_count += 1

    def bspline_callback(self, msg):
        self.bspline_stats.add()
        if len(self.bspline_summaries) < 20:
            self.bspline_summaries.append(bspline_summary(msg))

    def diagnostics_callback(self, msg):
        for status in msg.status:
            if status.name == 'thin_odom_adapter/velocity_estimator':
                self.latest_diagnostics = {
                    item.key: item.value for item in status.values
                }

    def maybe_send_goal(self):
        if self.goal is not None or self.args.send_goal_after < 0.0:
            return
        if time.monotonic() - self.started < self.args.send_goal_after:
            return
        if self.latest_adapter is None or self.goal_pub.get_subscription_count() == 0:
            return
        if self.args.goal_absolute:
            goal_x = self.args.goal_x
            goal_y = self.args.goal_y
        else:
            goal_x = self.latest_adapter['position'][0] + self.args.goal_x
            goal_y = self.latest_adapter['position'][1] + self.args.goal_y
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.pose.position.x = goal_x
        message.pose.position.y = goal_y
        message.pose.position.z = 1.0
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        self.goal = {'x': goal_x, 'y': goal_y, 'published_z': 1.0}
        self.goal_context = {
            'source': self.latest_source,
            'adapter': self.latest_adapter,
            'elapsed_sec': time.monotonic() - self.started,
        }
        self.get_logger().info(
            f'Published goal ({goal_x:.3f}, {goal_y:.3f}, 1.000) in odom'
        )

    @staticmethod
    def speed_summary(values):
        if not values:
            return {'min': 0.0, 'max': 0.0, 'mean': 0.0, 'nonzero_count': 0}
        return {
            'min': min(values),
            'max': max(values),
            'mean': sum(values) / len(values),
            'nonzero_count': sum(value > 1e-4 for value in values),
        }

    def result(self):
        return {
            'requested_duration_sec': self.args.duration,
            'actual_duration_sec': time.monotonic() - self.started,
            'source_odometry': self.source_stats.summary(),
            'adapter_odometry': self.adapter_stats.summary(),
            'cloud_registered': self.cloud_stats.summary(),
            'grid_map_occupancy_inflate': {
                **self.map_stats.summary(),
                'nonempty_count': self.map_nonempty_count,
                'max_points': self.map_max_points,
            },
            'cloud_registered_max_points': self.cloud_max_points,
            'planning_bspline': self.bspline_stats.summary(),
            'source_speed_mps': self.speed_summary(self.source_speeds),
            'adapter_speed_mps': self.speed_summary(self.adapter_speeds),
            'adapter_nonfinite_count': self.adapter_nonfinite_count,
            'observed_source_stamp_rollbacks': self.observed_source_rollbacks,
            'pose_passthrough': {
                'comparison_count': self.pose_comparison_count,
                'position_max_abs_error': self.pose_max_abs_error,
                'orientation_max_abs_error': self.orientation_max_abs_error,
                'frame_mismatch_count': self.frame_mismatch_count,
            },
            'latest_source': self.latest_source,
            'latest_adapter': self.latest_adapter,
            'latest_adapter_diagnostics': self.latest_diagnostics,
            'goal': self.goal,
            'goal_context': self.goal_context,
            'bsplines': self.bspline_summaries,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--send-goal-after', type=float, default=-1.0)
    parser.add_argument('--goal-x', type=float, default=3.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--goal-absolute', action='store_true')
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    rclpy.init()
    node = PipelineProbe(args)
    try:
        end = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        result = node.result()
        node.destroy_node()
        rclpy.shutdown()

    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as stream:
            stream.write(rendered + '\n')
    print(rendered)


if __name__ == '__main__':
    main()
