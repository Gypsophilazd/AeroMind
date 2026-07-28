#!/usr/bin/env python3
"""Publish one fixed, position-only local-NED target until stopped.

The tool never arms, takes off, lands, or changes flight mode. Without
--send it only prints the target derived from the first /odometry_001 sample.
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class PositionProbe(Node):
    def __init__(self):
        super().__init__("fcu_position_probe")
        self.sample = None
        self.create_subscription(
            Odometry, "/odometry_001", self._odom_callback, 1)
        self.publisher = self.create_publisher(
            Float32MultiArray, "/mission_001", 10)

    def _odom_callback(self, message):
        if self.sample is None:
            self.sample = message


def parse_args():
    parser = argparse.ArgumentParser(
        description="Hold one target delta from a fresh /odometry_001 sample")
    parser.add_argument(
        "--delta-x", type=float, choices=(0.3, 0.5), default=0.3,
        help="nose-forward displacement in metres (default: 0.3)")
    parser.add_argument(
        "--rate", type=float, default=10.0,
        help="fixed-target publish rate in Hz (default: 10)")
    parser.add_argument(
        "--send", action="store_true",
        help="actually publish; otherwise print one dry-run target and exit")
    return parser.parse_args()


def main():
    args = parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        print("ERROR: --rate must be finite and positive", file=sys.stderr)
        return 2

    rclpy.init()
    node = PositionProbe()
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and node.sample is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.sample is None:
        node.get_logger().error("no /odometry_001 sample received within 5 s")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    pose = node.sample.pose.pose
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        node.get_logger().error("non-finite /odometry_001 pose; refusing target")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    if quaternion_norm < 0.99 or quaternion_norm > 1.01:
        node.get_logger().error(
            f"invalid quaternion norm {quaternion_norm:.6f}; refusing target")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    lio_yaw = yaw_from_quaternion(*values[3:])
    target = Float32MultiArray()
    target.data = [
        -lio_yaw, 0.0,
        pose.position.x + args.delta_x,
        -pose.position.y,
        -pose.position.z,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ]
    node.get_logger().info(
        "fixed LOCAL_NED target: "
        f"x={target.data[2]:.3f} y={target.data[3]:.3f} "
        f"z={target.data[4]:.3f} yaw={target.data[0]:.3f}; "
        f"delta_x=+{args.delta_x:.1f} m")

    if not args.send:
        node.get_logger().info("DRY RUN: nothing published (add --send to transmit)")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    period = 1.0 / args.rate
    node.get_logger().warn(
        "PUBLISHING /mission_001; Ctrl-C stops the target stream")
    try:
        while rclpy.ok():
            node.publisher.publish(target)
            rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
