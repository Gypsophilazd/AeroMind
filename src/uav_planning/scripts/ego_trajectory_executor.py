#!/usr/bin/env python3
"""Sample EGO B-splines and publish the validated Mcontroller target format."""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class Spline:
    def __init__(self, points, degree, knots):
        self.points = [tuple(point) for point in points]
        self.degree = degree
        self.knots = tuple(knots)
        self.start = self.knots[self.degree]
        self.end = self.knots[len(self.points)]

    def evaluate(self, value):
        value = clamp(value, self.start, self.end)
        index = self.degree
        last_span = len(self.points) - 1
        while index < last_span and self.knots[index + 1] < value:
            index += 1
        work = [list(self.points[index - self.degree + i])
                for i in range(self.degree + 1)]
        for level in range(1, self.degree + 1):
            for i in range(self.degree, level - 1, -1):
                left = self.knots[i + index - self.degree]
                right = self.knots[i + 1 + index - level]
                denominator = right - left
                alpha = 0.0 if abs(denominator) < 1e-12 else (value - left) / denominator
                work[i] = [
                    (1.0 - alpha) * work[i - 1][axis] + alpha * work[i][axis]
                    for axis in range(len(work[i]))
                ]
        return tuple(work[self.degree])

    def derivative(self):
        if self.degree == 0:
            return None
        points = []
        for index in range(len(self.points) - 1):
            denominator = self.knots[index + self.degree + 1] - self.knots[index + 1]
            scale = 0.0 if abs(denominator) < 1e-12 else self.degree / denominator
            points.append(tuple(
                scale * (self.points[index + 1][axis] - self.points[index][axis])
                for axis in range(len(self.points[index]))))
        return Spline(points, self.degree - 1, self.knots[1:-1])


class EgoTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('ego_trajectory_executor')
        self.bspline_topic = self.declare_parameter(
            'bspline_topic', '/planning/bspline').value
        self.goal_topic = self.declare_parameter(
            'goal_topic', '/uav/planning/goal').value
        self.odom_topic = self.declare_parameter(
            'odometry_topic', '/uav/planning/odometry').value
        self.mission_topic = self.declare_parameter(
            'mission_topic', '/mission_001').value
        self.rate = float(self.declare_parameter('control_rate_hz', 20.0).value)
        self.enable_mission = bool(self.declare_parameter(
            'enable_mission_output', False).value)
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.lookahead = float(self.declare_parameter(
            'yaw_lookahead_sec', 0.5).value)
        self.max_yaw_rate = float(self.declare_parameter(
            'max_yaw_rate_rad_s', math.pi / 2.0).value)
        self.path_step = float(self.declare_parameter(
            'planned_path_sample_sec', 0.05).value)
        if self.rate <= 0.0 or self.max_yaw_rate <= 0.0 or self.path_step <= 0.0:
            raise ValueError('rate, max yaw rate, and path sample period must be positive')

        self.position_spline = None
        self.velocity_spline = None
        self.acceleration_spline = None
        self.received_steady = None
        self.elapsed_at_receive = 0.0
        self.duration = 0.0
        self.traj_id = None
        self.final_yaw = None
        self.last_yaw = None
        self.last_yaw_steady = None
        self.executed_path = Path()
        self.executed_path.header.frame_id = self.frame_id

        self.mission_pub = self.create_publisher(
            Float32MultiArray, self.mission_topic, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/uav/planning/commanded_setpoint', 10)
        self.sample_pub = self.create_publisher(
            Float64MultiArray, '/uav/planning/trajectory_sample', 10)
        self.planned_pub = self.create_publisher(
            Marker, '/uav/planning/planned_trajectory', 1)
        self.executed_pub = self.create_publisher(
            Path, '/uav/planning/executed_trajectory', 1)
        self.create_subscription(Bspline, self.bspline_topic, self.bspline_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info(
            f'B-spline executor: {self.bspline_topic} -> {self.mission_topic} '
            f'at {self.rate:.1f} Hz; mode=POSITION_ONLY; '
            f'mission_output={self.enable_mission}')

    def goal_callback(self, message):
        self.final_yaw = yaw_from_quaternion(message.pose.orientation)

    def odom_callback(self, message):
        if self.last_yaw is None:
            yaw = yaw_from_quaternion(message.pose.pose.orientation)
            if math.isfinite(yaw):
                self.last_yaw = yaw

    def bspline_callback(self, message):
        try:
            self.validate_bspline(message)
            points = [(point.x, point.y, point.z) for point in message.pos_pts]
            spline = Spline(points, int(message.order), list(message.knots))
            velocity = spline.derivative()
            acceleration = velocity.derivative()
        except (ValueError, IndexError, TypeError) as error:
            self.get_logger().error(f'rejecting invalid B-spline: {error}')
            return
        now_ros_ns = self.get_clock().now().nanoseconds
        start_ros_ns = message.start_time.sec * 1_000_000_000 + message.start_time.nanosec
        self.elapsed_at_receive = clamp(
            (now_ros_ns - start_ros_ns) / 1e9,
            0.0,
            spline.end - spline.start,
        )
        self.received_steady = time.monotonic()
        self.position_spline = spline
        self.velocity_spline = velocity
        self.acceleration_spline = acceleration
        self.duration = spline.end - spline.start
        self.traj_id = message.traj_id
        self.publish_planned_marker()
        self.get_logger().info(
            f'accepted trajectory id={self.traj_id}, duration={self.duration:.3f} s, '
            f'control_points={len(points)}')

    @staticmethod
    def validate_bspline(message):
        degree = int(message.order)
        if degree < 1 or len(message.pos_pts) <= degree:
            raise ValueError('invalid order/control-point count')
        if len(message.knots) != len(message.pos_pts) + degree + 1:
            raise ValueError('knot count does not equal points + order + 1')
        values = list(message.knots)
        for point in message.pos_pts:
            values.extend((point.x, point.y, point.z))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('non-finite control point or knot')
        if any(b < a for a, b in zip(message.knots, message.knots[1:])):
            raise ValueError('knots are not nondecreasing')
        if message.knots[len(message.pos_pts)] <= message.knots[degree]:
            raise ValueError('non-positive trajectory duration')

    def elapsed(self):
        return clamp(
            self.elapsed_at_receive + time.monotonic() - self.received_steady,
            0.0, self.duration)

    def desired_yaw(self, elapsed, position, velocity):
        at_end = elapsed >= self.duration - 0.05
        speed_xy = math.hypot(velocity[0], velocity[1])
        if at_end and self.final_yaw is not None:
            desired = self.final_yaw
        elif speed_xy > 0.10:
            lookahead_value = self.position_spline.start + min(
                self.duration, elapsed + self.lookahead)
            ahead = self.position_spline.evaluate(lookahead_value)
            delta_x = ahead[0] - position[0]
            delta_y = ahead[1] - position[1]
            desired = math.atan2(delta_y, delta_x) if math.hypot(delta_x, delta_y) > 0.05 \
                else math.atan2(velocity[1], velocity[0])
        elif self.final_yaw is not None:
            desired = self.final_yaw
        else:
            desired = self.last_yaw if self.last_yaw is not None else 0.0

        now_steady = time.monotonic()
        if self.last_yaw is None or self.last_yaw_steady is None:
            yaw = desired
            yaw_rate = 0.0
        else:
            dt = max(1e-3, now_steady - self.last_yaw_steady)
            delta = wrap_angle(desired - self.last_yaw)
            step = clamp(delta, -self.max_yaw_rate * dt, self.max_yaw_rate * dt)
            yaw = wrap_angle(self.last_yaw + step)
            yaw_rate = step / dt
        self.last_yaw = yaw
        self.last_yaw_steady = now_steady
        return yaw, yaw_rate

    def timer_callback(self):
        if self.position_spline is None:
            return
        elapsed = self.elapsed()
        parameter = self.position_spline.start + elapsed
        position = self.position_spline.evaluate(parameter)
        velocity = self.velocity_spline.evaluate(parameter)
        acceleration = self.acceleration_spline.evaluate(parameter)
        yaw, yaw_rate = self.desired_yaw(elapsed, position, velocity)
        values = position + velocity + acceleration + (yaw, yaw_rate)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('non-finite sampled state; suppressing output')
            return

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation = quaternion_from_yaw(yaw)
        self.pose_pub.publish(pose)

        sample = Float64MultiArray()
        sample.layout.dim = [MultiArrayDimension(
            label='t,duration,px,py,pz,vx,vy,vz,ax,ay,az,yaw,yaw_rate',
            size=13, stride=13)]
        sample.data = [elapsed, self.duration, *values]
        self.sample_pub.publish(sample)

        # The already flight-validated fcu_bridge simple_target mode consumes
        # local NED values and ignores velocity, acceleration, and yaw-rate.
        mission = Float32MultiArray()
        mission.data = [
            -yaw, 0.0,
            position[0], -position[1], -position[2],
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]
        if self.enable_mission:
            self.mission_pub.publish(mission)

        if not self.executed_path.poses or elapsed >= self.duration or \
                time.monotonic() - getattr(self, '_last_path_time', 0.0) >= 0.1:
            self.executed_path.header.stamp = stamp
            self.executed_path.poses.append(pose)
            if len(self.executed_path.poses) > 2000:
                self.executed_path.poses = self.executed_path.poses[-2000:]
            self.executed_pub.publish(self.executed_path)
            self._last_path_time = time.monotonic()

    def publish_planned_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'ego_planned_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.07
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        sample_count = max(2, int(math.ceil(self.duration / self.path_step)) + 1)
        for index in range(sample_count):
            elapsed = min(self.duration, index * self.path_step)
            value = self.position_spline.evaluate(self.position_spline.start + elapsed)
            point = Point()
            point.x, point.y, point.z = value
            marker.points.append(point)
        self.planned_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = EgoTrajectoryExecutor()
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
