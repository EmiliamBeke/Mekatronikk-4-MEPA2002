#!/usr/bin/env python3
from __future__ import annotations

import math
import re
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


STATUS_RE = re.compile(r"(?P<key>[A-Za-z_]+)=(?P<value>[^ ]+)")
MARKER_Z = 0.03
DEFAULT_MARKER_DISTANCE_M = 0.50
TARGET_MARKER_COLOR = (0.0, 1.0, 0.0, 0.9)
TEDDY_POINT_COLOR = (1.0, 0.65, 0.0, 0.95)
PARAM_DEFAULTS = {
    "enabled": False,
    "status_topic": "/teddy_detector/status",
    "cmd_vel_topic": "/cmd_vel_manual",
    "publish_period_s": 0.05,
    "lost_timeout_s": 0.5,
    "linear_speed": 0.08,
    "angular_kp": 1.2,
    "min_angular_speed": 0.0,
    "max_angular_speed": 0.8,
    "center_tolerance": 0.10,
    "scan_topic": "/lidar",
    "stop_lidar_distance_m": 0.0,
    "stop_lidar_front_angle_rad": 0.20,
    "stop_lidar_min_points": 3,
    "stop_lidar_timeout_s": 0.5,
    "visualize_lidar_stop": True,
    "marker_topic": "/teddy_approach/lidar_stop_markers",
    "drive_when_not_centered": False,
    "use_nav_goal": False,
    "send_goal_on_start": False,
    "goal_frame": "odom",
    "goal_x": 0.0,
    "goal_y": 0.0,
    "goal_yaw": 0.0,
}


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_status(text: str) -> dict[str, str]:
    return {match.group("key"): match.group("value") for match in STATUS_RE.finditer(text)}


def set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


def point_at(distance: float, angle: float, z: float = MARKER_Z) -> Point:
    return Point(x=distance * math.cos(angle), y=distance * math.sin(angle), z=z)


class TeddyApproachNode(Node):
    def __init__(self) -> None:
        super().__init__("teddy_approach")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        # Control parameters. Topic names stay explicit; no auto-discovery.
        self._enabled = self._param("enabled")
        status_topic = self._param("status_topic")
        cmd_vel_topic = self._param("cmd_vel_topic")
        publish_period_s = self._param("publish_period_s")
        self._lost_timeout_s = self._param("lost_timeout_s")
        self._linear_speed = self._param("linear_speed")
        self._angular_kp = self._param("angular_kp")
        self._min_angular_speed = self._param("min_angular_speed")
        self._max_angular_speed = self._param("max_angular_speed")
        self._center_tolerance = self._param("center_tolerance")
        scan_topic = self._param("scan_topic")
        self._stop_lidar_distance_m = self._param("stop_lidar_distance_m")
        self._stop_lidar_front_angle_rad = self._param("stop_lidar_front_angle_rad")
        self._stop_lidar_min_points = self._param("stop_lidar_min_points")
        self._stop_lidar_timeout_s = self._param("stop_lidar_timeout_s")
        self._visualize_lidar_stop = self._param("visualize_lidar_stop")
        marker_topic = self._param("marker_topic")
        self._drive_when_not_centered = self._param("drive_when_not_centered")
        self._use_nav_goal = self._param("use_nav_goal")
        self._send_goal_on_start = self._param("send_goal_on_start")

        self._validate_params()

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._marker_pub = (
            self.create_publisher(MarkerArray, marker_topic, 10)
            if self._visualize_lidar_stop
            else None
        )
        self._status_sub = self.create_subscription(String, status_topic, self._on_status, 10)
        self._scan_sub = self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._timer = self.create_timer(publish_period_s, self._on_timer)
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._last_seen_at = -1.0
        self._last_dx = 0.0
        self._last_front_distance = math.inf
        self._last_front_angle = 0.0
        self._last_front_points = 0
        self._last_scan_frame = ""
        self._last_scan_at = -1.0
        self._last_count = 0
        self._nav_goal_sent = False
        self._last_mode = ""

        if self._enabled and self._use_nav_goal and self._send_goal_on_start:
            self._send_nav_goal()

        self.get_logger().info(
            "teddy approach enabled=%s status=%s cmd=%s scan=%s markers=%s nav_goal=%s"
            % (
                self._enabled,
                status_topic,
                cmd_vel_topic,
                scan_topic,
                marker_topic if self._visualize_lidar_stop else "off",
                self._use_nav_goal,
            )
        )

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _validate_params(self) -> None:
        checks = [
            (self._min_angular_speed >= 0.0, "min_angular_speed must be zero or greater"),
            (
                self._max_angular_speed >= self._min_angular_speed,
                "max_angular_speed must be greater than or equal to min_angular_speed",
            ),
            (self._stop_lidar_distance_m >= 0.0, "stop_lidar_distance_m must be zero or greater"),
            (self._stop_lidar_front_angle_rad >= 0.0, "stop_lidar_front_angle_rad must be zero or greater"),
            (self._stop_lidar_min_points >= 1, "stop_lidar_min_points must be at least 1"),
            (self._stop_lidar_timeout_s >= 0.0, "stop_lidar_timeout_s must be zero or greater"),
        ]
        for ok, message in checks:
            if not ok:
                raise ValueError(message)

    def _on_status(self, msg: String) -> None:
        fields = parse_status(msg.data)
        count_text = fields.get("teddy_count")
        if count_text is None:
            return

        self._last_count = int(count_text)
        dx_text = fields.get("dx")
        if self._last_count > 0 and dx_text is not None:
            self._last_dx = float(dx_text)
            self._last_seen_at = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        front_ranges = []
        closest_distance = math.inf
        closest_angle = 0.0
        angle = msg.angle_min
        for distance in msg.ranges:
            if (
                abs(angle) <= self._stop_lidar_front_angle_rad
                and math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):
                valid_distance = float(distance)
                front_ranges.append(valid_distance)
                if valid_distance < closest_distance:
                    closest_distance = valid_distance
                    closest_angle = angle
            angle += msg.angle_increment

        self._last_front_points = len(front_ranges)
        self._last_front_distance = closest_distance
        self._last_front_angle = closest_angle
        self._last_scan_frame = msg.header.frame_id
        self._last_scan_at = time.monotonic()

    def _send_nav_goal(self) -> None:
        if self._nav_goal_sent:
            return
        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning("navigate_to_pose action server not available yet")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._param("goal_frame")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._param("goal_x")
        goal.pose.pose.position.y = self._param("goal_y")
        yaw = self._param("goal_yaw")
        goal.pose.pose.orientation = yaw_to_quaternion(yaw)

        self._nav_client.send_goal_async(goal)
        self._nav_goal_sent = True
        self.get_logger().info(
            "sent teddy search goal x=%.2f y=%.2f yaw=%.2f"
            % (goal.pose.pose.position.x, goal.pose.pose.position.y, yaw)
        )

    def _on_timer(self) -> None:
        if not self._enabled:
            return

        if self._use_nav_goal and self._send_goal_on_start and not self._nav_goal_sent:
            self._send_nav_goal()

        now = time.monotonic()
        self._publish_lidar_markers()
        if not self._teddy_recent(now):
            self._log_mode("waiting_for_teddy")
            return

        cmd = Twist()
        centered = abs(self._last_dx) <= self._center_tolerance
        close_enough = centered and self._lidar_stop_active(now)
        if close_enough:
            self._cmd_pub.publish(cmd)
            self._log_mode("close_enough_lidar")
            return

        # P-controller on horizontal image error:
        # dx > 0 means teddy is right of center, so angular.z must turn right.
        if not centered:
            cmd.angular.z = self._turn_rate_from_dx(self._last_dx)

        # Forward motion is gated by centering unless explicitly disabled.
        if centered or self._drive_when_not_centered:
            cmd.linear.x = self._linear_speed

        self._cmd_pub.publish(cmd)
        self._log_mode("centering" if not centered else "approaching")

    def _teddy_recent(self, now: float) -> bool:
        return self._last_seen_at >= 0.0 and (now - self._last_seen_at) <= self._lost_timeout_s

    def _lidar_stop_active(self, now: float) -> bool:
        lidar_fresh = self._last_scan_at >= 0.0 and (now - self._last_scan_at) <= self._stop_lidar_timeout_s
        return (
            self._stop_lidar_distance_m > 0.0
            and lidar_fresh
            and self._last_front_points >= self._stop_lidar_min_points
            and self._last_front_distance <= self._stop_lidar_distance_m
        )

    def _turn_rate_from_dx(self, dx: float) -> float:
        raw = -self._angular_kp * dx
        if raw == 0.0:
            return 0.0
        with_minimum = math.copysign(max(abs(raw), self._min_angular_speed), raw)
        return clamp(with_minimum, -self._max_angular_speed, self._max_angular_speed)

    def _publish_lidar_markers(self) -> None:
        if self._marker_pub is None or not self._last_scan_frame:
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._sector_marker(now))
        markers.markers.append(self._arc_marker(now))
        markers.markers.append(self._closest_marker(now))
        self._marker_pub.publish(markers)

    def _base_marker(self, stamp, marker_id: int, marker_type: int, color: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._last_scan_frame
        marker.header.stamp = stamp
        marker.ns = "teddy_lidar_stop"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        set_color(marker, color)
        return marker

    def _sector_marker(self, stamp) -> Marker:
        distance = self._marker_ray_distance()
        angle = self._stop_lidar_front_angle_rad
        origin = Point(x=0.0, y=0.0, z=MARKER_Z)
        sector = self._base_marker(stamp, 0, Marker.LINE_LIST, TARGET_MARKER_COLOR)
        sector.scale.x = 0.002
        sector.points.extend([origin, point_at(distance, -angle), origin, point_at(distance, angle)])
        return sector

    def _arc_marker(self, stamp) -> Marker:
        distance = max(0.0, self._stop_lidar_distance_m)
        front_angle = self._stop_lidar_front_angle_rad
        arc = self._base_marker(stamp, 1, Marker.LINE_STRIP, TARGET_MARKER_COLOR)
        arc.scale.x = 0.002
        arc.points = [
            point_at(distance, -front_angle + (2.0 * front_angle * idx / 16))
            for idx in range(17)
        ]
        return arc

    def _closest_marker(self, stamp) -> Marker:
        closest = self._base_marker(stamp, 2, Marker.SPHERE, TEDDY_POINT_COLOR)
        closest.type = Marker.SPHERE
        closest.scale.x = 0.04
        closest.scale.y = 0.04
        closest.scale.z = 0.04
        if self._teddy_lidar_candidate_visible():
            closest.pose.position.x = self._last_front_distance * math.cos(self._last_front_angle)
            closest.pose.position.y = self._last_front_distance * math.sin(self._last_front_angle)
            closest.pose.position.z = 0.06
            closest.pose.orientation.w = 1.0
        else:
            closest.action = Marker.DELETE
        return closest

    def _teddy_lidar_candidate_visible(self) -> bool:
        return (
            self._last_count > 0
            and math.isfinite(self._last_front_distance)
            and abs(self._last_front_angle) <= self._stop_lidar_front_angle_rad
        )

    def _marker_ray_distance(self) -> float:
        if self._teddy_lidar_candidate_visible():
            return self._last_front_distance
        return DEFAULT_MARKER_DISTANCE_M

    def _log_mode(self, mode: str) -> None:
        if mode == self._last_mode:
            return
        self.get_logger().info(f"mode={mode}")
        self._last_mode = mode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeddyApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
