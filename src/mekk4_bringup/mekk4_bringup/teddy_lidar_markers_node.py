#!/usr/bin/env python3
from __future__ import annotations

import math
import re

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


STATUS_RE = re.compile(r"(?P<key>[A-Za-z_]+)=(?P<value>[^ ]+)")
MARKER_Z = 0.0
DEFAULT_RAY_DISTANCE_M = 0.50
TARGET_COLOR = (0.0, 1.0, 0.0, 0.9)
POINT_COLOR = (1.0, 0.65, 0.0, 0.95)

PARAM_DEFAULTS = {
    "enabled": True,
    "status_topic": "/teddy_detector/status",
    "scan_topic": "/lidar",
    "marker_topic": "/teddy_approach/lidar_stop_markers",
    "marker_frame_id": "base_laser",
    "marker_use_latest_tf": True,
    "stop_lidar_distance_m": 0.15,
    "stop_lidar_front_angle_rad": 0.20,
}


def parse_status(text):
    return {match.group("key"): match.group("value") for match in STATUS_RE.finditer(text)}


def point_at(distance, angle, z=MARKER_Z):
    return Point(x=distance * math.cos(angle), y=distance * math.sin(angle), z=z)


def set_color(marker, color):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color


class TeddyLidarMarkersNode(Node):
    def __init__(self):
        super().__init__("teddy_lidar_markers")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.enabled = self.param("enabled")
        self.marker_frame_id = str(self.param("marker_frame_id")).strip()
        self.marker_use_latest_tf = bool(self.param("marker_use_latest_tf"))
        self.stop_distance = self.param("stop_lidar_distance_m")
        self.front_angle = self.param("stop_lidar_front_angle_rad")

        self.marker_pub = self.create_publisher(MarkerArray, self.param("marker_topic"), 10)
        self.create_subscription(String, self.param("status_topic"), self.on_status, 10)
        self.create_subscription(LaserScan, self.param("scan_topic"), self.on_scan, 10)

        self.teddy_count = 0
        self.scan_frame = ""
        self.scan_stamp = Time()
        self.closest_distance = math.inf
        self.closest_angle = 0.0

        self.get_logger().info("teddy lidar markers enabled=%s" % self.enabled)

    def param(self, name):
        return self.get_parameter(name).value

    def on_status(self, msg):
        fields = parse_status(msg.data)
        self.teddy_count = int(fields.get("teddy_count", "0"))

    def on_scan(self, msg):
        if not self.enabled:
            return

        self.scan_frame = msg.header.frame_id.strip()
        self.scan_stamp = msg.header.stamp
        self.closest_distance = math.inf
        self.closest_angle = 0.0

        angle = msg.angle_min
        for distance in msg.ranges:
            in_front = abs(angle) <= self.front_angle
            valid = math.isfinite(distance) and msg.range_min <= distance <= msg.range_max

            if in_front and valid and distance < self.closest_distance:
                self.closest_distance = float(distance)
                self.closest_angle = angle

            angle += msg.angle_increment

        self.publish_markers()

    def publish_markers(self):
        if not self.scan_frame:
            return

        stamp = Time() if self.marker_use_latest_tf else self.scan_stamp
        markers = MarkerArray()
        markers.markers.append(self.sector_marker(stamp))
        markers.markers.append(self.arc_marker(stamp))
        markers.markers.append(self.closest_marker(stamp))
        self.marker_pub.publish(markers)

    def base_marker(self, stamp, marker_id, marker_type, color):
        marker = Marker()
        marker.header.frame_id = self.marker_frame_id or self.scan_frame
        marker.header.stamp = stamp
        marker.ns = "teddy_lidar_stop"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        set_color(marker, color)
        return marker

    def sector_marker(self, stamp):
        distance = self.ray_distance()
        origin = Point(x=0.0, y=0.0, z=MARKER_Z)
        marker = self.base_marker(stamp, 0, Marker.LINE_LIST, TARGET_COLOR)
        marker.scale.x = 0.002
        marker.points.extend([
            origin,
            point_at(distance, -self.front_angle),
            origin,
            point_at(distance, self.front_angle),
        ])
        return marker

    def arc_marker(self, stamp):
        marker = self.base_marker(stamp, 1, Marker.LINE_STRIP, TARGET_COLOR)
        marker.scale.x = 0.002
        marker.points = [
            point_at(self.stop_distance, -self.front_angle + (2.0 * self.front_angle * i / 16))
            for i in range(17)
        ]
        return marker

    def closest_marker(self, stamp):
        marker = self.base_marker(stamp, 2, Marker.SPHERE, POINT_COLOR)
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        if self.teddy_lidar_candidate_visible():
            marker.pose.position.x = self.closest_distance * math.cos(self.closest_angle)
            marker.pose.position.y = self.closest_distance * math.sin(self.closest_angle)
            marker.pose.position.z = 0.06
            marker.pose.orientation.w = 1.0
        else:
            marker.action = Marker.DELETE

        return marker

    def teddy_lidar_candidate_visible(self):
        return (
            self.teddy_count > 0
            and math.isfinite(self.closest_distance)
            and abs(self.closest_angle) <= self.front_angle
        )

    def ray_distance(self):
        if self.teddy_lidar_candidate_visible():
            return self.closest_distance
        return DEFAULT_RAY_DISTANCE_M


def main(args=None):
    rclpy.init(args=args)
    node = TeddyLidarMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
