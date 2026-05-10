#!/usr/bin/env python3
"""Publish a purple line marker for the gripper VL53 distance sensor.

Subscribes to /mega/distance_mm (Int32, millimeters) so it works on both the
real robot and the Gazebo sim (where dist_sensor_bridge republishes the GPU
lidar scan). The marker is a single line in dist_sensor_link from origin to
(d, 0, 0), where d is the latest reading clamped to a sensible max.
"""
from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker


PARAM_DEFAULTS = {
    "enabled": True,
    "distance_topic": "/mega/distance_mm",
    "marker_topic": "/teddy_grab/dist_marker",
    "marker_frame_id": "dist_sensor_link",
    "max_distance_m": 1.0,
    "line_width_m": 0.003,
    # Purple.
    "color_r": 0.7,
    "color_g": 0.2,
    "color_b": 0.9,
    "color_a": 0.95,
}


class DistSensorMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("dist_sensor_marker")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.enabled = bool(self.param("enabled"))
        self.frame_id = str(self.param("marker_frame_id"))
        self.max_distance_m = float(self.param("max_distance_m"))
        self.line_width_m = float(self.param("line_width_m"))
        self.color = (
            float(self.param("color_r")),
            float(self.param("color_g")),
            float(self.param("color_b")),
            float(self.param("color_a")),
        )

        self.pub = self.create_publisher(Marker, str(self.param("marker_topic")), 10)
        self.create_subscription(
            Int32, str(self.param("distance_topic")), self.on_distance, 10
        )

        self.get_logger().info(
            "dist sensor marker enabled=%s frame=%s" % (self.enabled, self.frame_id)
        )

    def param(self, name: str):
        return self.get_parameter(name).value

    def on_distance(self, msg: Int32) -> None:
        if not self.enabled:
            return

        distance_m = max(0.0, float(msg.data) * 1e-3)
        if distance_m > self.max_distance_m:
            distance_m = self.max_distance_m

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = Time()  # Use latest TF in RViz.
        marker.ns = "dist_sensor_beam"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.line_width_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.color
        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=distance_m, y=0.0, z=0.0),
        ]
        self.pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = DistSensorMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
