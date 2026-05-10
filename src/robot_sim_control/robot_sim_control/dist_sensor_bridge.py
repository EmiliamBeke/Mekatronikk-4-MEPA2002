#!/usr/bin/env python3
"""Republish the sim distance-sensor LaserScan as /mega/distance_mm (Int32).

Mirrors what mega_driver_node provides on real hardware so teddy_grab_node
can use distance contact detection in Gazebo.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class DistSensorBridge(Node):
    def __init__(self) -> None:
        super().__init__("dist_sensor_bridge")
        self.declare_parameter("input_topic", "/sim/dist_sensor/scan")
        self.declare_parameter("output_topic", "/mega/distance_mm")
        self.declare_parameter("max_distance_mm", 2000)

        self.max_mm = int(self.get_parameter("max_distance_mm").value)
        self.pub = self.create_publisher(
            Int32, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self._on_scan,
            10,
        )

    def _on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        d = msg.ranges[0]
        if math.isnan(d) or math.isinf(d) or d > msg.range_max:
            mm = self.max_mm
        elif d < msg.range_min:
            mm = 0
        else:
            mm = int(round(d * 1000.0))
        self.pub.publish(Int32(data=mm))


def main() -> None:
    rclpy.init()
    node = DistSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
