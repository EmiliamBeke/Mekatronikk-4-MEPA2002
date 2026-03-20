#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("twist_transform")

        self.declare_parameter("input_topic", "cmd_vel_smoothed")
        self.declare_parameter("output_topic", "cmd_vel_smoothed_corrected")
        self.declare_parameter("invert_linear_x", False)
        self.declare_parameter("invert_angular_z", True)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._invert_linear_x = bool(self.get_parameter("invert_linear_x").value)
        self._invert_angular_z = bool(self.get_parameter("invert_angular_z").value)

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._sub = self.create_subscription(Twist, self._input_topic, self._on_twist, 10)

        self.get_logger().info(
            "Twist transform %s -> %s invert_linear_x=%s invert_angular_z=%s"
            % (
                self._input_topic,
                self._output_topic,
                self._invert_linear_x,
                self._invert_angular_z,
            )
        )

    def _on_twist(self, msg: Twist) -> None:
        out = Twist()
        out.linear.x = -msg.linear.x if self._invert_linear_x else msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = -msg.angular.z if self._invert_angular_z else msg.angular.z
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = TwistTransformNode()
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
