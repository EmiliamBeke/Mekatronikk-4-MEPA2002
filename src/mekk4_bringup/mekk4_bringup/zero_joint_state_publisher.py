#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("zero_joint_state_publisher")

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("joint_names", ["left_wheel_joint", "right_wheel_joint"])
        self.declare_parameter("joint_positions", [0.0, 0.0])

        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        joint_positions = [float(value) for value in self.get_parameter("joint_positions").value]

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero.")
        if len(self._joint_names) != len(joint_positions):
            raise ValueError("joint_names and joint_positions must have the same length.")

        self._joint_positions = joint_positions
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_timer(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = list(self._joint_positions)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ZeroJointStatePublisher()
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
