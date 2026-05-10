#!/usr/bin/env python3
"""Publish /robotarm/{x,z}_position_state from /joint_states in sim.

Mirrors what mega_driver_node provides on real hardware so nodes that gate on
arm position feedback (teddy_grab_node, robotarm_safety_node) work in Gazebo.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RobotarmStateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("robotarm_state_adapter")
        self.declare_parameter("x_joint_name", "robotarm_x_joint")
        self.declare_parameter("z_joint_name", "robotarm_z_joint")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("x_state_topic", "/robotarm/x_position_state")
        self.declare_parameter("z_state_topic", "/robotarm/z_position_state")

        self.x_name = str(self.get_parameter("x_joint_name").value)
        self.z_name = str(self.get_parameter("z_joint_name").value)

        self.x_pub = self.create_publisher(
            Float64, str(self.get_parameter("x_state_topic").value), 10
        )
        self.z_pub = self.create_publisher(
            Float64, str(self.get_parameter("z_state_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_js,
            10,
        )

    def _on_js(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position):
            if name == self.x_name:
                self.x_pub.publish(Float64(data=float(value)))
            elif name == self.z_name:
                self.z_pub.publish(Float64(data=float(value)))


def main() -> None:
    rclpy.init()
    node = RobotarmStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
