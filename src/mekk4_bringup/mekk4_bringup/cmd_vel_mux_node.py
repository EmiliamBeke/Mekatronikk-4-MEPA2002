#!/usr/bin/env python3
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def copy_twist(msg: Twist) -> Twist:
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.linear.z = msg.linear.z
    out.angular.x = msg.angular.x
    out.angular.y = msg.angular.y
    out.angular.z = msg.angular.z
    return out


def zero_twist() -> Twist:
    return Twist()


class CmdVelMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("nav_input_topic", "cmd_vel_nav_flipped")
        self.declare_parameter("assist_input_topic", "cmd_vel_assist")
        self.declare_parameter("manual_input_topic", "cmd_vel_manual")
        self.declare_parameter("output_topic", "cmd_vel_muxed")
        self.declare_parameter("active_source_topic", "cmd_vel_mux_active")
        self.declare_parameter("manual_timeout_s", 0.25)
        self.declare_parameter("assist_timeout_s", 0.5)
        self.declare_parameter("nav_timeout_s", 0.5)
        self.declare_parameter("publish_period_s", 0.05)

        nav_input_topic = self.get_parameter("nav_input_topic").get_parameter_value().string_value
        assist_input_topic = (
            self.get_parameter("assist_input_topic").get_parameter_value().string_value
        )
        manual_input_topic = (
            self.get_parameter("manual_input_topic").get_parameter_value().string_value
        )
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        active_source_topic = (
            self.get_parameter("active_source_topic").get_parameter_value().string_value
        )
        self._manual_timeout_s = (
            self.get_parameter("manual_timeout_s").get_parameter_value().double_value
        )
        self._assist_timeout_s = (
            self.get_parameter("assist_timeout_s").get_parameter_value().double_value
        )
        self._nav_timeout_s = self.get_parameter("nav_timeout_s").get_parameter_value().double_value
        publish_period_s = self.get_parameter("publish_period_s").get_parameter_value().double_value

        self._cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self._source_pub = self.create_publisher(String, active_source_topic, 10)
        self._nav_sub = self.create_subscription(Twist, nav_input_topic, self._on_nav_cmd, 10)
        self._assist_sub = self.create_subscription(
            Twist, assist_input_topic, self._on_assist_cmd, 10
        )
        self._manual_sub = self.create_subscription(
            Twist, manual_input_topic, self._on_manual_cmd, 10
        )
        self._timer = self.create_timer(publish_period_s, self._on_timer)

        self._last_nav_cmd = zero_twist()
        self._last_nav_at = -1.0
        self._last_assist_cmd = zero_twist()
        self._last_assist_at = -1.0
        self._last_manual_cmd = zero_twist()
        self._last_manual_at = -1.0
        self._last_source = ""

        self.get_logger().info(
            "Muxing manual=%s assist=%s nav=%s -> %s"
            % (manual_input_topic, assist_input_topic, nav_input_topic, output_topic)
        )

    def _on_nav_cmd(self, msg: Twist) -> None:
        self._last_nav_cmd = copy_twist(msg)
        self._last_nav_at = time.monotonic()

    def _on_manual_cmd(self, msg: Twist) -> None:
        self._last_manual_cmd = copy_twist(msg)
        self._last_manual_at = time.monotonic()

    def _on_assist_cmd(self, msg: Twist) -> None:
        self._last_assist_cmd = copy_twist(msg)
        self._last_assist_at = time.monotonic()

    def _select_command(self) -> tuple[str, Twist]:
        now = time.monotonic()

        manual_active = (
            self._last_manual_at >= 0.0 and (now - self._last_manual_at) <= self._manual_timeout_s
        )
        if manual_active:
            return "manual", self._last_manual_cmd

        assist_active = (
            self._last_assist_at >= 0.0 and (now - self._last_assist_at) <= self._assist_timeout_s
        )
        if assist_active:
            return "assist", self._last_assist_cmd

        nav_active = self._last_nav_at >= 0.0 and (now - self._last_nav_at) <= self._nav_timeout_s
        if nav_active:
            return "nav", self._last_nav_cmd

        return "idle", zero_twist()

    def _on_timer(self) -> None:
        source, cmd = self._select_command()
        self._cmd_pub.publish(cmd)

        source_msg = String()
        source_msg.data = source
        self._source_pub.publish(source_msg)

        if source != self._last_source:
            self.get_logger().info(f"Active cmd_vel source: {source}")
            self._last_source = source


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
