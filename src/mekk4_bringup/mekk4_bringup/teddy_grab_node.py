#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


PARAM_DEFAULTS = {
    "enabled": False,
    "mode_topic": "/teddy_approach/mode",
    "trigger_mode": "close_enough_lidar",
    "x_topic": "/robotarm/request/x_position",
    "z_topic": "/robotarm/request/z_position",
    "left_gripper_topic": "/gripper/request/left_position",
    "right_gripper_topic": "/gripper/request/right_position",
    "x_state_topic": "/robotarm/x_position_state",
    "z_state_topic": "/robotarm/z_position_state",
    "publish_period_s": 0.05,
    "position_tolerance_m": 0.003,
    "pre_x": 0.0,
    "pre_z": 0.12,
    "reach_x": 0.12,
    "reach_z": 0.01,
    "carry_x": 0.0,
    "carry_z": 0.2,
    "gripper_open": 0.45,
    "gripper_closed": -0.45,
    "pre_hold_s": 0.8,
    "reach_hold_s": 1.2,
    "close_hold_s": 0.8,
    "lift_hold_s": 1.0,
}


class TeddyGrabNode(Node):
    def __init__(self) -> None:
        super().__init__("teddy_grab")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.enabled = bool(self.param("enabled"))
        self.trigger_mode = str(self.param("trigger_mode"))
        self.state = "idle"
        self.state_started_at = self.now_s()
        self.done = False
        self.current_x: float | None = None
        self.current_z: float | None = None

        self.x_pub = self.create_publisher(Float64, self.param("x_topic"), 10)
        self.z_pub = self.create_publisher(Float64, self.param("z_topic"), 10)
        self.left_gripper_pub = self.create_publisher(Float64, self.param("left_gripper_topic"), 10)
        self.right_gripper_pub = self.create_publisher(Float64, self.param("right_gripper_topic"), 10)
        self.create_subscription(String, self.param("mode_topic"), self.on_mode, 10)
        self.create_subscription(Float64, self.param("x_state_topic"), self.on_x_state, 10)
        self.create_subscription(Float64, self.param("z_state_topic"), self.on_z_state, 10)
        self.create_timer(float(self.param("publish_period_s")), self.on_timer)

        self.get_logger().info(
            "teddy grab enabled=%s trigger_mode=%s" % (self.enabled, self.trigger_mode)
        )

    def param(self, name: str):
        return self.get_parameter(name).value

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_mode(self, msg: String) -> None:
        if not self.enabled or self.done or self.state != "idle":
            return
        if msg.data != self.trigger_mode:
            return
        self.set_state("pre_grab")

    def on_x_state(self, msg: Float64) -> None:
        self.current_x = float(msg.data)

    def on_z_state(self, msg: Float64) -> None:
        self.current_z = float(msg.data)

    def on_timer(self) -> None:
        if not self.enabled or self.state == "idle":
            return

        elapsed = self.now_s() - self.state_started_at

        if self.state == "pre_grab":
            self.publish_targets(self.param("pre_x"), self.param("pre_z"), self.param("gripper_open"))
            if self.ready_for_next(elapsed, self.param("pre_hold_s"), self.param("pre_x"), self.param("pre_z")):
                self.set_state("reach")
        elif self.state == "reach":
            self.publish_targets(self.param("reach_x"), self.param("reach_z"), self.param("gripper_open"))
            if self.ready_for_next(elapsed, self.param("reach_hold_s"), self.param("reach_x"), self.param("reach_z")):
                self.set_state("close")
        elif self.state == "close":
            self.publish_targets(self.param("reach_x"), self.param("reach_z"), self.param("gripper_closed"))
            if self.ready_for_next(elapsed, self.param("close_hold_s"), self.param("reach_x"), self.param("reach_z")):
                self.set_state("lift")
        elif self.state == "lift":
            self.publish_targets(self.param("carry_x"), self.param("carry_z"), self.param("gripper_closed"))
            if self.ready_for_next(elapsed, self.param("lift_hold_s"), self.param("carry_x"), self.param("carry_z")):
                self.set_state("done")
                self.done = True

    def set_state(self, state: str) -> None:
        self.state = state
        self.state_started_at = self.now_s()
        self.get_logger().info(f"state={state}")

    def publish_targets(self, x: float, z: float, gripper: float) -> None:
        x_msg = Float64()
        x_msg.data = float(x)
        self.x_pub.publish(x_msg)

        z_msg = Float64()
        z_msg.data = float(z)
        self.z_pub.publish(z_msg)

        left_msg = Float64()
        left_msg.data = float(gripper)
        self.left_gripper_pub.publish(left_msg)

        right_msg = Float64()
        right_msg.data = -float(gripper)
        self.right_gripper_pub.publish(right_msg)

    def ready_for_next(self, elapsed: float, hold_s: float, target_x: float, target_z: float) -> bool:
        if elapsed < float(hold_s):
            return False
        if self.current_x is None or self.current_z is None:
            return True

        tolerance = float(self.param("position_tolerance_m"))
        return (
            abs(self.current_x - float(target_x)) <= tolerance
            and abs(self.current_z - float(target_z)) <= tolerance
        )


def main() -> None:
    rclpy.init()
    node = TeddyGrabNode()
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
