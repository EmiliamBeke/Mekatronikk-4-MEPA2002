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
    "safe_x": 0.11,
    "safe_z": 0.12,
    "lower_z": 0.01,
    "grab_x": 0.15,
    "lift_z": 0.21,
    "final_x": 0.0,
    "gripper_open": 500.0,
    "gripper_closed": 1800.0,
    "move_hold_s": 0.2,
    "grab_hold_s": 0.8,
    "final_hold_s": 0.5,
}


class TeddyGrabNode(Node):
    def __init__(self) -> None:
        super().__init__("teddy_grab")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.enabled = bool(self.param("enabled"))
        self.trigger_mode = str(self.param("trigger_mode"))
        self.state = "idle"
        self.step_index = -1
        self.state_started_at = self.now_s()
        self.done = False
        self.current_x: float | None = None
        self.current_z: float | None = None
        self.sequence = self.make_sequence()

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
        self.step_index = 0
        self.set_state(self.sequence[self.step_index]["name"])

    def on_x_state(self, msg: Float64) -> None:
        self.current_x = float(msg.data)

    def on_z_state(self, msg: Float64) -> None:
        self.current_z = float(msg.data)

    def on_timer(self) -> None:
        if not self.enabled or self.done or self.state == "idle":
            return

        elapsed = self.now_s() - self.state_started_at
        step = self.sequence[self.step_index]
        self.publish_targets(step["x"], step["z"], step["gripper"])

        if not self.ready_for_next(elapsed, step["hold_s"], step["x"], step["z"]):
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self.set_state("done")
            self.done = True
            return

        self.set_state(self.sequence[self.step_index]["name"])

    def set_state(self, state: str) -> None:
        self.state = state
        self.state_started_at = self.now_s()
        self.get_logger().info(f"state={state}")

    def make_sequence(self) -> list[dict[str, float | str]]:
        safe_x = float(self.param("safe_x"))
        safe_z = float(self.param("safe_z"))
        lower_z = float(self.param("lower_z"))
        grab_x = float(self.param("grab_x"))
        lift_z = float(self.param("lift_z"))
        final_x = float(self.param("final_x"))
        open_gripper = float(self.param("gripper_open"))
        closed_gripper = float(self.param("gripper_closed"))
        move_hold_s = float(self.param("move_hold_s"))

        return [
            {
                "name": "safe_x_110mm",
                "x": safe_x,
                "z": safe_z,
                "gripper": open_gripper,
                "hold_s": move_hold_s,
            },
            {
                "name": "lower_z_10mm",
                "x": safe_x,
                "z": lower_z,
                "gripper": open_gripper,
                "hold_s": move_hold_s,
            },
            {
                "name": "reach_x_150mm",
                "x": grab_x,
                "z": lower_z,
                "gripper": open_gripper,
                "hold_s": move_hold_s,
            },
            {
                "name": "grab_servo",
                "x": grab_x,
                "z": lower_z,
                "gripper": closed_gripper,
                "hold_s": float(self.param("grab_hold_s")),
            },
            {
                "name": "retract_x_90mm",
                "x": safe_x,
                "z": lower_z,
                "gripper": closed_gripper,
                "hold_s": move_hold_s,
            },
            {
                "name": "lift_z_210mm",
                "x": safe_x,
                "z": lift_z,
                "gripper": closed_gripper,
                "hold_s": move_hold_s,
            },
            {
                "name": "home_x_0mm",
                "x": final_x,
                "z": lift_z,
                "gripper": closed_gripper,
                "hold_s": float(self.param("final_hold_s")),
            },
        ]

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
        right_msg.data = float(gripper)
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
