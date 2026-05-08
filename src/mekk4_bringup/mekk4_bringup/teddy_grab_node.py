#!/usr/bin/env python3
from __future__ import annotations

import math
import re

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Int32, String


PARAM_DEFAULTS = {
    "enabled": False,
    "mode_topic": "/teddy_approach/mode",
    "trigger_mode": "close_enough_lidar",
    "cmd_vel_topic": "/cmd_vel_teddy",
    "stop_base_while_active": True,
    "x_topic": "/robotarm/request/x_position",
    "z_topic": "/robotarm/request/z_position",
    "left_gripper_topic": "/gripper/request/left_position",
    "right_gripper_topic": "/gripper/request/right_position",
    "x_state_topic": "/robotarm/x_position_state",
    "z_state_topic": "/robotarm/z_position_state",
    "distance_topic": "/mega/distance_mm",
    "teddy_status_topic": "/teddy_detector/status",
    "scan_topic": "/lidar",
    "publish_period_s": 0.05,
    "position_tolerance_m": 0.003,
    "distance_timeout_s": 0.5,
    "detector_status_timeout_s": 1.0,
    "scan_timeout_s": 0.5,
    "safe_x": 0.11,
    "safe_z": 0.12,
    "lower_z": 0.01,
    "grab_x": 0.15,
    "approach_x_max": 0.18,
    "approach_x_step_m": 0.002,
    "lift_z": 0.21,
    "final_x": 0.0,
    "gripper_open": 500.0,
    "gripper_closed": 1800.0,
    "move_hold_s": 0.2,
    "grab_hold_s": 0.8,
    "final_hold_s": 0.5,
    "require_state_feedback": True,
    "use_distance_contact": True,
    "require_distance_feedback": True,
    "contact_distance_mm": 0,
    "contact_hold_s": 0.25,
    "use_detector_dy_for_grab_z": False,
    "dy_to_z_gain_m": 0.04,
    "use_lidar_geometry_for_grab_z": False,
    "scan_front_angle_rad": 0.20,
    "scan_min_points": 3,
    "camera_vertical_fov_rad": 0.80,
    "camera_pitch_down_rad": 0.0,
    "lidar_geometry_z_origin_m": 0.01,
    "lidar_geometry_z_sign": -1.0,
    "lidar_geometry_z_offset_m": 0.0,
    "grab_z_min": 0.0,
    "grab_z_max": 0.12,
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
        self.distance_mm: int | None = None
        self.distance_received_at = -math.inf
        self.detector_dy: float | None = None
        self.detector_received_at = -math.inf
        self.front_lidar_distance_m = math.inf
        self.front_lidar_points = 0
        self.scan_received_at = -math.inf
        self.contact_started_at: float | None = None
        self.reach_x_target = float(self.param("grab_x"))
        self.grab_z_target = float(self.param("lower_z"))
        self.sequence = self.make_sequence()

        self.cmd_vel_pub = self.create_publisher(Twist, self.param("cmd_vel_topic"), 10)
        self.x_pub = self.create_publisher(Float64, self.param("x_topic"), 10)
        self.z_pub = self.create_publisher(Float64, self.param("z_topic"), 10)
        self.left_gripper_pub = self.create_publisher(Float64, self.param("left_gripper_topic"), 10)
        self.right_gripper_pub = self.create_publisher(Float64, self.param("right_gripper_topic"), 10)
        self.create_subscription(String, self.param("mode_topic"), self.on_mode, 10)
        self.create_subscription(Float64, self.param("x_state_topic"), self.on_x_state, 10)
        self.create_subscription(Float64, self.param("z_state_topic"), self.on_z_state, 10)
        self.create_subscription(Int32, self.param("distance_topic"), self.on_distance, 10)
        self.create_subscription(String, self.param("teddy_status_topic"), self.on_teddy_status, 10)
        self.create_subscription(LaserScan, self.param("scan_topic"), self.on_scan, 10)
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
        self.grab_z_target = self.compute_grab_z()
        self.reach_x_target = float(self.param("grab_x"))
        self.contact_started_at = None
        self.sequence = self.make_sequence()
        self.step_index = 0
        self.set_state(self.sequence[self.step_index]["name"])

    def on_x_state(self, msg: Float64) -> None:
        self.current_x = float(msg.data)

    def on_z_state(self, msg: Float64) -> None:
        self.current_z = float(msg.data)

    def on_distance(self, msg: Int32) -> None:
        self.distance_mm = int(msg.data)
        self.distance_received_at = self.now_s()

    def on_teddy_status(self, msg: String) -> None:
        match = re.search(r"(?:^| )dy=([-+]?[0-9]*\.?[0-9]+)", msg.data)
        if match is None:
            return
        self.detector_dy = float(match.group(1))
        self.detector_received_at = self.now_s()

    def on_scan(self, msg: LaserScan) -> None:
        self.front_lidar_distance_m = math.inf
        self.front_lidar_points = 0

        max_front_angle = float(self.param("scan_front_angle_rad"))
        angle = msg.angle_min
        for distance in msg.ranges:
            in_front = abs(angle) <= max_front_angle
            valid = math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
            if in_front and valid:
                self.front_lidar_points += 1
                self.front_lidar_distance_m = min(self.front_lidar_distance_m, float(distance))
            angle += msg.angle_increment

        self.scan_received_at = self.now_s()

    def on_timer(self) -> None:
        if not self.enabled or self.done or self.state == "idle":
            return

        self.publish_stop_if_active()
        elapsed = self.now_s() - self.state_started_at
        step = self.sequence[self.step_index]

        if step["name"] == "reach_until_contact":
            self.handle_contact_step(elapsed, step)
            return

        self.publish_targets(step["x"], step["z"], step["gripper"])

        if not self.ready_for_next(elapsed, step["hold_s"], step["x"], step["z"]):
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self.set_state("done")
            self.done = True
            self.publish_stop_if_active()
            return

        self.set_state(self.sequence[self.step_index]["name"])

    def set_state(self, state: str) -> None:
        self.state = state
        self.state_started_at = self.now_s()
        self.get_logger().info(f"state={state}")

    def make_sequence(self) -> list[dict[str, float | str]]:
        safe_x = float(self.param("safe_x"))
        safe_z = float(self.param("safe_z"))
        lower_z = self.grab_z_target
        grab_x = self.reach_x_target
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
                "name": "reach_until_contact",
                "x": grab_x,
                "z": lower_z,
                "gripper": open_gripper,
                "hold_s": 0.0,
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

    def compute_grab_z(self) -> float:
        base_z = float(self.param("lower_z"))
        if bool(self.param("use_lidar_geometry_for_grab_z")):
            return self.compute_lidar_geometry_grab_z(base_z)
        if not bool(self.param("use_detector_dy_for_grab_z")):
            return base_z
        if self.detector_dy is None:
            self.get_logger().warning("No teddy_detector dy available; using configured lower_z.")
            return base_z
        if self.now_s() - self.detector_received_at > float(self.param("detector_status_timeout_s")):
            self.get_logger().warning("teddy_detector dy is stale; using configured lower_z.")
            return base_z

        z = base_z + (self.detector_dy * float(self.param("dy_to_z_gain_m")))
        z_min = float(self.param("grab_z_min"))
        z_max = float(self.param("grab_z_max"))
        return max(z_min, min(z_max, z))

    def compute_lidar_geometry_grab_z(self, fallback_z: float) -> float:
        now = self.now_s()
        if self.detector_dy is None:
            self.get_logger().warning("No teddy_detector dy available; using configured lower_z.")
            return fallback_z
        if now - self.detector_received_at > float(self.param("detector_status_timeout_s")):
            self.get_logger().warning("teddy_detector dy is stale; using configured lower_z.")
            return fallback_z
        if now - self.scan_received_at > float(self.param("scan_timeout_s")):
            self.get_logger().warning("LiDAR scan is stale; using configured lower_z.")
            return fallback_z
        if self.front_lidar_points < int(self.param("scan_min_points")):
            self.get_logger().warning("Too few front LiDAR points; using configured lower_z.")
            return fallback_z
        if not math.isfinite(self.front_lidar_distance_m):
            self.get_logger().warning("No valid front LiDAR distance; using configured lower_z.")
            return fallback_z

        vertical_angle = float(self.param("camera_pitch_down_rad")) + (
            self.detector_dy * float(self.param("camera_vertical_fov_rad")) * 0.5
        )
        z_from_image = self.front_lidar_distance_m * math.tan(vertical_angle)
        z = (
            float(self.param("lidar_geometry_z_origin_m"))
            + float(self.param("lidar_geometry_z_sign")) * z_from_image
            + float(self.param("lidar_geometry_z_offset_m"))
        )
        z_min = float(self.param("grab_z_min"))
        z_max = float(self.param("grab_z_max"))
        z = max(z_min, min(z_max, z))
        self.get_logger().info(
            "computed grab_z=%.3f from dy=%.3f lidar=%.3f angle=%.3f"
            % (z, self.detector_dy, self.front_lidar_distance_m, vertical_angle)
        )
        return z

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

    def publish_stop_if_active(self) -> None:
        if not bool(self.param("stop_base_while_active")):
            return
        self.cmd_vel_pub.publish(Twist())

    def handle_contact_step(self, elapsed: float, step: dict[str, float | str]) -> None:
        del elapsed
        self.publish_targets(self.reach_x_target, float(step["z"]), float(step["gripper"]))

        if not bool(self.param("use_distance_contact")):
            self.finish_contact_step()
            return

        now = self.now_s()
        distance_fresh = now - self.distance_received_at <= float(self.param("distance_timeout_s"))
        contact = (
            distance_fresh
            and self.distance_mm is not None
            and self.distance_mm >= 0
            and self.distance_mm <= int(self.param("contact_distance_mm"))
        )

        if contact:
            if self.contact_started_at is None:
                self.contact_started_at = now
            if now - self.contact_started_at >= float(self.param("contact_hold_s")):
                self.finish_contact_step()
            return

        self.contact_started_at = None
        if bool(self.param("require_distance_feedback")) and not distance_fresh:
            return

        if not self.target_reached(self.reach_x_target, float(step["z"])):
            return

        next_x = self.reach_x_target + float(self.param("approach_x_step_m"))
        self.reach_x_target = min(float(self.param("approach_x_max")), next_x)

    def finish_contact_step(self) -> None:
        for step in self.sequence:
            if step["name"] in ("grab_servo", "retract_x_90mm", "lift_z_210mm", "home_x_0mm"):
                if step["name"] == "grab_servo":
                    step["x"] = self.reach_x_target
                elif step["name"] == "retract_x_90mm":
                    step["z"] = self.grab_z_target
                elif step["name"] == "lift_z_210mm":
                    step["x"] = float(self.param("safe_x"))
                elif step["name"] == "home_x_0mm":
                    step["z"] = float(self.param("lift_z"))
        self.step_index += 1
        self.set_state(self.sequence[self.step_index]["name"])

    def ready_for_next(self, elapsed: float, hold_s: float, target_x: float, target_z: float) -> bool:
        if elapsed < float(hold_s):
            return False
        if self.current_x is None or self.current_z is None:
            return not bool(self.param("require_state_feedback"))

        tolerance = float(self.param("position_tolerance_m"))
        return self.target_reached(target_x, target_z, tolerance)

    def target_reached(
        self, target_x: float, target_z: float, tolerance: float | None = None
    ) -> bool:
        if self.current_x is None or self.current_z is None:
            return not bool(self.param("require_state_feedback"))
        if tolerance is None:
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
