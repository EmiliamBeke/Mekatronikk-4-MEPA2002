#!/usr/bin/env python3

import math
import re

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float64, Int32, String


class TeddyGrabNode(Node):
    # Set up ROS IO and local grab state.
    def __init__(self):
        super().__init__("teddy_grab", automatically_declare_parameters_from_overrides=True)

        self.enabled = bool(self.p("enabled"))
        self.trigger_mode = str(self.p("trigger_mode"))
        self.state = "idle"
        self.step_i = -1
        self.step_t0 = self.now_s()

        self.x = None
        self.z = None
        self.distance_mm = None
        self.distance_t = -math.inf
        self.dy = None
        self.dy_t = -math.inf
        self.lidar_m = math.inf
        self.lidar_points = 0
        self.scan_t = -math.inf

        self.grab_z = float(self.p("lower_z"))
        self.grab_z_calc = "not computed"
        self.reach_x = float(self.p("final_x"))
        self.retry_count = 0
        self.contact_t = None
        self.sequence = []
        self.target_x = None
        self.target_z = None
        self.target_gripper = None
        self.last_gripper_us = None
        self.last_log_t = -math.inf

        self.cmd_pub = self.create_publisher(Twist, self.p("cmd_vel_topic"), 10)
        self.reset_pub = self.create_publisher(Empty, self.p("approach_reset_topic"), 10)
        self.x_pub = self.create_publisher(Float64, "/robotarm/request/x_position", 10)
        self.z_pub = self.create_publisher(Float64, "/robotarm/request/z_position", 10)
        self.left_pub = self.create_publisher(Float64, "/gripper/request/left_position", 10)
        self.right_pub = self.create_publisher(Float64, "/gripper/request/right_position", 10)

        self.create_subscription(String, self.p("mode_topic"), self.on_mode, 10)
        self.create_subscription(Float64, self.p("x_state_topic"), self.on_x_state, 10)
        self.create_subscription(Float64, self.p("z_state_topic"), self.on_z_state, 10)
        self.create_subscription(Int32, self.p("distance_topic"), self.on_distance, 10)
        self.create_subscription(String, self.p("teddy_status_topic"), self.on_teddy_status, 10)
        self.create_subscription(LaserScan, self.p("scan_topic"), self.on_scan, 10)
        self.create_timer(float(self.p("publish_period_s")), self.on_timer)

        self.get_logger().info("teddy grab enabled=%s trigger=%s" % (self.enabled, self.trigger_mode))

    # Read a ROS parameter.
    def p(self, name):
        return self.get_parameter(name).value

    # Return ROS time in seconds.
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # Start grab when teddy_approach reports settled.
    def on_mode(self, msg):
        if not self.enabled or self.state != "idle" or msg.data != self.trigger_mode:
            return
        self.grab_z = self.compute_grab_z()
        self.reach_x = float(self.p("final_x"))
        self.retry_count = 0
        self.contact_t = None
        self.last_gripper_us = None
        self.sequence = self.make_sequence()
        self.step_i = 0
        self.get_logger().info("Phase 2: grab_z=%.3f m (%s)" % (self.grab_z, self.grab_z_calc))
        self.enter_step()

    # Store X feedback from Mega.
    def on_x_state(self, msg):
        self.x = float(msg.data)

    # Store Z feedback from Mega.
    def on_z_state(self, msg):
        self.z = float(msg.data) - self.z_offset()

    # Store gripper distance sensor.
    def on_distance(self, msg):
        self.distance_mm = int(msg.data)
        self.distance_t = self.now_s()

    # Store teddy detector dy.
    def on_teddy_status(self, msg):
        match = re.search(r"(?:^| )dy=([-+]?[0-9]*\.?[0-9]+)", msg.data)
        if match:
            self.dy = float(match.group(1))
            self.dy_t = self.now_s()

    # Store nearest front LiDAR reading.
    def on_scan(self, msg):
        self.lidar_m = math.inf
        self.lidar_points = 0
        angle = msg.angle_min
        max_angle = float(self.p("scan_front_angle_rad"))
        for distance in msg.ranges:
            valid = math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
            if valid and abs(angle) <= max_angle:
                self.lidar_points += 1
                self.lidar_m = min(self.lidar_m, float(distance))
            angle += msg.angle_increment
        self.scan_t = self.now_s()

    # Run the active sequence step.
    def on_timer(self):
        if not self.enabled or self.state in ("idle", "done"):
            return
        if bool(self.p("stop_base_while_active")):
            self.cmd_pub.publish(Twist())
        if 0 <= self.step_i < len(self.sequence):
            self.run_step(self.sequence[self.step_i])

    # Normal grab sequence. X and Z are separate actions.
    def make_sequence(self):
        f_x = float(self.p("final_x"))
        g_open = float(self.p("gripper_open"))
        g_closed = float(self.p("gripper_closed"))
        move_s = float(self.p("move_hold_s"))
        return [
            # Phase 1: wait after approach handoff.
            self.step("settle", "Phase 1", "wait", f_x, self.hold_z(), g_open, self.p("settle_after_trigger_s")),
            # Phase 3: move X to 0.
            self.step("move_x_zero", "Phase 3", "x", f_x, self.hold_z(), g_open, move_s),
            # Phase 4: move Z to computed teddy center height.
            self.step("move_z_grab", "Phase 4", "z", f_x, self.grab_z, g_open, move_s),
            # Phase 5: extend X until distance contact is stable.
            self.step("reach_until_contact", "Phase 5", "reach", f_x, self.grab_z, g_open, 0.0),
            # Phase 6: close gripper.
            self.step("grab_servo", "Phase 6", "gripper", f_x, self.grab_z, g_closed, self.p("grab_hold_s")),
            # Phase 7: verify teddy is still held.
            self.step("verify_grab", "Phase 7", "verify_grab", f_x, self.grab_z, g_closed, self.p("verify_hold_s")),
            # Phase 8: retract X to 0.
            self.step("retract_x", "Phase 8", "x", f_x, self.grab_z, g_closed, move_s),
            # Phase 8: lift Z after X is retracted.
            self.step("lift_z", "Phase 8", "z", f_x, self.p("lift_z"), g_closed, move_s),
            # Phase 9: final distance check.
            self.step("verify_final", "Phase 9", "verify_final", f_x, self.p("lift_z"), g_closed, self.p("verify_hold_s")),
        ]

    # Retry once after failed grab verification.
    def make_retry_sequence(self):
        f_x = float(self.p("final_x"))
        g_open = float(self.p("gripper_open"))
        g_closed = float(self.p("gripper_closed"))
        move_s = float(self.p("move_hold_s"))
        return [
            # Phase 7.1: open gripper.
            self.step("retry_open", "Phase 7.1", "gripper", self.reach_x, self.grab_z, g_open, move_s),
            # Phase 7.1: retract X to 0.
            self.step("retry_x_zero", "Phase 7.1", "x", f_x, self.grab_z, g_open, move_s),
            # Phase 5: reach again.
            self.step("reach_until_contact", "Phase 5", "reach", f_x, self.grab_z, g_open, 0.0),
            # Phase 6: close gripper again.
            self.step("grab_servo", "Phase 6", "gripper", f_x, self.grab_z, g_closed, self.p("grab_hold_s")),
            # Phase 7: verify second attempt.
            self.step("verify_grab", "Phase 7", "verify_grab", f_x, self.grab_z, g_closed, self.p("verify_hold_s")),
            # Phase 8: retract X to 0.
            self.step("retract_x", "Phase 8", "x", f_x, self.grab_z, g_closed, move_s),
            # Phase 8: lift Z.
            self.step("lift_z", "Phase 8", "z", f_x, self.p("lift_z"), g_closed, move_s),
            # Phase 9: final distance check.
            self.step("verify_final", "Phase 9", "verify_final", f_x, self.p("lift_z"), g_closed, self.p("verify_hold_s")),
        ]

    # Reset arm before handing control back to teddy_approach.
    def make_restart_sequence(self):
        f_x = float(self.p("final_x"))
        z0 = float(self.p("lower_z"))
        g_open = float(self.p("gripper_open"))
        move_s = float(self.p("move_hold_s"))
        return [
            # Phase 7.2: open gripper.
            self.step("restart_open", "Phase 7.2", "gripper", self.x_or(f_x), self.hold_z(), g_open, move_s),
            # Phase 7.2: retract X to 0.
            self.step("restart_x_zero", "Phase 7.2", "x", f_x, self.hold_z(), g_open, move_s),
            # Phase 7.2: lower Z to 0.
            self.step("restart_z_zero", "Phase 7.2", "z", f_x, z0, g_open, move_s),
            # Phase 7.2: reset teddy_approach.
            self.step("restart_approach", "Phase 7.2", "reset", f_x, z0, g_open, self.p("final_hold_s")),
        ]

    # Create one sequence dictionary.
    def step(self, name, phase, action, x, z, gripper, hold_s):
        return {
            "name": name,
            "phase": phase,
            "action": action,
            "x": float(x),
            "z": float(z),
            "gripper": float(gripper),
            "hold_s": float(hold_s),
        }

    # Execute one sequence dictionary.
    def run_step(self, step):
        self.target_x, self.target_z, self.target_gripper = step["x"], step["z"], step["gripper"]
        action = step["action"]
        if action == "wait":
            self.command_gripper(step["gripper"])
            self.advance_after(step["hold_s"])
        elif action == "x":
            self.command_gripper(step["gripper"])
            self.command_x(step["x"])
            if self.axis_reached("x", step["x"], step["hold_s"]):
                self.next_step()
        elif action == "z":
            self.command_gripper(step["gripper"])
            self.command_z(step["z"])
            if self.axis_reached("z", step["z"], step["hold_s"]):
                self.next_step()
        elif action == "gripper":
            self.command_gripper(step["gripper"])
            self.advance_after(step["hold_s"])
        elif action == "reach":
            self.reach_until_contact(step)
        elif action == "verify_grab":
            self.verify_grab(step)
        elif action == "verify_final":
            self.verify_final(step)
        elif action == "reset":
            self.advance_after(step["hold_s"], reset=True)
        self.log_status()

    # Enter the current step after step_i changes.
    def enter_step(self):
        self.state = self.sequence[self.step_i]["name"]
        self.step_t0 = self.now_s()
        self.contact_t = None
        self.last_log_t = -math.inf
        self.get_logger().info("%s: %s" % (self.phase(), self.state))

    # Move to next sequence step.
    def next_step(self):
        self.step_i += 1
        if self.step_i >= len(self.sequence):
            self.state = "done"
            self.get_logger().info("done")
            return
        self.enter_step()

    # Wait for hold time, optionally reset teddy_approach.
    def advance_after(self, hold_s, reset=False):
        if self.elapsed_s() < hold_s:
            return
        if reset:
            self.state = "idle"
            self.step_i = -1
            self.reset_pub.publish(Empty())
            self.get_logger().info("state=idle waiting for next teddy_approach_settled")
        else:
            self.next_step()

    # Command only X.
    def command_x(self, x):
        self.publish(self.x_pub, x)

    # Command only Z.
    def command_z(self, z):
        self.publish(self.z_pub, z + self.z_offset())

    # Convert Mega's physical Z to teddy_grab's zero-centered work Z.
    def z_offset(self):
        return float(self.p("z_offset_m"))

    # Command gripper servos.
    def command_gripper(self, us):
        if self.last_gripper_us == us:
            return
        self.last_gripper_us = us
        self.publish(self.left_pub, us)
        self.publish(self.right_pub, us)

    # Extend X in small steps until contact is stable.
    def reach_until_contact(self, step):
        self.command_gripper(step["gripper"])
        self.command_x(self.reach_x)
        self.target_x = self.reach_x
        if self.stable_contact_for(float(self.p("contact_hold_s"))) and self.axis_reached("x", self.reach_x, 0.0):
            self.patch_reach_x()
            self.next_step()
            return
        if bool(self.p("require_distance_feedback")) and not self.distance_fresh():
            return
        if not self.axis_reached("x", self.reach_x, 0.0):
            return
        max_x = float(self.p("approach_x_max"))
        if self.reach_x >= max_x - 1e-9:
            self.get_logger().warning("no stable distance contact at max reach; restarting approach")
            self.start_restart()
            return
        self.reach_x = min(max_x, self.reach_x + float(self.p("approach_x_step_m")))
        step["x"] = self.reach_x

    # Verify contact after closing gripper.
    def verify_grab(self, step):
        self.command_gripper(step["gripper"])
        if self.stable_contact_for(step["hold_s"]):
            self.next_step()
            return
        if self.elapsed_s() < step["hold_s"]:
            return
        self.retry_count += 1
        if self.retry_count == 1:
            self.get_logger().warning("grab contact lost; retrying once")
            self.sequence = self.make_retry_sequence()
            self.step_i = 0
            self.enter_step()
        else:
            self.get_logger().warning("grab contact lost twice; restarting approach")
            self.start_restart()

    # Verify contact after retract/lift.
    def verify_final(self, step):
        self.command_gripper(step["gripper"])
        if self.stable_contact_for(step["hold_s"]):
            self.state = "done"
            self.get_logger().info("teddy grab successful")
            return
        if self.elapsed_s() >= step["hold_s"]:
            self.get_logger().warning("final contact check failed; restarting approach")
            self.start_restart()

    # Start restart sequence.
    def start_restart(self):
        self.sequence = self.make_restart_sequence()
        self.step_i = 0
        self.enter_step()

    # Keep later status targets consistent with final reach X.
    def patch_reach_x(self):
        for step in self.sequence[self.step_i + 1 :]:
            if step["name"] in ("grab_servo", "verify_grab"):
                step["x"] = self.reach_x

    # Verify requested axis reached target using Mega feedback.
    def axis_reached(self, axis, target, hold_s):
        if self.elapsed_s() < hold_s:
            return False
        value = self.x if axis == "x" else self.z
        if value is None:
            return not bool(self.p("require_state_feedback"))
        if axis == "x" and target == float(self.p("final_x")):
            return value >= target
        return abs(value - target) <= float(self.p("position_tolerance_m"))

    # Compute one locked Z target before the camera is blocked.
    def compute_grab_z(self):
        fallback = float(self.p("lower_z"))
        if bool(self.p("use_lidar_geometry_for_grab_z")):
            return self.compute_lidar_grab_z(fallback)
        if not bool(self.p("use_detector_dy_for_grab_z")):
            self.grab_z_calc = "configured lower_z=%.3f" % fallback
            return fallback
        if self.dy is None or self.now_s() - self.dy_t > float(self.p("detector_status_timeout_s")):
            self.grab_z_calc = "fallback lower_z=%.3f because dy missing/stale" % fallback
            return fallback
        z = self.clamp_z(fallback + self.dy * float(self.p("dy_to_z_gain_m")))
        self.grab_z_calc = "dy=%.3f gain=%.3f -> z=%.3f" % (self.dy, self.p("dy_to_z_gain_m"), z)
        return z

    # Estimate teddy height from detector dy and LiDAR distance.
    def compute_lidar_grab_z(self, fallback):
        now = self.now_s()
        if self.dy is None or now - self.dy_t > float(self.p("detector_status_timeout_s")):
            self.grab_z_calc = "fallback lower_z=%.3f because dy missing/stale" % fallback
            return fallback
        if now - self.scan_t > float(self.p("scan_timeout_s")):
            self.grab_z_calc = "fallback lower_z=%.3f because LiDAR stale" % fallback
            return fallback
        if self.lidar_points < int(self.p("scan_min_points")) or not math.isfinite(self.lidar_m):
            self.grab_z_calc = "fallback lower_z=%.3f because LiDAR invalid" % fallback
            return fallback

        angle = float(self.p("camera_pitch_down_rad")) + self.dy * float(self.p("camera_vertical_fov_rad")) * 0.5
        raw_z = (
            float(self.p("lidar_geometry_z_origin_m"))
            + float(self.p("lidar_geometry_z_sign")) * self.lidar_m * math.tan(angle)
            + float(self.p("lidar_geometry_z_offset_m"))
        )
        z = self.clamp_z(raw_z)
        self.grab_z_calc = "dy=%.3f lidar=%.3f angle=%.3f -> z=%.3f" % (self.dy, self.lidar_m, angle, z)
        return z

    # Clamp computed grab height.
    def clamp_z(self, z):
        return max(float(self.p("grab_z_min")), min(float(self.p("grab_z_max")), z))

    # Hold current Z while moving X.
    def hold_z(self):
        return self.z if self.z is not None else self.grab_z

    # Hold current X for restart metadata.
    def x_or(self, fallback):
        return self.x if self.x is not None else fallback

    # Check current distance contact.
    def has_contact(self):
        return self.distance_fresh() and self.distance_mm is not None and 0 <= self.distance_mm <= int(self.p("contact_distance_mm"))

    # Require continuous contact for seconds.
    def stable_contact_for(self, seconds):
        now = self.now_s()
        if not self.has_contact():
            self.contact_t = None
            return False
        if self.contact_t is None:
            self.contact_t = now
        return now - self.contact_t >= seconds

    # Check distance reading age.
    def distance_fresh(self):
        return self.now_s() - self.distance_t <= float(self.p("distance_timeout_s"))

    # Time since current step started.
    def elapsed_s(self):
        return self.now_s() - self.step_t0

    # Current phase label.
    def phase(self):
        if 0 <= self.step_i < len(self.sequence):
            return self.sequence[self.step_i]["phase"]
        return self.state

    # Periodic terminal status.
    def log_status(self):
        now = self.now_s()
        if now - self.last_log_t < float(self.p("status_log_period_s")):
            return
        self.last_log_t = now
        self.get_logger().info(
            "%s: %s | target x=%s z=%s cmd_z=%s gripper=%s | current x=%s z=%s | dist=%s | grab_z_calc=%s"
            % (
                self.phase(),
                self.state,
                self.fmt(self.target_x, "m"),
                self.fmt(self.target_z, "m"),
                self.fmt(None if self.target_z is None else self.target_z + self.z_offset(), "m"),
                self.fmt(self.target_gripper, "us"),
                self.fmt(self.x, "m"),
                self.fmt(self.z, "m"),
                self.distance_text(),
                self.grab_z_calc,
            )
        )

    # Format a nullable value.
    def fmt(self, value, unit):
        if value is None:
            return "unknown"
        return "%.0f us" % value if unit == "us" else "%.3f m" % value

    # Format distance status.
    def distance_text(self):
        if self.distance_mm is None:
            return "unknown"
        return "%d mm %s age=%.2fs" % (
            self.distance_mm,
            "fresh" if self.distance_fresh() else "stale",
            self.now_s() - self.distance_t,
        )

    # Publish one Float64.
    def publish(self, publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)


def main():
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
