#!/usr/bin/env python3
from __future__ import annotations

import math
import re

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String


STATUS_RE = re.compile(r"(?P<key>[A-Za-z_]+)=(?P<value>[^ ]+)")

PARAM_DEFAULTS = {
    "enabled": False,
    "status_topic": "/teddy_detector/status",
    "cmd_vel_topic": "/cmd_vel_teddy",
    "publish_period_s": 0.05,
    "lost_timeout_s": 0.5,
    "linear_speed": 0.08,
    "drive_when_not_centered": False,
    "center_tolerance": 0.10,
    "center_settle_s": 2.0,
    "angular_kp": 1.2,
    "angular_kd": 0.0,
    "min_angular_speed": 0.0,
    "max_angular_speed": 0.8,
    "scan_topic": "/lidar",
    "stop_lidar_distance_m": 0.0,
    "stop_lidar_front_angle_rad": 0.20,
    "stop_lidar_min_points": 3,
    "stop_lidar_timeout_s": 0.5,
    "mode_topic": "/teddy_approach/mode",
    "reset_topic": "/teddy_approach/reset",
}


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_status(text):
    return {match.group("key"): match.group("value") for match in STATUS_RE.finditer(text)}


class BasicPd:
    """Small P/PD controller in the same direct style as basic-pid."""

    def __init__(self):
        self._Kp = 1.0
        self._Kd = 0.0
        self._min_out = 0.0
        self._max_out = 1.0
        self.reset()

    def reset(self):
        self._e_prev = None
        self._t_prev = None
        self._P = 0.0
        self._D = 0.0
        self._out = 0.0

    def setGains(self, Kp, Kd):
        self._Kp = Kp
        self._Kd = Kd

    def setLimits(self, min_out, max_out):
        self._min_out = min_out
        self._max_out = max_out

    def get(self, signal_ref, signal, now):
        return self.pid(signal_ref, signal, now)

    def pid(self, signal_ref, signal, now):
        e = signal_ref - signal
        self._P = e
        self._D = self._derivative(e, now)
        self._out = self._Kp * self._P + self._Kd * self._D

        self._e_prev = e
        self._t_prev = now
        return self._limited(self._out)

    def _derivative(self, e, now):
        if self._e_prev is None or self._t_prev is None:
            return 0.0

        dt = now - self._t_prev
        if dt <= 0.0:
            return 0.0

        return (e - self._e_prev) / dt

    def _limited(self, value):
        if value == 0.0:
            return 0.0

        value = math.copysign(max(abs(value), self._min_out), value)
        return clamp(value, -self._max_out, self._max_out)


class TeddyApproachNode(Node):
    def __init__(self):
        super().__init__("teddy_approach")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.enabled = self.param("enabled")
        self.lost_timeout_s = self.param("lost_timeout_s")
        self.linear_speed = self.param("linear_speed")
        self.drive_when_not_centered = self.param("drive_when_not_centered")
        self.center_tolerance = self.param("center_tolerance")
        self.center_settle_s = self.param("center_settle_s")
        self.stop_lidar_distance_m = self.param("stop_lidar_distance_m")
        self.stop_lidar_front_angle_rad = self.param("stop_lidar_front_angle_rad")
        self.stop_lidar_min_points = self.param("stop_lidar_min_points")
        self.stop_lidar_timeout_s = self.param("stop_lidar_timeout_s")

        self.validate_params()

        self.turn_pid = BasicPd()
        self.turn_pid.setGains(self.param("angular_kp"), self.param("angular_kd"))
        self.turn_pid.setLimits(self.param("min_angular_speed"), self.param("max_angular_speed"))

        status_topic = self.param("status_topic")
        cmd_vel_topic = self.param("cmd_vel_topic")
        scan_topic = self.param("scan_topic")
        mode_topic = self.param("mode_topic")
        reset_topic = self.param("reset_topic")

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.mode_pub = self.create_publisher(String, mode_topic, 10)
        self.create_subscription(String, status_topic, self.on_status, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.create_subscription(Empty, reset_topic, self.on_reset, 10)
        self.create_timer(self.param("publish_period_s"), self.on_timer)

        self.last_seen_at = -1.0
        self.last_dx = 0.0
        self.centered_since = -1.0
        self.front_distance = math.inf
        self.front_points = 0
        self.last_scan_at = -1.0
        self.last_mode = ""
        # Once we publish teddy_approach_settled, stop driving so teddy_grab owns
        # the base. Stays latched until the node is restarted.
        self.handed_off = False

        self.get_logger().info(
            "teddy approach enabled=%s status=%s cmd=%s scan=%s"
            % (self.enabled, status_topic, cmd_vel_topic, scan_topic)
        )

    def param(self, name):
        return self.get_parameter(name).value

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def validate_params(self):
        min_speed = self.param("min_angular_speed")
        max_speed = self.param("max_angular_speed")

        if min_speed < 0.0:
            raise ValueError("min_angular_speed must be zero or greater")
        if max_speed < min_speed:
            raise ValueError("max_angular_speed must be greater than or equal to min_angular_speed")
        if self.param("angular_kd") < 0.0:
            raise ValueError("angular_kd must be zero or greater")
        if self.stop_lidar_distance_m < 0.0:
            raise ValueError("stop_lidar_distance_m must be zero or greater")
        if self.stop_lidar_front_angle_rad < 0.0:
            raise ValueError("stop_lidar_front_angle_rad must be zero or greater")
        if self.stop_lidar_min_points < 1:
            raise ValueError("stop_lidar_min_points must be at least 1")
        if self.stop_lidar_timeout_s < 0.0:
            raise ValueError("stop_lidar_timeout_s must be zero or greater")
        if self.center_settle_s < 0.0:
            raise ValueError("center_settle_s must be zero or greater")

    def on_status(self, msg):
        fields = parse_status(msg.data)

        if int(fields.get("teddy_count", "0")) <= 0:
            return
        if "dx" not in fields:
            return

        self.last_dx = float(fields["dx"])
        self.last_seen_at = self.now_s()

    def on_scan(self, msg):
        self.front_distance = math.inf
        self.front_points = 0

        angle = msg.angle_min
        for distance in msg.ranges:
            in_front = abs(angle) <= self.stop_lidar_front_angle_rad
            valid = math.isfinite(distance) and msg.range_min <= distance <= msg.range_max

            if in_front and valid:
                self.front_points += 1
                self.front_distance = min(self.front_distance, float(distance))

            angle += msg.angle_increment

        self.last_scan_at = self.now_s()

    def on_reset(self, _msg):
        self.turn_pid.reset()
        self.centered_since = -1.0
        self.handed_off = False
        self.last_mode = ""
        self.publish_stop()
        self.get_logger().info("reset by teddy_grab")

    def on_timer(self):
        if not self.enabled:
            return

        if self.handed_off:
            self.publish_stop()
            self.publish_mode("teddy_approach_settled", log_on_change=False)
            return

        now = self.now_s()
        if not self.teddy_recent(now):
            self.turn_pid.reset()
            self.centered_since = -1.0
            self.log_mode("waiting_for_teddy")
            return

        cmd = Twist()
        centered = abs(self.last_dx) <= self.center_tolerance
        if centered:
            if self.centered_since < 0.0:
                self.centered_since = now
        else:
            self.centered_since = -1.0
        centered_settled = (
            self.centered_since >= 0.0
            and (now - self.centered_since) >= self.center_settle_s
        )

        centered_close = centered and self.lidar_stop_active(now)
        if centered_close:
            self.turn_pid.reset()
            self.publish_stop()
            if centered_settled:
                self.publish_mode("teddy_approach_settled")
                self.handed_off = True
            else:
                self.log_mode("waiting_center_settle")
            return

        if centered:
            self.turn_pid.reset()
        else:
            cmd.angular.z = self.turn_pid.get(0.0, self.last_dx, now)

        if centered or self.drive_when_not_centered:
            cmd.linear.x = self.linear_speed

        self.cmd_pub.publish(cmd)
        self.log_mode("approaching" if centered else "centering")

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def teddy_recent(self, now):
        return self.last_seen_at >= 0.0 and (now - self.last_seen_at) <= self.lost_timeout_s

    def lidar_stop_active(self, now):
        lidar_fresh = self.last_scan_at >= 0.0 and (now - self.last_scan_at) <= self.stop_lidar_timeout_s
        return (
            self.stop_lidar_distance_m > 0.0
            and lidar_fresh
            and self.front_points >= self.stop_lidar_min_points
            and self.front_distance <= self.stop_lidar_distance_m
        )

    def log_mode(self, mode):
        self.publish_mode(mode)

    def publish_mode(self, mode, log_on_change=True):
        should_log = log_on_change and mode != self.last_mode
        if not should_log and mode == self.last_mode and log_on_change:
            return

        if mode == self.last_mode:
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
            return
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        if should_log:
            self.get_logger().info(f"mode={mode}")
        self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = TeddyApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
