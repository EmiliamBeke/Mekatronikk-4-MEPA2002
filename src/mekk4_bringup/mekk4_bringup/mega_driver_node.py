#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster


IGNORED_SERIAL_PREFIXES = ("MEGA_KEYBOARD_READY", "EVENT ")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MegaDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("mega_driver")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("post_open_wait_s", 2.5)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("send_period_s", 0.2)
        self.declare_parameter("odom_poll_period_s", 0.05)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("reply_timeout_s", 0.4)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("swap_sides", False)
        self.declare_parameter("max_track_speed_mps", 0.45)
        self.declare_parameter("max_pwm", 255)
        self.declare_parameter("min_nonzero_pwm", 55)
        self.declare_parameter("pure_rotation_linear_deadband_mps", 0.03)
        self.declare_parameter("left_cmd_sign", 1)
        self.declare_parameter("right_cmd_sign", 1)
        self.declare_parameter("angular_cmd_sign", 1)
        self.declare_parameter("left_cmd_scale", 1.0)
        self.declare_parameter("right_cmd_scale", 1.0)
        self.declare_parameter("left_tick_sign", 1)
        self.declare_parameter("right_tick_sign", 1)
        self.declare_parameter("left_m_per_tick", 0.0)
        self.declare_parameter("right_m_per_tick", 0.0)
        self.declare_parameter("track_width_eff_m", 0.35)
        self.declare_parameter("reset_encoders_on_connect", True)

        self._port = self.get_parameter("port").get_parameter_value().string_value
        self._baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._post_open_wait_s = (
            self.get_parameter("post_open_wait_s").get_parameter_value().double_value
        )
        self._reconnect_delay_s = (
            self.get_parameter("reconnect_delay_s").get_parameter_value().double_value
        )
        self._send_period_s = self.get_parameter("send_period_s").get_parameter_value().double_value
        self._odom_poll_period_s = (
            self.get_parameter("odom_poll_period_s").get_parameter_value().double_value
        )
        self._cmd_vel_timeout_s = (
            self.get_parameter("cmd_vel_timeout_s").get_parameter_value().double_value
        )
        self._reply_timeout_s = self.get_parameter("reply_timeout_s").get_parameter_value().double_value
        self._base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self._odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self._publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self._swap_sides = self.get_parameter("swap_sides").get_parameter_value().bool_value
        self._max_track_speed_mps = (
            self.get_parameter("max_track_speed_mps").get_parameter_value().double_value
        )
        self._max_pwm = self.get_parameter("max_pwm").get_parameter_value().integer_value
        self._min_nonzero_pwm = (
            self.get_parameter("min_nonzero_pwm").get_parameter_value().integer_value
        )
        self._pure_rotation_linear_deadband_mps = (
            self.get_parameter("pure_rotation_linear_deadband_mps")
            .get_parameter_value()
            .double_value
        )
        self._left_cmd_sign = self.get_parameter("left_cmd_sign").get_parameter_value().integer_value
        self._right_cmd_sign = self.get_parameter("right_cmd_sign").get_parameter_value().integer_value
        self._angular_cmd_sign = (
            self.get_parameter("angular_cmd_sign").get_parameter_value().integer_value
        )
        self._left_cmd_scale = (
            self.get_parameter("left_cmd_scale").get_parameter_value().double_value
        )
        self._right_cmd_scale = (
            self.get_parameter("right_cmd_scale").get_parameter_value().double_value
        )
        self._left_tick_sign = self.get_parameter("left_tick_sign").get_parameter_value().integer_value
        self._right_tick_sign = self.get_parameter("right_tick_sign").get_parameter_value().integer_value
        self._left_m_per_tick = (
            self.get_parameter("left_m_per_tick").get_parameter_value().double_value
        )
        self._right_m_per_tick = (
            self.get_parameter("right_m_per_tick").get_parameter_value().double_value
        )
        self._track_width_eff_m = (
            self.get_parameter("track_width_eff_m").get_parameter_value().double_value
        )
        self._reset_encoders_on_connect = (
            self.get_parameter("reset_encoders_on_connect").get_parameter_value().bool_value
        )

        if self._max_track_speed_mps <= 0.0:
            raise ValueError("max_track_speed_mps must be greater than zero.")
        if self._left_cmd_scale <= 0.0 or self._right_cmd_scale <= 0.0:
            raise ValueError("left_cmd_scale and right_cmd_scale must be greater than zero.")
        if (
            self._left_cmd_sign not in (-1, 1)
            or self._right_cmd_sign not in (-1, 1)
            or self._angular_cmd_sign not in (-1, 1)
        ):
            raise ValueError("left_cmd_sign, right_cmd_sign, and angular_cmd_sign must be -1 or 1.")
        if self._track_width_eff_m <= 0.0:
            raise ValueError("track_width_eff_m must be greater than zero.")
        if self._pure_rotation_linear_deadband_mps < 0.0:
            raise ValueError("pure_rotation_linear_deadband_mps must be zero or greater.")
        if self._send_period_s <= 0.0 or self._odom_poll_period_s <= 0.0:
            raise ValueError("Timer periods must be greater than zero.")

        self._serial = None
        self._serial_module = None
        self._next_connect_attempt = 0.0
        self._last_motion_command = "STOP"
        self._last_motion_sent_at = 0.0
        self._last_stop_sent = False
        self._last_poll_at = 0.0

        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._last_cmd_vel_at = -1.0

        self._last_left_ticks = None
        self._last_right_ticks = None
        self._last_encoder_stamp = None

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._odom_enabled = self._left_m_per_tick > 0.0 and self._right_m_per_tick > 0.0
        if not self._odom_enabled:
            self.get_logger().warning(
                "Mega driver started without calibrated meters-per-tick; /odom will stay disabled "
                "until left_m_per_tick and right_m_per_tick are set."
            )

        self._load_serial()

        self._cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._left_pwm_pub = self.create_publisher(Int32, "mega_driver/left_pwm", 10)
        self._right_pwm_pub = self.create_publisher(Int32, "mega_driver/right_pwm", 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._timer = self.create_timer(0.02, self._on_timer)

    def _load_serial(self) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "Missing pyserial in runtime environment. Install pyserial where the node runs."
            ) from exc

        self._serial_module = serial

    def _close_serial(self) -> None:
        if self._serial is None:
            return

        try:
            self._serial.write(b"STOP\n")
            self._serial.flush()
        except Exception:
            pass

        try:
            self._serial.close()
        except Exception:
            pass

        self._serial = None
        self._last_motion_command = "STOP"
        self._last_stop_sent = False
        self._last_left_ticks = None
        self._last_right_ticks = None
        self._last_encoder_stamp = None
        self._next_connect_attempt = time.monotonic() + self._reconnect_delay_s

    def _try_connect(self) -> bool:
        if self._serial is not None:
            return True

        now = time.monotonic()
        if now < self._next_connect_attempt:
            return False

        try:
            self._serial = self._serial_module.Serial(
                self._port,
                self._baudrate,
                timeout=0.1,
                write_timeout=1.0,
            )
            time.sleep(max(0.0, self._post_open_wait_s))
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            firmware = self._send_expect("ID", "MEGA_")
            if firmware != "MEGA_KEYBOARD_DRIVE":
                raise RuntimeError(
                    f"mega driver expects mega_keyboard_drive firmware, got {firmware!r}"
                )
            self._send_expect("PING", "PONG")
            self._send_expect("STOP", "OK STOP")
            if self._reset_encoders_on_connect:
                self._send_expect("RESET ENC1", "OK RESET ENC1")
                self._send_expect("RESET ENC2", "OK RESET ENC2")
            left_ticks_raw, right_ticks_raw = self._read_encoder_pair()
            self._last_left_ticks = left_ticks_raw * self._left_tick_sign
            self._last_right_ticks = right_ticks_raw * self._right_tick_sign
            self._last_encoder_stamp = time.monotonic()
            self._last_motion_command = "STOP"
            self._last_stop_sent = True
            self._last_motion_sent_at = time.monotonic()
            self._last_poll_at = 0.0
            self.get_logger().info(f"Connected to Mega on {self._port} @ {self._baudrate}")
            return True
        except Exception as exc:
            self.get_logger().warning(f"Failed to connect to Mega on {self._port}: {exc}")
            self._close_serial()
            return False

    def _read_reply(self, timeout_s: float) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            if any(text.startswith(prefix) for prefix in IGNORED_SERIAL_PREFIXES):
                self.get_logger().debug(f"Mega event: {text}")
                continue
            return text
        raise RuntimeError("timeout waiting for Mega reply")

    def _send_expect(self, command: str, expected_prefix: str) -> str:
        self._serial.write((command + "\n").encode("utf-8"))
        self._serial.flush()
        reply = self._read_reply(self._reply_timeout_s)
        if not reply.startswith(expected_prefix):
            raise RuntimeError(
                f"unexpected reply to {command!r}: expected prefix {expected_prefix!r}, got {reply!r}"
            )
        return reply

    def _send_motion(self, command: str) -> None:
        self._serial.write((command + "\n").encode("utf-8"))
        self._serial.flush()
        self._last_motion_command = command
        self._last_motion_sent_at = time.monotonic()
        self._last_stop_sent = command == "STOP"

    def _read_encoder_pair(self) -> tuple[int, int]:
        left_reply = self._send_expect("ENC1", "ENC1 ")
        right_reply = self._send_expect("ENC2", "ENC2 ")
        first = self._parse_encoder(left_reply, "ENC1")
        second = self._parse_encoder(right_reply, "ENC2")
        if self._swap_sides:
            return second, first
        return first, second

    @staticmethod
    def _parse_encoder(reply: str, label: str) -> int:
        parts = reply.split()
        if len(parts) != 2 or parts[0] != label:
            raise RuntimeError(f"failed to parse encoder reply {reply!r}")
        return int(parts[1])

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._desired_linear = float(msg.linear.x)
        self._desired_angular = float(msg.angular.z)
        self._last_cmd_vel_at = time.monotonic()

    def _desired_motion_command(self) -> str:
        now = time.monotonic()
        if self._last_cmd_vel_at < 0.0 or (now - self._last_cmd_vel_at) > self._cmd_vel_timeout_s:
            self._publish_pwm(0, 0)
            return "STOP"

        half_width = self._track_width_eff_m / 2.0
        linear = self._desired_linear
        if (
            abs(linear) <= self._pure_rotation_linear_deadband_mps
            and abs(self._desired_angular) > 1e-6
        ):
            linear = 0.0

        angular_sign = self._angular_cmd_sign if abs(linear) < 1e-6 else 1
        angular = self._desired_angular * angular_sign
        left_speed = linear - (angular * half_width)
        right_speed = linear + (angular * half_width)
        left_speed *= self._left_cmd_scale
        right_speed *= self._right_cmd_scale

        left_pwm = self._speed_to_pwm(left_speed, self._left_cmd_sign)
        right_pwm = self._speed_to_pwm(right_speed, self._right_cmd_sign)
        if left_pwm == 0 and right_pwm == 0:
            self._publish_pwm(0, 0)
            return "STOP"
        if self._swap_sides:
            left_pwm, right_pwm = right_pwm, left_pwm
        self._publish_pwm(left_pwm, right_pwm)
        return f"BOTH {left_pwm} {right_pwm}"

    def _publish_pwm(self, left_pwm: int, right_pwm: int) -> None:
        left_msg = Int32()
        left_msg.data = int(left_pwm)
        right_msg = Int32()
        right_msg.data = int(right_pwm)
        self._left_pwm_pub.publish(left_msg)
        self._right_pwm_pub.publish(right_msg)

    def _speed_to_pwm(self, track_speed_mps: float, sign: int) -> int:
        normalized = max(-1.0, min(1.0, track_speed_mps / self._max_track_speed_mps))
        pwm = int(round(normalized * self._max_pwm))
        if pwm > 0:
            pwm = max(self._min_nonzero_pwm, pwm)
        elif pwm < 0:
            pwm = min(-self._min_nonzero_pwm, pwm)
        return max(-self._max_pwm, min(self._max_pwm, pwm * sign))

    def _maybe_send_motion(self, command: str) -> None:
        now = time.monotonic()
        if command == "STOP":
            if self._last_stop_sent:
                return
            self._send_expect("STOP", "OK STOP")
            self._last_motion_command = "STOP"
            self._last_motion_sent_at = now
            self._last_stop_sent = True
            return

        if command != self._last_motion_command or (now - self._last_motion_sent_at) >= self._send_period_s:
            self._send_motion(command)

    def _poll_odometry(self) -> None:
        if not self._odom_enabled:
            return

        now = time.monotonic()
        if self._last_poll_at and (now - self._last_poll_at) < self._odom_poll_period_s:
            return

        left_ticks_raw, right_ticks_raw = self._read_encoder_pair()
        left_ticks = left_ticks_raw * self._left_tick_sign
        right_ticks = right_ticks_raw * self._right_tick_sign
        stamp_now = time.monotonic()

        if self._last_left_ticks is None or self._last_right_ticks is None or self._last_encoder_stamp is None:
            self._last_left_ticks = left_ticks
            self._last_right_ticks = right_ticks
            self._last_encoder_stamp = stamp_now
            self._last_poll_at = now
            return

        dt = max(1e-6, stamp_now - self._last_encoder_stamp)
        delta_left_ticks = left_ticks - self._last_left_ticks
        delta_right_ticks = right_ticks - self._last_right_ticks
        self._last_left_ticks = left_ticks
        self._last_right_ticks = right_ticks
        self._last_encoder_stamp = stamp_now
        self._last_poll_at = now

        d_left = delta_left_ticks * self._left_m_per_tick
        d_right = delta_right_ticks * self._right_m_per_tick
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self._track_width_eff_m

        theta_mid = self._yaw + 0.5 * d_theta
        self._x += d_center * math.cos(theta_mid)
        self._y += d_center * math.sin(theta_mid)
        self._yaw = normalize_angle(self._yaw + d_theta)

        linear_velocity = d_center / dt
        angular_velocity = d_theta / dt
        self._publish_odometry(linear_velocity, angular_velocity)

    def _publish_odometry(self, linear_velocity: float, angular_velocity: float) -> None:
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.08
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.12

        self._odom_pub.publish(odom)

        if self._tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame_id
        transform.child_frame_id = self._base_frame_id
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    def _on_timer(self) -> None:
        if not self._try_connect():
            return

        try:
            self._maybe_send_motion(self._desired_motion_command())
            self._poll_odometry()
        except Exception as exc:
            self.get_logger().warning(f"Mega driver loop failed: {exc}")
            self._close_serial()

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MegaDriverNode()
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
