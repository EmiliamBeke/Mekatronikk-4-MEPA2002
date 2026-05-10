#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


IGNORED_SERIAL_PREFIXES = (
    "MEGA_KEYBOARD_READY",
    "EVENT ",
    "OK HOME ",
)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MegaDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("mega_driver")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("post_open_wait_s", 2.5)
        self.declare_parameter("startup_ready_timeout_s", 120.0)
        self.declare_parameter("reconnect_delay_s", 1.0)
        self.declare_parameter("send_period_s", 0.2)
        self.declare_parameter("odom_poll_period_s", 0.05)
        self.declare_parameter("arm_state_poll_period_s", 0.25)
        self.declare_parameter("arm_motion_timeout_s", 180.0)
        self.declare_parameter("init_distance_sensor_on_connect", True)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("reply_timeout_s", 2.0)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("swap_sides", False)
        self.declare_parameter("max_track_speed_mps", 0.45)
        self.declare_parameter("max_pwm", 255)
        self.declare_parameter("min_nonzero_pwm", 55)
        self.declare_parameter("min_forward_pwm", 0)
        self.declare_parameter("min_reverse_pwm", 0)
        self.declare_parameter("min_turn_pwm", 0)
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
        self.declare_parameter("initial_arm_x_m", 0.0)
        self.declare_parameter("initial_arm_z_m", 0.12)
        self.declare_parameter("arm_x_steps_per_mm", 18.65)
        self.declare_parameter("arm_z_steps_per_mm", 2929.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("x_joint_name", "robotarm_x_joint")
        self.declare_parameter("z_joint_name", "robotarm_z_joint")
        self.declare_parameter("gripper_min_us", 500)
        self.declare_parameter("gripper_max_us", 1800)

        self._port = self.get_parameter("port").get_parameter_value().string_value
        self._baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._post_open_wait_s = (
            self.get_parameter("post_open_wait_s").get_parameter_value().double_value
        )
        self._startup_ready_timeout_s = (
            self.get_parameter("startup_ready_timeout_s").get_parameter_value().double_value
        )
        self._reconnect_delay_s = (
            self.get_parameter("reconnect_delay_s").get_parameter_value().double_value
        )
        self._send_period_s = self.get_parameter("send_period_s").get_parameter_value().double_value
        self._odom_poll_period_s = (
            self.get_parameter("odom_poll_period_s").get_parameter_value().double_value
        )
        self._arm_state_poll_period_s = (
            self.get_parameter("arm_state_poll_period_s").get_parameter_value().double_value
        )
        self._arm_motion_timeout_s = (
            self.get_parameter("arm_motion_timeout_s").get_parameter_value().double_value
        )
        self._init_distance_sensor_on_connect = (
            self.get_parameter("init_distance_sensor_on_connect").get_parameter_value().bool_value
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
        self._min_forward_pwm = (
            self.get_parameter("min_forward_pwm").get_parameter_value().integer_value
        )
        self._min_reverse_pwm = (
            self.get_parameter("min_reverse_pwm").get_parameter_value().integer_value
        )
        self._min_turn_pwm = self.get_parameter("min_turn_pwm").get_parameter_value().integer_value
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
        self._initial_arm_x_m = (
            self.get_parameter("initial_arm_x_m").get_parameter_value().double_value
        )
        self._initial_arm_z_m = (
            self.get_parameter("initial_arm_z_m").get_parameter_value().double_value
        )
        self._arm_x_steps_per_mm = (
            self.get_parameter("arm_x_steps_per_mm").get_parameter_value().double_value
        )
        self._arm_z_steps_per_mm = (
            self.get_parameter("arm_z_steps_per_mm").get_parameter_value().double_value
        )
        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._x_joint_name = self.get_parameter("x_joint_name").get_parameter_value().string_value
        self._z_joint_name = self.get_parameter("z_joint_name").get_parameter_value().string_value
        self._gripper_min_us = (
            self.get_parameter("gripper_min_us").get_parameter_value().integer_value
        )
        self._gripper_max_us = (
            self.get_parameter("gripper_max_us").get_parameter_value().integer_value
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
        if (
            self._min_nonzero_pwm < 0
            or self._min_forward_pwm < 0
            or self._min_reverse_pwm < 0
            or self._min_turn_pwm < 0
        ):
            raise ValueError("minimum PWM values must be zero or greater.")
        if self._pure_rotation_linear_deadband_mps < 0.0:
            raise ValueError("pure_rotation_linear_deadband_mps must be zero or greater.")
        if self._startup_ready_timeout_s <= 0.0:
            raise ValueError("startup_ready_timeout_s must be greater than zero.")
        if (
            self._send_period_s <= 0.0
            or self._odom_poll_period_s <= 0.0
            or self._arm_state_poll_period_s <= 0.0
        ):
            raise ValueError("Timer periods must be greater than zero.")
        if self._arm_motion_timeout_s <= 0.0:
            raise ValueError("arm_motion_timeout_s must be greater than zero.")
        if self._arm_x_steps_per_mm <= 0.0:
            raise ValueError("arm_x_steps_per_mm must be greater than zero.")
        if self._arm_z_steps_per_mm <= 0.0:
            raise ValueError("arm_z_steps_per_mm must be greater than zero.")
        if self._gripper_min_us <= 0 or self._gripper_min_us >= self._gripper_max_us:
            raise ValueError("gripper_min_us must be greater than zero and below gripper_max_us.")

        self._serial = None
        self._serial_module = None
        self._serial_error_count = 0
        self._next_connect_attempt = 0.0
        self._last_motion_command = "STOP"
        self._last_motion_sent_at = 0.0
        self._last_stop_sent = False
        self._last_poll_at = 0.0
        self._last_arm_state_poll_at = 0.0

        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._last_cmd_vel_at = -1.0

        self._desired_arm_x = self._initial_arm_x_m
        self._last_arm_x_cmd_m = self._initial_arm_x_m
        self._pending_arm_x_delta_steps = 0
        self._desired_arm_z = self._initial_arm_z_m
        self._last_arm_z_cmd_m = self._initial_arm_z_m
        self._pending_arm_z_delta_steps = 0
        self._actual_arm_x = self._initial_arm_x_m
        self._actual_arm_z = self._initial_arm_z_m
        self._arm_homed = False
        self._warned_arm_not_homed = False
        self._active_arm_axis: str | None = None
        self._desired_left_gripper = float(self._gripper_min_us)
        self._desired_right_gripper = float(self._gripper_min_us)
        self._last_gripper_us_sent = None

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
        self._arm_x_sub = self.create_subscription(
            Float64, "/robotarm/x_position_cmd", self._on_arm_x_cmd, 10
        )
        self._arm_z_sub = self.create_subscription(
            Float64, "/robotarm/z_position_cmd", self._on_arm_z_cmd, 10
        )
        self._left_gripper_sub = self.create_subscription(
            Float64, "/gripper/left_position_cmd", self._on_left_gripper_cmd, 10
        )
        self._right_gripper_sub = self.create_subscription(
            Float64, "/gripper/right_position_cmd", self._on_right_gripper_cmd, 10
        )
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._left_pwm_pub = self.create_publisher(Int32, "mega_driver/left_pwm", 10)
        self._right_pwm_pub = self.create_publisher(Int32, "mega_driver/right_pwm", 10)
        self._arm_x_state_pub = self.create_publisher(Float64, "/robotarm/x_position_state", 10)
        self._arm_z_state_pub = self.create_publisher(Float64, "/robotarm/z_position_state", 10)
        self._joint_state_pub = self.create_publisher(JointState, self._joint_states_topic, 10)
        self._distance_pub = self.create_publisher(Int32, "/mega/distance_mm", 10)
        self._home_arm_srv = self.create_service(Trigger, "/mega/home_arm", self._on_home_arm)
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
        self._active_arm_axis = None
        self._serial_error_count = 0
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
            self._wait_for_ready(self._startup_ready_timeout_s)
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
            self._try_init_distance_sensor()
            left_ticks_raw, right_ticks_raw = self._read_encoder_pair()
            self._last_left_ticks = left_ticks_raw * self._left_tick_sign
            self._last_right_ticks = right_ticks_raw * self._right_tick_sign
            self._last_encoder_stamp = time.monotonic()
            self._last_motion_command = "STOP"
            self._last_stop_sent = True
            self._last_motion_sent_at = time.monotonic()
            self._last_poll_at = 0.0
            self._last_arm_state_poll_at = 0.0
            self._serial_error_count = 0
            self._sync_arm_state_from_mega()
            if self._arm_homed:
                self._desired_arm_x = self._actual_arm_x
                self._last_arm_x_cmd_m = self._actual_arm_x
                self._desired_arm_z = self._actual_arm_z
                self._last_arm_z_cmd_m = self._actual_arm_z
                self._publish_arm_state()
            else:
                self.get_logger().warning("Mega arm is not homed. Call /mega/home_arm before arm motion.")
            self.get_logger().info(f"Connected to Mega on {self._port} @ {self._baudrate}")
            return True
        except Exception as exc:
            self.get_logger().warning(f"Failed to connect to Mega on {self._port}: {exc}")
            self._close_serial()
            return False

    def _wait_for_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        next_probe_at = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if raw:
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                if text in ("MEGA_KEYBOARD_READY", "MEGA_KEYBOARD_DRIVE"):
                    return
                if text.startswith("EVENT LIMIT "):
                    self.get_logger().info(f"Mega {text}")
                    continue
                self.get_logger().debug(f"Mega startup: {text}")

            if time.monotonic() < next_probe_at:
                continue

            next_probe_at = time.monotonic() + 2.0
            try:
                self._serial.write(b"ID\n")
                self._serial.flush()
                reply_raw = self._serial.readline()
                if not reply_raw:
                    continue
                reply = reply_raw.decode("utf-8", errors="replace").strip()
                if not reply:
                    continue
                if reply == "MEGA_KEYBOARD_DRIVE":
                    return
                if reply.startswith("EVENT LIMIT "):
                    self.get_logger().info(f"Mega {reply}")
                else:
                    self.get_logger().debug(f"Mega probe: {reply}")
            except Exception:
                continue
        raise RuntimeError("timeout waiting for Mega startup ready")

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
                if text.startswith("EVENT LIMIT "):
                    self.get_logger().info(f"Mega {text}")
                else:
                    self.get_logger().debug(f"Mega event: {text}")
                continue
            return text
        raise RuntimeError("timeout waiting for Mega reply")

    def _send_expect(self, command: str, expected_prefix: str, timeout_s: float | None = None) -> str:
        self._serial.write((command + "\n").encode("utf-8"))
        self._serial.flush()
        reply = self._read_reply(self._reply_timeout_s if timeout_s is None else timeout_s)
        if not reply.startswith(expected_prefix):
            raise RuntimeError(
                f"unexpected reply to {command!r}: expected prefix {expected_prefix!r}, got {reply!r}"
            )
        self._serial_error_count = 0
        return reply

    def _send_motion(self, command: str) -> None:
        self._serial.write((command + "\n").encode("utf-8"))
        self._serial.flush()
        self._last_motion_command = command
        self._last_motion_sent_at = time.monotonic()
        self._last_stop_sent = command == "STOP"

    def _try_init_distance_sensor(self) -> None:
        if not self._init_distance_sensor_on_connect:
            return
        try:
            self._send_expect("DIST INIT", "OK DIST INIT")
        except Exception as exc:
            self.get_logger().warning(f"Mega distance sensor init failed: {exc}")

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

    def _on_arm_x_cmd(self, msg: Float64) -> None:
        x_m = float(msg.data)
        if self._last_arm_x_cmd_m is None:
            self._last_arm_x_cmd_m = x_m
            self._desired_arm_x = x_m
            self._pending_arm_x_delta_steps = 0
            return

        delta_m = x_m - self._last_arm_x_cmd_m
        self._last_arm_x_cmd_m = x_m
        self._desired_arm_x = x_m
        delta_steps = self._meters_to_x_steps(delta_m)
        if delta_steps != 0:
            self._pending_arm_x_delta_steps += delta_steps

    def _on_arm_z_cmd(self, msg: Float64) -> None:
        z_m = float(msg.data)
        if self._last_arm_z_cmd_m is None:
            self._last_arm_z_cmd_m = z_m
            self._desired_arm_z = z_m
            self._pending_arm_z_delta_steps = 0
            return

        delta_m = z_m - self._last_arm_z_cmd_m
        self._last_arm_z_cmd_m = z_m
        self._desired_arm_z = z_m
        delta_steps = self._meters_to_z_steps(delta_m)
        if delta_steps != 0:
            self._pending_arm_z_delta_steps += delta_steps

    def _on_left_gripper_cmd(self, msg: Float64) -> None:
        self._desired_left_gripper = float(msg.data)

    def _on_right_gripper_cmd(self, msg: Float64) -> None:
        self._desired_right_gripper = float(msg.data)

    def _gripper_us_from_command(self, value: float) -> int:
        pulse_us = int(round(value))
        return max(self._gripper_min_us, min(self._gripper_max_us, pulse_us))

    def _meters_to_z_steps(self, meters: float) -> int:
        millimeters = meters * 1000.0
        return int(round(millimeters * self._arm_z_steps_per_mm))

    def _meters_to_x_steps(self, meters: float) -> int:
        millimeters = meters * 1000.0
        return int(round(millimeters * self._arm_x_steps_per_mm))

    def _maybe_send_arm_x(self) -> None:
        if self._pending_arm_x_delta_steps == 0:
            return
        if not self._arm_homed:
            if not self._warned_arm_not_homed:
                self.get_logger().warning("Ignoring ARM X command until /mega/home_arm succeeds.")
                self._warned_arm_not_homed = True
            self._pending_arm_x_delta_steps = 0
            return
        if self._active_arm_axis not in (None, "x"):
            return

        delta_steps = self._pending_arm_x_delta_steps
        self._send_expect(f"ARM X {delta_steps}", "OK ARM X", self._arm_motion_timeout_s)
        self._pending_arm_x_delta_steps = 0
        self._active_arm_axis = None
        self._last_arm_state_poll_at = 0.0
        self._sync_arm_state_from_mega()

    def _maybe_send_arm_z(self) -> None:
        if self._pending_arm_z_delta_steps == 0:
            return
        if not self._arm_homed:
            if not self._warned_arm_not_homed:
                self.get_logger().warning("Ignoring ARM Z command until /mega/home_arm succeeds.")
                self._warned_arm_not_homed = True
            self._pending_arm_z_delta_steps = 0
            return
        if self._active_arm_axis not in (None, "z"):
            return

        delta_steps = self._pending_arm_z_delta_steps
        self._send_expect(f"ARM Z {delta_steps}", "OK ARM Z", self._arm_motion_timeout_s)
        self._pending_arm_z_delta_steps = 0
        self._active_arm_axis = None
        self._last_arm_state_poll_at = 0.0
        self._sync_arm_state_from_mega()

    def _maybe_send_gripper(self) -> None:
        gripper_us = self._gripper_us_from_command(self._desired_left_gripper)
        if gripper_us == self._last_gripper_us_sent:
            return

        self._send_expect(f"SERVO {gripper_us}", "OK SERVO")
        self._last_gripper_us_sent = gripper_us

    def _on_home_arm(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._try_connect():
            response.success = False
            response.message = "Mega is not connected."
            return response

        try:
            self._send_expect("HOME ARM", "OK ARM STARTUP HOME", self._startup_ready_timeout_s)
            self._sync_arm_state_from_mega()
            if not self._arm_homed:
                raise RuntimeError("Mega reported arm still not homed after HOME ARM.")
            self._desired_arm_x = self._actual_arm_x
            self._last_arm_x_cmd_m = self._actual_arm_x
            self._pending_arm_x_delta_steps = 0
            self._desired_arm_z = self._actual_arm_z
            self._last_arm_z_cmd_m = self._actual_arm_z
            self._pending_arm_z_delta_steps = 0
            self._warned_arm_not_homed = False
            self._active_arm_axis = None
            self._publish_arm_state()
        except Exception as exc:
            response.success = False
            response.message = f"HOME ARM failed: {exc}"
            self._close_serial()
            return response

        response.success = True
        response.message = "Mega arm homed."
        return response

    def _publish_arm_state(self) -> None:
        x_msg = Float64()
        x_msg.data = float(self._actual_arm_x)
        z_msg = Float64()
        z_msg.data = float(self._actual_arm_z)
        self._arm_x_state_pub.publish(x_msg)
        self._arm_z_state_pub.publish(z_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [self._x_joint_name, self._z_joint_name]
        joint_msg.position = [float(self._actual_arm_x), float(self._actual_arm_z)]
        self._joint_state_pub.publish(joint_msg)

    def _publish_distance_state(self, distance_mm: int) -> None:
        msg = Int32()
        msg.data = int(distance_mm)
        self._distance_pub.publish(msg)

    def _sync_arm_state_from_mega(self) -> None:
        reply = self._send_expect("STATE", "STATE ")
        state = {}
        for token in reply.split()[1:]:
            if "=" not in token:
                continue
            name, value = token.split("=", 1)
            state[name] = value

        was_homed = self._arm_homed
        self._arm_homed = state.get("H") == "1"
        if "D" in state:
            self._publish_distance_state(int(state["D"]))
        if not self._arm_homed:
            self._active_arm_axis = None
            return

        if "X" in state:
            self._actual_arm_x = int(state["X"]) / self._arm_x_steps_per_mm / 1000.0
        if "Z" in state:
            self._actual_arm_z = int(state["Z"]) / self._arm_z_steps_per_mm / 1000.0
        if not was_homed:
            self._desired_arm_x = self._actual_arm_x
            self._last_arm_x_cmd_m = self._actual_arm_x
            self._pending_arm_x_delta_steps = 0
            self._desired_arm_z = self._actual_arm_z
            self._last_arm_z_cmd_m = self._actual_arm_z
            self._pending_arm_z_delta_steps = 0
            self._warned_arm_not_homed = False
            self._active_arm_axis = None
        self._refresh_active_arm_axis(state)
        self._publish_arm_state()

    def _arm_x_at_target(self) -> bool:
        tolerance_m = 1.5 / self._arm_x_steps_per_mm / 1000.0
        return abs(self._actual_arm_x - self._desired_arm_x) <= tolerance_m

    def _arm_z_at_target(self) -> bool:
        tolerance_m = 1.5 / self._arm_z_steps_per_mm / 1000.0
        return abs(self._actual_arm_z - self._desired_arm_z) <= tolerance_m

    def _refresh_active_arm_axis(self, state: dict[str, str]) -> None:
        if self._active_arm_axis == "x" and (
            state.get("XM") == "0" or ("XM" not in state and self._arm_x_at_target())
        ):
            self._active_arm_axis = None
        elif self._active_arm_axis == "z" and (
            state.get("ZM") == "0" or ("ZM" not in state and self._arm_z_at_target())
        ):
            self._active_arm_axis = None

    def _maybe_poll_arm_state(self) -> None:
        now = time.monotonic()
        if now - self._last_arm_state_poll_at < self._arm_state_poll_period_s:
            return
        self._last_arm_state_poll_at = now
        self._sync_arm_state_from_mega()

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

        min_pwm = self._minimum_pwm_for_motion(linear)
        left_pwm = self._speed_to_pwm(left_speed, self._left_cmd_sign, min_pwm)
        right_pwm = self._speed_to_pwm(right_speed, self._right_cmd_sign, min_pwm)
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

    def _minimum_pwm_for_motion(self, linear_mps: float) -> int:
        if abs(linear_mps) < 1e-6:
            return self._min_turn_pwm or self._min_nonzero_pwm
        if linear_mps > 0.0:
            return self._min_forward_pwm or self._min_nonzero_pwm
        return self._min_reverse_pwm or self._min_nonzero_pwm

    def _speed_to_pwm(self, track_speed_mps: float, sign: int, min_pwm: int) -> int:
        normalized = max(-1.0, min(1.0, track_speed_mps / self._max_track_speed_mps))
        pwm = int(round(normalized * self._max_pwm))
        if pwm > 0:
            pwm = max(min_pwm, pwm)
        elif pwm < 0:
            pwm = min(-min_pwm, pwm)
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
            self._maybe_send_arm_x()
            self._maybe_send_arm_z()
            self._maybe_poll_arm_state()
            self._maybe_send_gripper()
            self._poll_odometry()
        except Exception as exc:
            self.get_logger().warning(f"Mega driver loop failed: {exc}")
            self._serial_error_count += 1
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                self._close_serial()
                return
            if self._serial_error_count >= 3:
                self.get_logger().warning("Closing Mega serial after 3 consecutive driver failures.")
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
