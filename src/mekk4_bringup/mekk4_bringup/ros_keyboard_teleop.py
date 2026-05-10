#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


ARM_COMMAND_TOPICS = {
    "/robotarm/x_position_cmd",
    "/robotarm/z_position_cmd",
    "/gripper/left_position_cmd",
    "/gripper/right_position_cmd",
}


def reject_command_topics(**topics: str) -> None:
    for name, topic in topics.items():
        if topic in ARM_COMMAND_TOPICS:
            raise ValueError(
                f"{name} must publish to a request topic, not low-level command topic {topic}. "
                "Arm commands must pass through robotarm_safety_node."
            )


class RosKeyboardTeleop:
    def __init__(self, args: argparse.Namespace) -> None:
        reject_command_topics(
            arm_x_topic=args.arm_x_topic,
            arm_z_topic=args.arm_z_topic,
            gripper_topic=args.gripper_topic,
            right_gripper_topic=args.right_gripper_topic,
        )

        self.node = rclpy.create_node("ros_keyboard_teleop")
        self.pub = self.node.create_publisher(Twist, args.topic, 10)
        self.arm_x_pub = self.node.create_publisher(Float64, args.arm_x_topic, 10)
        self.arm_z_pub = self.node.create_publisher(Float64, args.arm_z_topic, 10)
        self.gripper_pub = self.node.create_publisher(Float64, args.gripper_topic, 10)
        self.right_gripper_pub = self.node.create_publisher(
            Float64, args.right_gripper_topic, 10
        )
        self.node.create_subscription(Float64, args.arm_x_state_topic, self._on_arm_x_state, 10)
        self.node.create_subscription(Float64, args.arm_z_state_topic, self._on_arm_z_state, 10)

        self.topic = args.topic
        self.arm_x_topic = args.arm_x_topic
        self.arm_z_topic = args.arm_z_topic
        self.gripper_topic = args.gripper_topic
        self.speed = max(0.0, args.speed)
        self.turn_speed = max(0.0, args.turn_speed)
        self.speed_step = max(0.01, args.speed_step)
        self.turn_speed_step = max(0.01, args.turn_speed_step)
        self.max_speed = max(self.speed, args.max_speed)
        self.max_turn_speed = max(self.turn_speed, args.max_turn_speed)
        self.send_period = max(0.01, args.send_period)
        self.arm_x = clamp(args.arm_x_initial, args.arm_x_min, args.arm_x_max)
        self.arm_z = clamp(args.arm_z_initial, args.arm_z_min, args.arm_z_max)
        self.arm_x_min = args.arm_x_min
        self.arm_x_max = args.arm_x_max
        self.arm_z_min = args.arm_z_min
        self.arm_z_max = args.arm_z_max
        self.arm_z_offset = args.arm_z_offset
        self.arm_x_speed = max(0.0, args.arm_x_speed)
        self.arm_z_speed = max(0.0, args.arm_z_speed)
        self.arm_x_speed_step = max(0.001, args.arm_x_speed_step)
        self.arm_z_speed_step = max(0.001, args.arm_z_speed_step)
        self.max_arm_x_speed = max(self.arm_x_speed, args.max_arm_x_speed)
        self.max_arm_z_speed = max(self.arm_z_speed, args.max_arm_z_speed)
        self.gripper = clamp(args.gripper_initial, args.gripper_min, args.gripper_max)
        self.gripper_min = args.gripper_min
        self.gripper_max = args.gripper_max
        self.gripper_step = max(1.0, args.gripper_step)

        self.pressed_keys: set[str] = set()
        self.release_jobs: dict[str, str] = {}
        self.last_command = (None, None)
        self.last_sent_at = 0.0
        self.last_tick_at = time.monotonic()
        self.closed = False
        self._suppress_slider_cb = False

        self.root = tk.Tk()
        self.root.title("ROS Keyboard Teleop")
        self.root.geometry("640x520")
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.arm_x_var = tk.DoubleVar(value=self.arm_x)
        self.arm_z_var = tk.DoubleVar(value=self.arm_z)
        self.gripper_var = tk.DoubleVar(value=self.gripper)

        self.status_var = tk.StringVar(value=f"Publishing to {self.topic}")
        self.command_var = tk.StringVar(value=self._command_text(0.0, 0.0))
        self.speed_var = tk.StringVar(value=self._speed_text())
        self.hint_var = tk.StringVar(
            value=(
                "Hold W/S/A/D drive. Y/H arm up/down. J/K arm out/in. "
                "E/Q drive speed. P/O turn speed. M/N x speed +/-. T/G z speed +/-. "
                "U/I gripper open/close. "
                "SPACE stop. - quit."
            )
        )

        self._build_ui()
        self._bind_keys()

        self.root.after(20, self._tick)
        self.root.after(20, self._spin_ros)

    def _build_ui(self) -> None:
        title = tk.Label(
            self.root,
            text="ROS Manual Teleop",
            font=("TkDefaultFont", 18, "bold"),
            fg="#f5f5f5",
            bg="#111111",
        )
        title.pack(pady=(16, 8))

        status = tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("TkDefaultFont", 11),
            fg="#d6d6d6",
            bg="#111111",
        )
        status.pack(pady=4)

        command = tk.Label(
            self.root,
            textvariable=self.command_var,
            font=("TkDefaultFont", 14, "bold"),
            fg="#7fe7a2",
            bg="#111111",
        )
        command.pack(pady=8)

        speed = tk.Label(
            self.root,
            textvariable=self.speed_var,
            font=("TkDefaultFont", 12),
            fg="#f5d97b",
            bg="#111111",
        )
        speed.pack(pady=4)

        hint = tk.Label(
            self.root,
            textvariable=self.hint_var,
            font=("TkDefaultFont", 10),
            fg="#bbbbbb",
            bg="#111111",
            wraplength=520,
            justify="center",
        )
        hint.pack(pady=(12, 6))

        focus_hint = tk.Label(
            self.root,
            text="Klikk i vinduet hvis tastene ikke fanges.",
            font=("TkDefaultFont", 10),
            fg="#8f8f8f",
            bg="#111111",
        )
        focus_hint.pack()

        self._add_slider("X", self.arm_x_var, self.arm_x_min, self.arm_x_max,
                         self._on_arm_x_slider, resolution=0.001)
        self._add_slider("Z", self.arm_z_var, self.arm_z_min, self.arm_z_max,
                         self._on_arm_z_slider, resolution=0.001)
        self._add_slider("G", self.gripper_var, self.gripper_min, self.gripper_max,
                         self._on_gripper_slider, resolution=1.0)

    def _add_slider(self, label_text, variable, low, high, callback, resolution):
        frame = tk.Frame(self.root, bg="#111111")
        frame.pack(fill="x", padx=22, pady=4)
        label = tk.Label(
            frame, text=label_text, width=2, anchor="w",
            font=("TkDefaultFont", 11, "bold"), fg="#f5f5f5", bg="#111111",
        )
        label.pack(side="left")
        slider = tk.Scale(
            frame, from_=low, to=high, resolution=resolution, orient="horizontal",
            variable=variable, command=lambda v: callback(float(v)),
            length=520, fg="#f5f5f5", bg="#111111", highlightthickness=0,
            troughcolor="#333333", takefocus=0,
        )
        slider.pack(side="left", padx=(10, 0))

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_force()

    def _speed_text(self) -> str:
        return (
            f"drive={self.speed:.2f} m/s  turn={self.turn_speed:.2f} rad/s\n"
            f"arm_x_speed={self.arm_x_speed:.3f} m/s  arm_z_speed={self.arm_z_speed:.3f} m/s  "
            f"gripper={self.gripper:.0f} us"
        )

    def _command_text(self, linear_x: float, angular_z: float) -> str:
        return (
            f"cmd_vel=({linear_x:.2f}, {angular_z:.2f})  "
            f"arm=(x={self.arm_x:.3f}, z={self.arm_z:.3f})  gripper={self.gripper:.0f} us"
        )

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()

        if key == "minus":
            self.close()
            return

        if key == "space":
            self.pressed_keys.clear()
            return

        first_press = key not in self.pressed_keys
        release_job = self.release_jobs.pop(key, None)
        if release_job is not None:
            self.root.after_cancel(release_job)

        if key in {"w", "a", "s", "d", "y", "h", "j", "k"}:
            self.pressed_keys.add(key)

        if not first_press:
            return

        if key == "e":
            self.speed = clamp(self.speed + self.speed_step, 0.0, self.max_speed)
        elif key == "q":
            self.speed = clamp(self.speed - self.speed_step, 0.0, self.max_speed)
        elif key == "p":
            self.turn_speed = clamp(self.turn_speed + self.turn_speed_step, 0.0, self.max_turn_speed)
        elif key == "o":
            self.turn_speed = clamp(self.turn_speed - self.turn_speed_step, 0.0, self.max_turn_speed)
        elif key == "m":
            self.arm_x_speed = clamp(self.arm_x_speed + self.arm_x_speed_step, 0.0, self.max_arm_x_speed)
        elif key == "n":
            self.arm_x_speed = clamp(self.arm_x_speed - self.arm_x_speed_step, 0.0, self.max_arm_x_speed)
        elif key == "t":
            self.arm_z_speed = clamp(self.arm_z_speed + self.arm_z_speed_step, 0.0, self.max_arm_z_speed)
        elif key == "g":
            self.arm_z_speed = clamp(self.arm_z_speed - self.arm_z_speed_step, 0.0, self.max_arm_z_speed)
        elif key == "u":
            self.gripper = clamp(self.gripper - self.gripper_step, self.gripper_min, self.gripper_max)
            self._publish_gripper()
            self._set_var_silent(self.gripper_var, self.gripper)
        elif key == "i":
            self.gripper = clamp(self.gripper + self.gripper_step, self.gripper_min, self.gripper_max)
            self._publish_gripper()
            self._set_var_silent(self.gripper_var, self.gripper)

        self.speed_var.set(self._speed_text())

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        release_job = self.release_jobs.pop(key, None)
        if release_job is not None:
            self.root.after_cancel(release_job)
        self.release_jobs[key] = self.root.after(50, lambda key=key: self._release_key(key))

    def _release_key(self, key: str) -> None:
        self.release_jobs.pop(key, None)
        self.pressed_keys.discard(key)

    def _compute_command(self) -> tuple[float, float]:
        linear_x = 0.0
        angular_z = 0.0

        if "w" in self.pressed_keys and "s" not in self.pressed_keys:
            linear_x = self.speed
        elif "s" in self.pressed_keys and "w" not in self.pressed_keys:
            linear_x = -self.speed

        if "a" in self.pressed_keys and "d" not in self.pressed_keys:
            angular_z = self.turn_speed
        elif "d" in self.pressed_keys and "a" not in self.pressed_keys:
            angular_z = -self.turn_speed

        return linear_x, angular_z

    def _publish(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def _publish_arm(self, publisher, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def _publish_gripper(self) -> None:
        msg = Float64()
        msg.data = float(self.gripper)
        self.gripper_pub.publish(msg)
        self.right_gripper_pub.publish(msg)

    def _set_var_silent(self, var: tk.DoubleVar, value: float) -> None:
        self._suppress_slider_cb = True
        try:
            var.set(value)
        finally:
            self._suppress_slider_cb = False

    def _on_arm_x_slider(self, value: float) -> None:
        if self._suppress_slider_cb:
            return
        self.arm_x = clamp(value, self.arm_x_min, self.arm_x_max)
        self._publish_arm(self.arm_x_pub, self.arm_x)

    def _on_arm_z_slider(self, value: float) -> None:
        if self._suppress_slider_cb:
            return
        self.arm_z = clamp(value, self.arm_z_min, self.arm_z_max)
        self._publish_arm(self.arm_z_pub, self.arm_z + self.arm_z_offset)

    def _on_gripper_slider(self, value: float) -> None:
        if self._suppress_slider_cb:
            return
        self.gripper = clamp(value, self.gripper_min, self.gripper_max)
        self._publish_gripper()

    def _on_arm_x_state(self, msg: Float64) -> None:
        if "j" in self.pressed_keys or "k" in self.pressed_keys:
            return
        self.arm_x = clamp(float(msg.data), self.arm_x_min, self.arm_x_max)
        self._set_var_silent(self.arm_x_var, self.arm_x)

    def _on_arm_z_state(self, msg: Float64) -> None:
        if "y" in self.pressed_keys or "h" in self.pressed_keys:
            return
        self.arm_z = clamp(float(msg.data) - self.arm_z_offset, self.arm_z_min, self.arm_z_max)
        self._set_var_silent(self.arm_z_var, self.arm_z)

    def _update_arm_targets(self, dt: float) -> None:
        x_dir = 0
        if "j" in self.pressed_keys and "k" not in self.pressed_keys:
            x_dir = 1
        elif "k" in self.pressed_keys and "j" not in self.pressed_keys:
            x_dir = -1

        z_dir = 0
        if "y" in self.pressed_keys and "h" not in self.pressed_keys:
            z_dir = 1
        elif "h" in self.pressed_keys and "y" not in self.pressed_keys:
            z_dir = -1

        if x_dir:
            self.arm_x = clamp(
                self.arm_x + x_dir * self.arm_x_speed * dt,
                self.arm_x_min,
                self.arm_x_max,
            )
            self._publish_arm(self.arm_x_pub, self.arm_x)
            self._set_var_silent(self.arm_x_var, self.arm_x)

        if z_dir:
            self.arm_z = clamp(
                self.arm_z + z_dir * self.arm_z_speed * dt,
                self.arm_z_min,
                self.arm_z_max,
            )
            self._publish_arm(self.arm_z_pub, self.arm_z + self.arm_z_offset)
            self._set_var_silent(self.arm_z_var, self.arm_z)

    def _tick(self) -> None:
        if self.closed:
            return

        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_tick_at))
        self.last_tick_at = now

        linear_x, angular_z = self._compute_command()
        self._update_arm_targets(dt)
        self.command_var.set(self._command_text(linear_x, angular_z))
        self.speed_var.set(self._speed_text())

        command = (linear_x, angular_z)
        should_repeat = command != (0.0, 0.0)
        should_send = command != self.last_command or (
            should_repeat and now - self.last_sent_at >= self.send_period
        )

        if should_send:
            self._publish(linear_x, angular_z)
            self.last_command = command
            self.last_sent_at = now

        self.root.after(20, self._tick)

    def _spin_ros(self) -> None:
        if self.closed:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self._spin_ros)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self._publish(0.0, 0.0)
        self.root.after(20, self.root.destroy)

    def run(self) -> int:
        self.root.mainloop()
        return 0

    def shutdown(self) -> None:
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass
        self.node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Keyboard teleop over ROS for the real robot with Nav2-safe manual override."
    )
    parser.add_argument("--topic", default="/cmd_vel_manual", help="Twist topic to publish manual commands to")
    parser.add_argument("--speed", type=float, default=0.20, help="Default linear speed in m/s")
    parser.add_argument("--turn-speed", type=float, default=0.90, help="Default angular speed in rad/s")
    parser.add_argument("--speed-step", type=float, default=0.005, help="Linear speed increment for E/Q")
    parser.add_argument("--turn-speed-step", type=float, default=0.10, help="Angular speed increment for P/O")
    parser.add_argument("--max-speed", type=float, default=0.50, help="Maximum linear speed in m/s")
    parser.add_argument("--max-turn-speed", type=float, default=3.70, help="Maximum angular speed in rad/s")
    parser.add_argument("--send-period", type=float, default=0.03, help="Seconds between repeated cmd_vel publishes")
    parser.add_argument("--arm-x-topic", default="/robotarm/request/x_position", help="Robot arm x request topic")
    parser.add_argument("--arm-z-topic", default="/robotarm/request/z_position", help="Robot arm z request topic")
    parser.add_argument("--gripper-topic", default="/gripper/request/left_position", help="Gripper request topic")
    parser.add_argument("--right-gripper-topic", default="/gripper/request/right_position", help="Right gripper request topic (mirrors left for the single physical servo)")
    parser.add_argument("--arm-x-state-topic", default="/robotarm/x_position_cmd", help="Robot arm x command feedback topic")
    parser.add_argument("--arm-z-state-topic", default="/robotarm/z_position_cmd", help="Robot arm z command feedback topic")
    parser.add_argument("--arm-x-initial", type=float, default=0.0, help="Initial arm x target in meters")
    parser.add_argument("--arm-z-initial", type=float, default=0.227, help="Initial arm z target in meters")
    parser.add_argument("--arm-x-min", type=float, default=-0.2, help="Minimum arm x target in meters")
    parser.add_argument("--arm-x-max", type=float, default=0.2, help="Maximum arm x target in meters")
    parser.add_argument("--arm-z-min", type=float, default=0.112, help="Minimum arm z target in meters")
    parser.add_argument("--arm-z-max", type=float, default=0.3, help="Maximum arm z target in meters")
    parser.add_argument("--arm-z-offset", type=float, default=0.0, help="Offset added to arm z before publishing (control zero-point shift)")
    parser.add_argument("--arm-x-speed", type=float, default=0.010, help="Default arm x jog speed in m/s")
    parser.add_argument("--arm-z-speed", type=float, default=0.002, help="Default arm z jog speed in m/s")
    parser.add_argument("--arm-x-speed-step", type=float, default=0.002, help="Arm x jog speed increment for M/N")
    parser.add_argument("--arm-z-speed-step", type=float, default=0.001, help="Arm z jog speed increment for T/G")
    parser.add_argument("--max-arm-x-speed", type=float, default=0.050, help="Maximum arm x jog speed in m/s")
    parser.add_argument("--max-arm-z-speed", type=float, default=0.010, help="Maximum arm z jog speed in m/s")
    parser.add_argument("--gripper-min", type=float, default=500.0, help="Minimum gripper pulse width in microseconds")
    parser.add_argument("--gripper-max", type=float, default=2500.0, help="Maximum gripper pulse width in microseconds")
    parser.add_argument("--gripper-initial", type=float, default=500.0, help="Initial gripper pulse width in microseconds")
    parser.add_argument("--gripper-step", type=float, default=50.0, help="Gripper pulse-width increment for U/I")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    app = RosKeyboardTeleop(args)
    exit_code = 0
    try:
        exit_code = app.run()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        app.shutdown()
        rclpy.try_shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
