#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.utilities import remove_ros_args


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RosKeyboardTeleop:
    def __init__(self, args: argparse.Namespace) -> None:
        self.node = rclpy.create_node("ros_keyboard_teleop")
        self.pub = self.node.create_publisher(Twist, args.topic, 10)

        self.topic = args.topic
        self.speed = max(0.0, args.speed)
        self.turn_speed = max(0.0, args.turn_speed)
        self.speed_step = max(0.01, args.speed_step)
        self.turn_speed_step = max(0.01, args.turn_speed_step)
        self.max_speed = max(self.speed, args.max_speed)
        self.max_turn_speed = max(self.turn_speed, args.max_turn_speed)
        self.send_period = max(0.01, args.send_period)

        self.pressed_keys: set[str] = set()
        self.last_command = (None, None)
        self.last_sent_at = 0.0
        self.closed = False

        self.root = tk.Tk()
        self.root.title("ROS Keyboard Teleop")
        self.root.geometry("560x260")
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = tk.StringVar(value=f"Publishing to {self.topic}")
        self.command_var = tk.StringVar(value="cmd_vel=(0.00, 0.00)")
        self.speed_var = tk.StringVar(value=self._speed_text())
        self.hint_var = tk.StringVar(
            value="Hold W/S/A/D. E/Q speed. P/O turn speed. SPACE stop. - quit."
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

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_force()

    def _speed_text(self) -> str:
        return f"speed={self.speed:.2f} m/s  turn_speed={self.turn_speed:.2f} rad/s"

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()

        if key == "minus":
            self.close()
            return

        if key == "space":
            self.pressed_keys.clear()
            return

        first_press = key not in self.pressed_keys
        if key in {"w", "a", "s", "d"}:
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

        self.speed_var.set(self._speed_text())

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
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

    def _tick(self) -> None:
        if self.closed:
            return

        linear_x, angular_z = self._compute_command()
        self.command_var.set(f"cmd_vel=({linear_x:.2f}, {angular_z:.2f})")
        self.speed_var.set(self._speed_text())

        now = time.monotonic()
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
    parser.add_argument("--speed-step", type=float, default=0.02, help="Linear speed increment for E/Q")
    parser.add_argument("--turn-speed-step", type=float, default=0.10, help="Angular speed increment for P/O")
    parser.add_argument("--max-speed", type=float, default=0.40, help="Maximum linear speed in m/s")
    parser.add_argument("--max-turn-speed", type=float, default=3.70, help="Maximum angular speed in rad/s")
    parser.add_argument("--send-period", type=float, default=0.03, help="Seconds between repeated cmd_vel publishes")
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
