#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RobotarmGui:
    def __init__(self, args: argparse.Namespace) -> None:
        self.node = rclpy.create_node("robotarm_gui")
        self.x_pub = self.node.create_publisher(Float64, args.x_topic, 10)
        self.z_pub = self.node.create_publisher(Float64, args.z_topic, 10)
        self.left_gripper_pub = self.node.create_publisher(Float64, args.left_gripper_topic, 10)
        self.right_gripper_pub = self.node.create_publisher(Float64, args.right_gripper_topic, 10)

        self.x_min = args.x_min
        self.x_max = args.x_max
        self.z_min = args.z_min
        self.z_max = args.z_max
        self.gripper_min = args.gripper_min
        self.gripper_max = args.gripper_max
        self.x_initial = clamp(args.x_initial, self.x_min, self.x_max)
        self.z_initial = clamp(args.z_initial, self.z_min, self.z_max)
        self.gripper_initial = clamp(
            args.gripper_initial, self.gripper_min, self.gripper_max
        )
        self.closed = False

        self.root = tk.Tk()
        self.root.title("Robotarm Control")
        self.root.geometry("420x280")
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.x_var = tk.DoubleVar(value=self.x_initial)
        self.z_var = tk.DoubleVar(value=self.z_initial)
        self.gripper_var = tk.DoubleVar(value=self.gripper_initial)
        self.status_var = tk.StringVar(value=self._status_text())

        self._build_ui()
        self.root.after(20, self._spin_ros)
        self.root.after(100, self._publish_once)

    def _build_ui(self) -> None:
        style = ttk.Style()
        style.configure("TScale", background="#111111")

        title = tk.Label(
            self.root,
            text="Robotarm",
            font=("TkDefaultFont", 18, "bold"),
            fg="#f5f5f5",
            bg="#111111",
        )
        title.pack(pady=(14, 6))

        self._add_slider("X", self.x_var, self.x_min, self.x_max)
        self._add_slider("Z", self.z_var, self.z_min, self.z_max)
        self._add_slider("G", self.gripper_var, self.gripper_min, self.gripper_max)

        status = tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("TkDefaultFont", 12, "bold"),
            fg="#7fe7a2",
            bg="#111111",
        )
        status.pack(pady=(8, 6))

        reset = tk.Button(self.root, text="Nullstill", command=self._reset)
        reset.pack()

    def _add_slider(self, label_text: str, variable: tk.DoubleVar, low: float, high: float) -> None:
        frame = tk.Frame(self.root, bg="#111111")
        frame.pack(fill="x", padx=22, pady=6)

        label = tk.Label(
            frame,
            text=label_text,
            width=2,
            anchor="w",
            font=("TkDefaultFont", 11, "bold"),
            fg="#f5f5f5",
            bg="#111111",
        )
        label.pack(side="left")

        slider = tk.Scale(
            frame,
            from_=low,
            to=high,
            resolution=0.001,
            orient="horizontal",
            variable=variable,
            command=lambda _value: self._on_slider_changed(),
            length=320,
            fg="#f5f5f5",
            bg="#111111",
            highlightthickness=0,
            troughcolor="#333333",
        )
        slider.pack(side="left", padx=(10, 0))

    def _status_text(self) -> str:
        return (
            f"x={self.x_var.get():.3f} m   "
            f"z={self.z_var.get():.3f} m   "
            f"g={self.gripper_var.get():.0f} µs"
        )

    def _on_slider_changed(self) -> None:
        self.status_var.set(self._status_text())
        self._publish_once()

    def _reset(self) -> None:
        self.x_var.set(self.x_initial)
        self.z_var.set(self.z_initial)
        self.gripper_var.set(self.gripper_initial)
        self.status_var.set(self._status_text())
        self._publish_once()

    def _publish_once(self) -> None:
        x_msg = Float64()
        x_msg.data = clamp(float(self.x_var.get()), self.x_min, self.x_max)
        self.x_pub.publish(x_msg)

        z_msg = Float64()
        z_msg.data = clamp(float(self.z_var.get()), self.z_min, self.z_max)
        self.z_pub.publish(z_msg)

        gripper_position = clamp(
            float(self.gripper_var.get()), self.gripper_min, self.gripper_max
        )
        # One physical servo: publish the same PWM µs value to both legacy topics
        # so the gripper_sim_adapter (and the real Mega driver) sees consistent input.
        pwm_msg = Float64()
        pwm_msg.data = gripper_position
        self.left_gripper_pub.publish(pwm_msg)
        self.right_gripper_pub.publish(pwm_msg)

    def _spin_ros(self) -> None:
        if self.closed:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self._spin_ros)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.root.after(20, self.root.destroy)

    def run(self) -> int:
        self.root.mainloop()
        return 0

    def shutdown(self) -> None:
        self.node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description="Small GUI for simulated robotarm joint targets.")
    parser.add_argument("--x-topic", default="/robotarm/request/x_position")
    parser.add_argument("--z-topic", default="/robotarm/request/z_position")
    parser.add_argument("--left-gripper-topic", default="/gripper/request/left_position")
    parser.add_argument("--right-gripper-topic", default="/gripper/request/right_position")
    parser.add_argument("--x-min", type=float, default=0.01)
    parser.add_argument("--x-max", type=float, default=0.09)
    parser.add_argument("--z-min", type=float, default=0.0)
    parser.add_argument("--z-max", type=float, default=0.25)
    parser.add_argument("--gripper-min", type=float, default=500.0)
    parser.add_argument("--gripper-max", type=float, default=2500.0)
    parser.add_argument("--x-initial", type=float, default=0.01)
    parser.add_argument("--z-initial", type=float, default=0.12)
    parser.add_argument("--gripper-initial", type=float, default=500.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    app = RobotarmGui(args)
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
