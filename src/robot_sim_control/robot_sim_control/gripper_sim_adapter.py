#!/usr/bin/env python3
"""Convert real gripper PWM (µs) commands into Gazebo finger joint angles (rad).

The real robot has one servo whose PWM (500–2500 µs) drives both fingers via a
gear/rack. In sim, each finger is a separate revolute joint with its own range:

  left_gripper_finger_joint:  [-3.228859, -0.523599] rad
  right_gripper_finger_joint: [ 0.523599,  3.228859 ] rad

The convention here: PWM=500 µs ⇒ fully open (extreme angles: ±3.228859),
PWM=2500 µs ⇒ fully closed (near-centre: ±0.523599). We subscribe to the existing
real-robot command topic (`/gripper/left_position_cmd` is what teleop and
robotarm_safety publish), interpret it as µs, and republish per-finger angles.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class GripperSimAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gripper_sim_adapter")

        self.declare_parameter("input_topic", "/gripper/left_position_cmd")
        self.declare_parameter("left_output_topic", "/sim/gripper/left_angle_cmd")
        self.declare_parameter("right_output_topic", "/sim/gripper/right_angle_cmd")
        self.declare_parameter("pwm_min_us", 500.0)
        self.declare_parameter("pwm_max_us", 2500.0)
        self.declare_parameter("left_open_rad", -3.228859)
        self.declare_parameter("left_closed_rad", -0.523599)
        self.declare_parameter("right_open_rad", 3.228859)
        self.declare_parameter("right_closed_rad", 0.523599)
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("initial_pwm_us", 500.0)

        self.pwm_min = float(self.get_parameter("pwm_min_us").value)
        self.pwm_max = float(self.get_parameter("pwm_max_us").value)
        self.left_open = float(self.get_parameter("left_open_rad").value)
        self.left_closed = float(self.get_parameter("left_closed_rad").value)
        self.right_open = float(self.get_parameter("right_open_rad").value)
        self.right_closed = float(self.get_parameter("right_closed_rad").value)
        self.current_pwm = clamp(
            float(self.get_parameter("initial_pwm_us").value), self.pwm_min, self.pwm_max
        )

        self.left_pub = self.create_publisher(
            Float64, str(self.get_parameter("left_output_topic").value), 10
        )
        self.right_pub = self.create_publisher(
            Float64, str(self.get_parameter("right_output_topic").value), 10
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("input_topic").value),
            self._on_pwm,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_s").value), self._publish
        )

        self.get_logger().info(
            "gripper_sim_adapter: %s (µs) -> %s, %s (rad), home=%.0f µs"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("left_output_topic").value,
                self.get_parameter("right_output_topic").value,
                self.current_pwm,
            )
        )

    def _on_pwm(self, msg: Float64) -> None:
        self.current_pwm = clamp(float(msg.data), self.pwm_min, self.pwm_max)

    def _publish(self) -> None:
        span = self.pwm_max - self.pwm_min
        opening = 0.0 if span <= 0.0 else (self.current_pwm - self.pwm_min) / span
        opening = clamp(opening, 0.0, 1.0)
        left_angle = self.left_open + opening * (self.left_closed - self.left_open)
        right_angle = self.right_open + opening * (self.right_closed - self.right_open)
        self.left_pub.publish(Float64(data=left_angle))
        self.right_pub.publish(Float64(data=right_angle))


def main() -> None:
    rclpy.init()
    node = GripperSimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
