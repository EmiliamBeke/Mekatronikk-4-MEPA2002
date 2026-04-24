#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SimCmdVelCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("sim_cmd_vel_calibrator")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/sim_cmd_vel")
        self.declare_parameter("track_width_eff_m", 0.184)
        self.declare_parameter("max_track_speed_mps", 0.45)
        self.declare_parameter("swap_sides", False)
        self.declare_parameter("left_cmd_sign", 1)
        self.declare_parameter("right_cmd_sign", 1)
        self.declare_parameter("left_cmd_scale", 1.0)
        self.declare_parameter("right_cmd_scale", 1.0)

        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._track_width_eff_m = float(self.get_parameter("track_width_eff_m").value)
        self._max_track_speed_mps = float(self.get_parameter("max_track_speed_mps").value)
        self._swap_sides = bool(self.get_parameter("swap_sides").value)
        self._left_cmd_sign = int(self.get_parameter("left_cmd_sign").value)
        self._right_cmd_sign = int(self.get_parameter("right_cmd_sign").value)
        self._left_cmd_scale = float(self.get_parameter("left_cmd_scale").value)
        self._right_cmd_scale = float(self.get_parameter("right_cmd_scale").value)

        if self._track_width_eff_m <= 0.0:
            raise ValueError("track_width_eff_m must be greater than zero")
        if self._max_track_speed_mps <= 0.0:
            raise ValueError("max_track_speed_mps must be greater than zero")
        if self._left_cmd_scale <= 0.0 or self._right_cmd_scale <= 0.0:
            raise ValueError("left_cmd_scale and right_cmd_scale must be greater than zero")
        if self._left_cmd_sign == 0 or self._right_cmd_sign == 0:
            raise ValueError("left_cmd_sign and right_cmd_sign must be non-zero")

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._sub = self.create_subscription(Twist, self._input_topic, self._on_cmd_vel, 10)

        self.get_logger().info(
            "Calibrating sim cmd_vel %s -> %s with track_width_eff_m=%.6f max_track_speed_mps=%.3f "
            "swap_sides=%s left_cmd_sign=%d right_cmd_sign=%d left_cmd_scale=%.3f right_cmd_scale=%.3f"
            % (
                self._input_topic,
                self._output_topic,
                self._track_width_eff_m,
                self._max_track_speed_mps,
                str(self._swap_sides).lower(),
                self._left_cmd_sign,
                self._right_cmd_sign,
                self._left_cmd_scale,
                self._right_cmd_scale,
            )
        )

    def _apply_track_calibration(self, track_speed: float, cmd_scale: float, cmd_sign: int) -> float:
        scaled = track_speed * cmd_scale
        clamped = clamp(scaled, -self._max_track_speed_mps, self._max_track_speed_mps)
        sign = 1.0 if cmd_sign > 0 else -1.0
        return clamped * sign

    def _on_cmd_vel(self, msg: Twist) -> None:
        half_width = self._track_width_eff_m / 2.0

        left_speed = msg.linear.x - (msg.angular.z * half_width)
        right_speed = msg.linear.x + (msg.angular.z * half_width)

        left_speed = self._apply_track_calibration(
            left_speed, self._left_cmd_scale, self._left_cmd_sign
        )
        right_speed = self._apply_track_calibration(
            right_speed, self._right_cmd_scale, self._right_cmd_sign
        )
        if self._swap_sides:
            left_speed, right_speed = right_speed, left_speed

        calibrated = Twist()
        calibrated.linear.x = 0.5 * (left_speed + right_speed)
        calibrated.angular.z = (right_speed - left_speed) / self._track_width_eff_m
        self._pub.publish(calibrated)


def main() -> int:
    rclpy.init()
    node = SimCmdVelCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
