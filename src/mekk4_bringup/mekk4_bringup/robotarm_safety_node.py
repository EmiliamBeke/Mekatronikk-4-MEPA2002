#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


PARAM_DEFAULTS = {
    "x_request_topic": "/robotarm/request/x_position",
    "z_request_topic": "/robotarm/request/z_position",
    "left_gripper_request_topic": "/gripper/request/left_position",
    "right_gripper_request_topic": "/gripper/request/right_position",
    "x_command_topic": "/robotarm/x_position_cmd",
    "z_command_topic": "/robotarm/z_position_cmd",
    "left_gripper_command_topic": "/gripper/left_position_cmd",
    "right_gripper_command_topic": "/gripper/right_position_cmd",
    "x_state_topic": "/robotarm/x_position_state",
    "z_state_topic": "/robotarm/z_position_state",
    "joint_states_topic": "/joint_states",
    "z_joint_name": "robotarm_z_joint",
    "publish_period_s": 0.05,
    "x_min": -0.2,
    "x_max": 0.2,
    "z_min": 0.0,
    "z_max": 0.3,
    "gripper_min": 500.0,
    "gripper_max": 1800.0,
    "lidar_x_threshold": 0.08,
    "lidar_z_clearance": 0.12,
    "max_x_step_per_publish": 0.005,
    "max_z_step_per_publish": 0.001,
    "initial_x": 0.0,
    "initial_z": 0.12,
    "initial_left_gripper": 500.0,
    "initial_right_gripper": 500.0,
    "startup_lock_s": 1.5,
    "spawn_x": 0.0,
    "spawn_z": 0.0,
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def step_towards(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if abs(delta) <= max_step:
        return target
    if delta > 0.0:
        return current + max_step
    return current - max_step


class RobotarmSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("robotarm_safety")

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.x_min = float(self.param("x_min"))
        self.x_max = float(self.param("x_max"))
        self.z_min = float(self.param("z_min"))
        self.z_max = float(self.param("z_max"))
        self.gripper_min = float(self.param("gripper_min"))
        self.gripper_max = float(self.param("gripper_max"))
        self.lidar_x_threshold = float(self.param("lidar_x_threshold"))
        self.lidar_z_clearance = float(self.param("lidar_z_clearance"))
        self.max_x_step_per_publish = float(self.param("max_x_step_per_publish"))
        self.max_z_step_per_publish = float(self.param("max_z_step_per_publish"))
        self.z_joint_name = str(self.param("z_joint_name"))

        if self.max_x_step_per_publish <= 0.0:
            raise ValueError("max_x_step_per_publish must be greater than zero.")
        if self.max_z_step_per_publish <= 0.0:
            raise ValueError("max_z_step_per_publish must be greater than zero.")

        self.requested_x = clamp(float(self.param("initial_x")), self.x_min, self.x_max)
        self.requested_z = clamp(float(self.param("initial_z")), self.z_min, self.z_max)
        self.requested_left_gripper = clamp(
            float(self.param("initial_left_gripper")),
            self.gripper_min,
            self.gripper_max,
        )
        self.requested_right_gripper = clamp(
            float(self.param("initial_right_gripper")),
            self.gripper_min,
            self.gripper_max,
        )
        self.current_x: float | None = None
        self.current_z: float | None = None
        self.commanded_x = self.requested_x
        self.commanded_z = self.requested_z
        self.startup_lock_s = float(self.param("startup_lock_s"))
        self._startup_t0 = None
        self.spawn_x = float(self.param("spawn_x"))
        self.spawn_z = float(self.param("spawn_z"))

        self.x_pub = self.create_publisher(Float64, self.param("x_command_topic"), 10)
        self.z_pub = self.create_publisher(Float64, self.param("z_command_topic"), 10)
        self.left_gripper_pub = self.create_publisher(
            Float64, self.param("left_gripper_command_topic"), 10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64, self.param("right_gripper_command_topic"), 10
        )

        self.create_subscription(
            Float64,
            self.param("x_request_topic"),
            lambda msg: self.set_x(msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            self.param("z_request_topic"),
            lambda msg: self.set_z(msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            self.param("left_gripper_request_topic"),
            lambda msg: self.set_left_gripper(msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            self.param("right_gripper_request_topic"),
            lambda msg: self.set_right_gripper(msg.data),
            10,
        )
        self.create_subscription(
            JointState,
            self.param("joint_states_topic"),
            self.on_joint_states,
            10,
        )
        self.create_subscription(
            Float64,
            self.param("x_state_topic"),
            self.on_x_state,
            10,
        )
        self.create_subscription(
            Float64,
            self.param("z_state_topic"),
            self.on_z_state,
            10,
        )
        self.create_timer(float(self.param("publish_period_s")), self.on_timer)

        self.get_logger().info(
            "robotarm safety active: z_min=%.3f lidar_clearance=(x<%.3f => z>=%.3f)"
            % (self.z_min, self.lidar_x_threshold, self.lidar_z_clearance)
        )

    def param(self, name: str):
        return self.get_parameter(name).value

    def set_x(self, value: float) -> None:
        self.requested_x = clamp(float(value), self.x_min, self.x_max)

    def set_z(self, value: float) -> None:
        self.requested_z = clamp(float(value), self.z_min, self.z_max)

    def set_left_gripper(self, value: float) -> None:
        self.requested_left_gripper = clamp(float(value), self.gripper_min, self.gripper_max)

    def set_right_gripper(self, value: float) -> None:
        self.requested_right_gripper = clamp(float(value), self.gripper_min, self.gripper_max)

    def on_x_state(self, msg: Float64) -> None:
        self.current_x = clamp(float(msg.data), self.x_min, self.x_max)

    def on_z_state(self, msg: Float64) -> None:
        self.current_z = clamp(float(msg.data), self.z_min, self.z_max)

    def on_joint_states(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.z_joint_name)
        except ValueError:
            return
        if index < len(msg.position):
            self.current_z = float(msg.position[index])

    def on_timer(self) -> None:
        if self._startup_t0 is None:
            self._startup_t0 = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._startup_t0).nanoseconds * 1e-9
        if elapsed < self.startup_lock_s:
            # Hold spawn pose so z PID can settle before x retracts.
            self.publish(self.x_pub, self.spawn_x)
            self.publish(self.z_pub, self.spawn_z)
            self.commanded_x = self.spawn_x
            self.commanded_z = self.spawn_z
            self.publish(self.left_gripper_pub, self.requested_left_gripper)
            self.publish(self.right_gripper_pub, self.requested_right_gripper)
            return
        x_position, z_position = self.commanded_xz()
        self.commanded_x = x_position
        self.commanded_z = z_position
        self.publish(self.x_pub, x_position)
        self.publish(self.z_pub, z_position)
        self.publish(self.left_gripper_pub, self.requested_left_gripper)
        self.publish(self.right_gripper_pub, self.requested_right_gripper)

    def commanded_xz(self) -> tuple[float, float]:
        target_x, target_z = self.safe_requested_xz()
        x_position = self.current_x if self.current_x is not None else self.commanded_x
        z_position = self.current_z if self.current_z is not None else self.commanded_z

        # Tolerance avoids deadlock when z hovers within a few mm of clearance
        # because of PID jitter or sensor noise.
        z_tol = 0.005

        in_keepout = (
            x_position < self.lidar_x_threshold
            and z_position < self.lidar_z_clearance - z_tol
        )

        if in_keepout:
            # Only enforce z-up when current z is meaningfully below clearance.
            z_position = step_towards(
                z_position,
                self.lidar_z_clearance,
                self.max_z_step_per_publish,
            )
            return x_position, z_position

        # Block lowering z below clearance while x stays inside the keepout band.
        if (
            target_z < self.lidar_z_clearance - z_tol
            and x_position < self.lidar_x_threshold
        ):
            target_z = self.lidar_z_clearance

        # Move z and x simultaneously; do not gate one behind the other once
        # we are out of the hard keepout zone.
        if abs(z_position - target_z) > 1e-9:
            z_position = step_towards(z_position, target_z, self.max_z_step_per_publish)

        if abs(x_position - target_x) > 1e-9:
            x_position = step_towards(x_position, target_x, self.max_x_step_per_publish)

        return x_position, z_position

    def safe_requested_xz(self) -> tuple[float, float]:
        x_position = self.requested_x
        z_position = self.requested_z

        if self.in_lidar_keepout(x_position, z_position):
            z_position = clamp(self.lidar_z_clearance, self.z_min, self.z_max)

        return x_position, z_position

    def in_lidar_keepout(self, x_position: float, z_position: float) -> bool:
        return x_position < self.lidar_x_threshold and z_position < self.lidar_z_clearance

    def publish(self, publisher, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = RobotarmSafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
