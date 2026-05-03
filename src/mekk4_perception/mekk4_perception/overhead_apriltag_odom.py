import math
from typing import Dict, List, Tuple

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


PixelPoint = Tuple[float, float]


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    half_yaw = 0.5 * yaw
    return math.sin(half_yaw), math.cos(half_yaw)


def _point_xy(point) -> PixelPoint:
    return float(point.x), float(point.y)


def _detection_centre(detection) -> PixelPoint:
    if hasattr(detection, "centre"):
        return _point_xy(detection.centre)
    if hasattr(detection, "center"):
        return _point_xy(detection.center)

    corners = [_point_xy(corner) for corner in detection.corners]
    xs = [corner[0] for corner in corners]
    ys = [corner[1] for corner in corners]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def _detection_corners(detection) -> List[PixelPoint]:
    return [_point_xy(corner) for corner in detection.corners]


class OverheadApriltagOdom(Node):
    def __init__(self) -> None:
        super().__init__("overhead_apriltag_odom")

        self.declare_parameter("detections_topic", "/overhead/tag_detections")
        self.declare_parameter("odom_topic", "/overhead/odom")
        self.declare_parameter("pose_topic", "/overhead/pose")
        self.declare_parameter("pose2d_topic", "/overhead/pose2d")
        self.declare_parameter("publish_pose", True)
        self.declare_parameter("publish_pose2d", True)
        self.declare_parameter("field_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("robot_tag_frame_id", "robot_tag_20")
        self.declare_parameter("publish_robot_tag_tf", True)
        self.declare_parameter("robot_tag_id", 20)
        self.declare_parameter("field_tag_ids", [10, 11, 12, 13])
        self.declare_parameter("field_tag_x", [-2.2, 2.2, 2.2, -2.2])
        self.declare_parameter("field_tag_y", [-1.0, -1.0, 1.0, 1.0])
        self.declare_parameter("robot_yaw_offset_rad", 0.0)
        self.declare_parameter("position_variance", 0.0004)
        self.declare_parameter("yaw_variance", 0.0025)
        self.declare_parameter("warn_period_s", 2.0)

        self._field_frame_id = self.get_parameter("field_frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value
        self._robot_tag_frame_id = self.get_parameter("robot_tag_frame_id").value
        self._publish_robot_tag_tf = bool(self.get_parameter("publish_robot_tag_tf").value)
        self._robot_tag_id = int(self.get_parameter("robot_tag_id").value)
        self._robot_yaw_offset_rad = float(self.get_parameter("robot_yaw_offset_rad").value)
        self._position_variance = float(self.get_parameter("position_variance").value)
        self._yaw_variance = float(self.get_parameter("yaw_variance").value)
        self._warn_period_s = float(self.get_parameter("warn_period_s").value)

        field_ids = [int(tag_id) for tag_id in self.get_parameter("field_tag_ids").value]
        field_x = [float(x) for x in self.get_parameter("field_tag_x").value]
        field_y = [float(y) for y in self.get_parameter("field_tag_y").value]
        if len(field_ids) != len(field_x) or len(field_ids) != len(field_y):
            raise ValueError("field_tag_ids, field_tag_x and field_tag_y must have equal lengths")
        if len(field_ids) < 4:
            raise ValueError("At least four field tags are required for homography")

        self._field_tags = {
            tag_id: (x, y)
            for tag_id, x, y in zip(field_ids, field_x, field_y)
        }

        self._last_warn_ns = 0
        self._tf_broadcaster = (
            TransformBroadcaster(self)
            if self._publish_robot_tag_tf
            else None
        )

        detections_topic = self.get_parameter("detections_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        pose2d_topic = self.get_parameter("pose2d_topic").value
        publish_pose = bool(self.get_parameter("publish_pose").value)
        publish_pose2d = bool(self.get_parameter("publish_pose2d").value)

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._pose_pub = (
            self.create_publisher(PoseStamped, pose_topic, 10)
            if publish_pose
            else None
        )
        self._pose2d_pub = (
            self.create_publisher(Pose2D, pose2d_topic, 10)
            if publish_pose2d
            else None
        )
        self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            10,
        )

        self.get_logger().info(
            f"Publishing {odom_topic} from robot AprilTag {self._robot_tag_id} "
            f"using field tags {sorted(self._field_tags)}"
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        detections = self._detections_by_id(msg)

        missing_field_tags = [
            tag_id for tag_id in self._field_tags
            if tag_id not in detections
        ]
        if missing_field_tags:
            self._warn_throttled(
                f"Missing field AprilTags: {missing_field_tags}; "
                f"seen tags: {sorted(detections)}"
            )
            return
        if self._robot_tag_id not in detections:
            self._warn_throttled(
                f"Missing robot AprilTag: {self._robot_tag_id}; "
                f"seen tags: {sorted(detections)}"
            )
            return

        image_points = []
        field_points = []
        for tag_id, field_point in self._field_tags.items():
            image_points.append(_detection_centre(detections[tag_id]))
            field_points.append(field_point)

        homography, _ = cv2.findHomography(
            np.asarray(image_points, dtype=np.float32),
            np.asarray(field_points, dtype=np.float32),
            method=0,
        )
        if homography is None:
            self._warn_throttled("Could not compute overhead camera homography")
            return

        robot_detection = detections[self._robot_tag_id]
        robot_pixel = np.asarray([[_detection_centre(robot_detection)]], dtype=np.float32)
        robot_field = cv2.perspectiveTransform(robot_pixel, homography)[0, 0]

        robot_corners = np.asarray([_detection_corners(robot_detection)], dtype=np.float32)
        field_corners = cv2.perspectiveTransform(robot_corners, homography)[0]
        edge = field_corners[1] - field_corners[0]
        yaw = math.atan2(float(edge[1]), float(edge[0])) + self._robot_yaw_offset_rad
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        self._publish_odom(msg, float(robot_field[0]), float(robot_field[1]), yaw)

    def _detections_by_id(self, msg: AprilTagDetectionArray) -> Dict[int, object]:
        detections = {}
        for detection in msg.detections:
            corners = _detection_corners(detection)
            if len(corners) != 4:
                continue
            detections[int(detection.id)] = detection
        return detections

    def _publish_odom(self, msg: AprilTagDetectionArray, x: float, y: float, yaw: float) -> None:
        qz, qw = _yaw_to_quaternion(yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._field_frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = self._position_variance
        odom.pose.covariance[7] = self._position_variance
        odom.pose.covariance[14] = 999.0
        odom.pose.covariance[21] = 999.0
        odom.pose.covariance[28] = 999.0
        odom.pose.covariance[35] = self._yaw_variance
        for i in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[i] = 999.0

        self._odom_pub.publish(odom)

        if self._pose_pub is not None:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self._pose_pub.publish(pose)

        if self._pose2d_pub is not None:
            pose2d = Pose2D()
            pose2d.x = x
            pose2d.y = y
            pose2d.theta = yaw
            self._pose2d_pub.publish(pose2d)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self._robot_tag_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns < self._warn_period_s * 1e9:
            return
        self._last_warn_ns = now_ns
        self.get_logger().warn(message)


def main() -> None:
    rclpy.init()
    node = OverheadApriltagOdom()
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
