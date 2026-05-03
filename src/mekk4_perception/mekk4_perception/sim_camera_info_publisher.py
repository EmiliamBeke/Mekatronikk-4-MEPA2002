import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SimCameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_camera_info_publisher")

        self.declare_parameter("image_topic", "/overhead_camera")
        self.declare_parameter("camera_info_topic", "/overhead_camera/camera_info")
        self.declare_parameter("camera_info_alias_topic", "/camera_info")
        self.declare_parameter("frame_id", "overhead_camera_link")
        self.declare_parameter("horizontal_fov", 1.5707963267948966)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        alias_topic = self.get_parameter("camera_info_alias_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._horizontal_fov = float(self.get_parameter("horizontal_fov").value)

        self._camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self._alias_pub = (
            self.create_publisher(CameraInfo, alias_topic, 10)
            if alias_topic
            else None
        )
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f"Publishing CameraInfo for {image_topic} on {camera_info_topic}"
        )

    def _on_image(self, image: Image) -> None:
        info = CameraInfo()
        info.header.stamp = image.header.stamp
        info.header.frame_id = self._frame_id
        info.width = image.width
        info.height = image.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = image.width / (2.0 * math.tan(self._horizontal_fov / 2.0))
        fy = fx
        cx = (image.width - 1.0) / 2.0
        cy = (image.height - 1.0) / 2.0

        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        self._camera_info_pub.publish(info)
        if self._alias_pub is not None:
            self._alias_pub.publish(info)


def main() -> None:
    rclpy.init()
    node = SimCameraInfoPublisher()
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
