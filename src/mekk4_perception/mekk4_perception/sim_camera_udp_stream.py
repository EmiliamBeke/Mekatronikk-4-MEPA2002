import shlex
import subprocess

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class SimCameraUdpStream(Node):
    def __init__(self):
        super().__init__("sim_camera_udp_stream")

        self.declare_parameter("image_topic", "/camera")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5600)
        self.declare_parameter("fps", 15)
        self.declare_parameter("bitrate_kbps", 1400)

        self.image_topic = self.get_parameter("image_topic").value
        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.fps = int(self.get_parameter("fps").value)
        self.bitrate_kbps = int(self.get_parameter("bitrate_kbps").value)

        self.bridge = CvBridge()
        self.proc = None
        self.frame_size = None
        self.sub = self.create_subscription(Image, self.image_topic, self._image_callback, 1)

        self.get_logger().info(f"Streaming {self.image_topic} as RTP/H264 to {self.host}:{self.port}")

    def _image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = frame.shape[:2]

        if self.proc is None or self.proc.poll() is not None or self.frame_size != (width, height):
            self._start_pipeline(width, height)

        if self.proc is None or self.proc.stdin is None:
            return

        try:
            self.proc.stdin.write(frame.tobytes())
            self.proc.stdin.flush()
        except Exception as exc:
            self.get_logger().warning(f"camera UDP stream write failed: {exc}")
            self._stop_pipeline()

    def _start_pipeline(self, width, height):
        self._stop_pipeline()
        self.frame_size = (width, height)

        pipeline = (
            "fdsrc ! "
            f"videoparse format=bgr width={width} height={height} framerate={self.fps}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoconvert ! "
            "video/x-raw,format=I420 ! "
            f"openh264enc bitrate={self.bitrate_kbps * 1000} gop-size={self.fps} rate-control=bitrate complexity=low ! "
            "h264parse ! "
            "rtph264pay pt=96 config-interval=1 ! "
            f"udpsink host={self.host} port={self.port} sync=false async=false"
        )
        cmd = ["gst-launch-1.0", "-q"] + shlex.split(pipeline)
        self.get_logger().info(f"pipeline: {' '.join(cmd)}")
        try:
            self.proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except Exception as exc:
            self.proc = None
            self.get_logger().error(f"failed to start gst-launch-1.0: {exc}")

    def _stop_pipeline(self):
        if self.proc is None:
            return
        try:
            if self.proc.stdin is not None:
                self.proc.stdin.close()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(timeout=1.0)
        except Exception:
            pass
        self.proc = None

    def destroy_node(self):
        self._stop_pipeline()
        super().destroy_node()


def main():
    rclpy.init()
    node = SimCameraUdpStream()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
