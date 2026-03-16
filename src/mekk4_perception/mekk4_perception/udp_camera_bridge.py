import os
import shlex
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class UdpCameraBridge(Node):
    def __init__(self):
        super().__init__("udp_camera_bridge")

        self.declare_parameter("gst_source", os.environ.get("MEKK4_CAM_SOURCE_GST", "").strip())
        self.declare_parameter("width", int(os.environ.get("MEKK4_CAM_WIDTH", "1296")))
        self.declare_parameter("height", int(os.environ.get("MEKK4_CAM_HEIGHT", "972")))
        self.declare_parameter("topic_name", "/camera")
        self.declare_parameter("frame_id", "camera_link")

        self.gst_source = self.get_parameter("gst_source").get_parameter_value().string_value.strip()
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.proc = None
        self.frame_bytes = self.width * self.height * 3
        self._buf = bytearray()
        self._stderr_lines = deque(maxlen=20)
        self._last_warn = 0.0
        self._stop = False

        if not self.gst_source:
            self.get_logger().error("gst_source is required")
            return

        self.get_logger().info(f"GStreamer source: {self.gst_source}")
        self.worker = threading.Thread(target=self._gst_loop, daemon=True)
        self.worker.start()

    def _warn_throttled(self, message: str, interval_sec: float = 5.0):
        now = time.monotonic()
        if now - self._last_warn >= interval_sec:
            self.get_logger().warning(message)
            self._last_warn = now

    def _gst_loop(self):
        while not self._stop:
            if self.proc is None or self.proc.poll() is not None:
                self.proc = self._start_gst_process()
                if self.proc is None:
                    self._warn_throttled("gstreamer source not available")
                    time.sleep(1.0)
                    continue
                if self.proc.stderr is not None:
                    threading.Thread(target=self._drain_stderr, daemon=True).start()

                self._buf.clear()
                self._stderr_lines.clear()

            chunk = self.proc.stdout.read(4096) if self.proc.stdout else b""
            if not chunk:
                if self._stop:
                    break
                if self.proc is not None and self.proc.poll() is not None:
                    self._log_gst_failure()
                else:
                    self._warn_throttled("failed to read frame")
                if self.proc is not None:
                    self.proc.terminate()
                self.proc = None
                time.sleep(0.1)
                continue

            self._buf.extend(chunk)
            while len(self._buf) >= self.frame_bytes:
                data = bytes(self._buf[: self.frame_bytes])
                del self._buf[: self.frame_bytes]
                frame = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3))
                self._publish_frame(frame)

    def _publish_frame(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _drain_stderr(self):
        if self.proc is None or self.proc.stderr is None:
            return

        while not self._stop:
            line = self.proc.stderr.readline()
            if not line:
                return
            self._stderr_lines.append(line.decode(errors="replace").rstrip())

    def _log_gst_failure(self):
        if self.proc is None:
            return

        code = self.proc.poll()
        if self._stderr_lines:
            message = "; ".join(self._stderr_lines)
            self._warn_throttled(f"gstreamer exited with code {code}: {message}", interval_sec=2.0)
            return

        self._warn_throttled(f"gstreamer exited with code {code}", interval_sec=2.0)

    def destroy_node(self):
        self._stop = True
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1.0)
            except Exception:
                pass
            self.proc = None
        worker = getattr(self, "worker", None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        super().destroy_node()

    def _start_gst_process(self):
        pipeline = self.gst_source.replace(", ", ",")
        sink = (
            f"video/x-raw,format=BGR,width={self.width},height={self.height} "
            "! fdsink fd=1"
        )
        if "appsink" in pipeline:
            pipeline = pipeline.split("appsink")[0].rstrip(" !")
        pipeline = f"{pipeline} ! {sink}"
        cmd = ["gst-launch-1.0", "-q"] + shlex.split(pipeline)
        try:
            return subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except Exception:
            return None


def main():
    rclpy.init()
    node = UdpCameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
