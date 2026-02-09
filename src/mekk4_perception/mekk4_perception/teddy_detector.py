import os
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

TEDDY_CLASS_ID = 77  # COCO teddy bear


class TeddyDetector(Node):
    def __init__(self):
        super().__init__("teddy_detector")

        self.model_path = os.environ.get("MEKK4_NCNN_MODEL", "/ws/models/yolo26n_ncnn_model")
        self.gst_source = os.environ.get("MEKK4_CAM_SOURCE_GST", "").strip()
        self.conf = float(os.environ.get("MEKK4_CONF", "0.25"))
        self.imgsz = int(os.environ.get("MEKK4_IMGSZ", "640"))
        self.max_fps = float(os.environ.get("MEKK4_MAX_FPS", "0"))

        self.model = YOLO(self.model_path, task="detect")
        self.pub = self.create_publisher(String, "/teddy_detector/status", 10)
        self.last = None
        self.cap = None
        self._last_warn = 0.0

        if not self.gst_source:
            self.get_logger().error("MEKK4_CAM_SOURCE_GST is required for UDP input")
            self._start_timer()
            return

        self.get_logger().info(f"GStreamer source: {self.gst_source}")
        self.cap = cv2.VideoCapture(self.gst_source, cv2.CAP_GSTREAMER)
        self._start_timer()

        self.get_logger().info(
            "conf={conf} imgsz={imgsz} max_fps={max_fps}".format(
                conf=self.conf,
                imgsz=self.imgsz,
                max_fps=self.max_fps,
            )
        )

    def _start_timer(self):
        period = 1.0 / self.max_fps if self.max_fps > 0 else 0.1
        self.create_timer(period, self.on_timer)

    def _warn_throttled(self, message: str, interval_sec: float = 5.0):
        now = time.monotonic()
        if now - self._last_warn >= interval_sec:
            self.get_logger().warning(message)
            self._last_warn = now

    def on_timer(self):
        if self.cap is None or not self.cap.isOpened():
            self._warn_throttled("gstreamer source not available")
            return

        ok, frame = self.cap.read()
        if not ok:
            self._warn_throttled("failed to read frame")
            return

        self._infer_frame(frame)

    def _infer_frame(self, frame):
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf,
            classes=[TEDDY_CLASS_ID],
            verbose=False,
        )
        if results:
            r = results[0]
            count = 0 if r.boxes is None else len(r.boxes)
        else:
            count = 0

        msg = String()
        msg.data = f"teddy_count={count}"
        self.pub.publish(msg)
        if msg.data != self.last:
            self.get_logger().info(msg.data)
            self.last = msg.data

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = TeddyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
