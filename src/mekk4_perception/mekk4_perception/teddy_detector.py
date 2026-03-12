import os
import shlex
import subprocess
import threading
import time

import cv2
import numpy as np
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
        self.width = int(os.environ.get("MEKK4_CAM_WIDTH", "1296"))
        self.height = int(os.environ.get("MEKK4_CAM_HEIGHT", "972"))
        self.conf = float(os.environ.get("MEKK4_CONF", "0.25"))
        self.imgsz = int(os.environ.get("MEKK4_IMGSZ", "640"))
        self.show_gui = os.environ.get("MEKK4_SHOW", "0").strip() == "1"
        self.center_tol = float(os.environ.get("MEKK4_CENTER_TOL", "0.1"))

        self.model = YOLO(self.model_path, task="detect")
        self.pub = self.create_publisher(String, "/teddy_detector/status", 10)
        self.last = None
        self.proc = None
        self.frame_bytes = self.width * self.height * 3
        self._buf = bytearray()
        self._last_warn = 0.0
        self._stop = False

        if not self.gst_source:
            self.get_logger().error("MEKK4_CAM_SOURCE_GST is required for UDP input")
            return

        self.get_logger().info(f"GStreamer source: {self.gst_source}")
        self.worker = threading.Thread(target=self._gst_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            "conf={conf} imgsz={imgsz}".format(
                conf=self.conf,
                imgsz=self.imgsz,
            )
        )
        if self.show_gui:
            self.get_logger().info("GUI enabled (MEKK4_SHOW=1)")

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

                self._buf.clear()

            chunk = self.proc.stdout.read(4096) if self.proc.stdout else b""
            if not chunk:
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
                self._infer_frame(frame)

    def _infer_frame(self, frame):
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf,
            classes=[TEDDY_CLASS_ID],
            verbose=False,
        )
        count = 0
        dx = None
        dy = None
        centered = False
        best_box = None

        if results:
            r = results[0]
            boxes = [] if r.boxes is None else r.boxes
            count = len(boxes)
            if count > 0:
                # pick largest box for center guidance
                best = None
                best_area = -1.0
                for b in boxes:
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    area = (x2 - x1) * (y2 - y1)
                    if area > best_area:
                        best_area = area
                        best = (x1, y1, x2, y2)
                if best is not None:
                    x1, y1, x2, y2 = best
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    dx = (cx - (self.width / 2.0)) / (self.width / 2.0)
                    dy = (cy - (self.height / 2.0)) / (self.height / 2.0)
                    centered = abs(dx) <= self.center_tol and abs(dy) <= self.center_tol
                    best_box = (int(x1), int(y1), int(x2), int(y2))

        msg = String()
        if dx is None or dy is None:
            msg.data = f"teddy_count={count} centered=false"
        else:
            msg.data = f"teddy_count={count} dx={dx:.3f} dy={dy:.3f} centered={str(centered).lower()}"
        self.pub.publish(msg)
        if msg.data != self.last:
            self.get_logger().info(msg.data)
            self.last = msg.data

        if self.show_gui:
            view = frame.copy()
            if best_box is not None:
                x1, y1, x2, y2 = best_box
                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(view, (self.width // 2, self.height // 2), 4, (0, 0, 255), -1)
            cv2.imshow("teddy_detector", view)
            cv2.waitKey(1)

    def destroy_node(self):
        self._stop = True
        if self.proc is not None:
            self.proc.terminate()
        if self.show_gui:
            cv2.destroyAllWindows()
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
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except Exception:
            return None

def main():
    rclpy.init()
    node = TeddyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
