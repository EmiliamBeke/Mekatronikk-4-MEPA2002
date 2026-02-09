import os
import shlex
import subprocess
import threading
import time

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

        self.model = YOLO(self.model_path, task="detect")
        self.pub = self.create_publisher(String, "/teddy_detector/status", 10)
        self.last = None
        self.proc = None
        self.frame_bytes = self.width * self.height * 3
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

            data = self.proc.stdout.read(self.frame_bytes) if self.proc.stdout else b""
            if len(data) != self.frame_bytes:
                self._warn_throttled("failed to read frame")
                if self.proc is not None:
                    self.proc.terminate()
                self.proc = None
                time.sleep(0.1)
                continue

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
        self._stop = True
        if self.proc is not None:
            self.proc.terminate()
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
