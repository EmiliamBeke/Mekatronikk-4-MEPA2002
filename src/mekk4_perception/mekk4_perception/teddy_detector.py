import os
import threading
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

TEDDY_CLASS_ID = 77  # COCO teddy bear

class TeddyDetector(Node):
    def __init__(self):
        super().__init__("teddy_detector")

        self.model_path = os.environ.get("MEKK4_NCNN_MODEL", "/ws/models/yolo26n_ncnn_model")
        self.source = os.environ.get("MEKK4_CAM_SOURCE", "").strip()
        self.image_topic = os.environ.get("MEKK4_CAM_TOPIC", "/camera/image_raw")
        self.conf = float(os.environ.get("MEKK4_CONF", "0.25"))
        self.imgsz = int(os.environ.get("MEKK4_IMGSZ", "640"))
        self.vid_stride = int(os.environ.get("MEKK4_VID_STRIDE", "1"))
        self.max_fps = float(os.environ.get("MEKK4_MAX_FPS", "0"))

        self.get_logger().info(f"Model:  {self.model_path}")
        if self.source:
            self.get_logger().info(f"Source: {self.source}")
        else:
            self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(
            "conf={conf} imgsz={imgsz} vid_stride={vid_stride} max_fps={max_fps}".format(
                conf=self.conf,
                imgsz=self.imgsz,
                vid_stride=self.vid_stride,
                max_fps=self.max_fps,
            )
        )

        # Explicit task to remove warning
        self.model = YOLO(self.model_path, task="detect")

        self.pub = self.create_publisher(String, "/teddy_detector/status", 10)
        self.last = None
        self.bridge = CvBridge()

        self._stop = False
        self._frame_count = 0
        self._last_process_time = 0.0
        self._lock = threading.Lock()
        self._latest = None
        self._event = threading.Event()

        if self.source:
            self.worker = threading.Thread(target=self.stream_loop, daemon=True)
        else:
            self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
            self.worker = threading.Thread(target=self.image_loop, daemon=True)
        self.worker.start()

    def on_image(self, msg: Image):
        with self._lock:
            self._latest = msg
        self._event.set()

    def _should_process(self) -> bool:
        self._frame_count += 1
        if self.vid_stride > 1 and (self._frame_count - 1) % self.vid_stride != 0:
            return False
        if self.max_fps > 0:
            now = time.monotonic()
            min_dt = 1.0 / self.max_fps
            if now - self._last_process_time < min_dt:
                return False
            self._last_process_time = now
        return True

    def _should_process_time(self) -> bool:
        if self.max_fps > 0:
            now = time.monotonic()
            min_dt = 1.0 / self.max_fps
            if now - self._last_process_time < min_dt:
                return False
            self._last_process_time = now
        return True

    def _publish_count(self, count: int):
        msg = String()
        msg.data = f"teddy_count={count}"
        self.pub.publish(msg)
        if msg.data != self.last:
            self.get_logger().info(msg.data)
            self.last = msg.data

    def image_loop(self):
        try:
            while not self._stop:
                if not self._event.wait(0.1):
                    continue
                self._event.clear()
                with self._lock:
                    msg = self._latest
                if msg is None:
                    continue
                if not self._should_process():
                    continue
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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
                self._publish_count(count)
        except Exception as e:
            self.get_logger().error(f"inference loop crashed: {e}")

    def stream_loop(self):
        # stream=True => generator, no RAM accumulation
        try:
            results = self.model.predict(
                source=self.source,
                stream=True,
                imgsz=self.imgsz,
                conf=self.conf,
                classes=[TEDDY_CLASS_ID],
                vid_stride=self.vid_stride,
                verbose=False,
            )
            for r in results:
                if self._stop:
                    break
                if not self._should_process_time():
                    continue
                count = 0 if r.boxes is None else len(r.boxes)
                self._publish_count(count)
        except Exception as e:
            self.get_logger().error(f"inference loop crashed: {e}")

    def destroy_node(self):
        self._stop = True
        self._event.set()
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
