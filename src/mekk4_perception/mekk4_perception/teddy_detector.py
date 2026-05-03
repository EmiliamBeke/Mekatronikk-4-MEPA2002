import os
import shlex
import subprocess
import threading
import time
from contextlib import suppress

import cv2
import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

TEDDY_CLASS_ID = 77  # COCO teddy bear


class TeddyDetector(Node):
    def __init__(self):
        super().__init__("teddy_detector")

        # Runtime setup comes from pi_bringup.sh / camera_params.yaml.
        self.model_path = os.environ.get("MEKK4_NCNN_MODEL", "/ws/models/yolo26n_ncnn_model")
        self.gst_source = os.environ.get("MEKK4_CAM_SOURCE_GST", "").strip()
        self.width = int(os.environ.get("MEKK4_CAM_WIDTH", "1296"))
        self.height = int(os.environ.get("MEKK4_CAM_HEIGHT", "972"))
        self.camera_fps = self._parse_positive_float(os.environ.get("MEKK4_CAM_FPS", "15"), default=15.0)
        self.conf = float(os.environ.get("MEKK4_CONF", "0.25"))
        self.imgsz = int(os.environ.get("MEKK4_IMGSZ", "640"))
        self.show_gui = os.environ.get("MEKK4_SHOW", "0").strip() == "1"
        self.center_tol = float(os.environ.get("MEKK4_CENTER_TOL", "0.1"))
        self.status_log_period_sec = float(os.environ.get("MEKK4_STATUS_LOG_PERIOD_SEC", "10.0"))
        self.stream_debug_video = os.environ.get("MEKK4_DEBUG_STREAM", "0").strip() == "1"
        self.debug_stream_host = os.environ.get("MEKK4_DEBUG_STREAM_HOST", "").strip()
        self.debug_stream_port = int(os.environ.get("MEKK4_DEBUG_STREAM_PORT", "5602"))
        self.debug_stream_scale = float(os.environ.get("MEKK4_DEBUG_STREAM_SCALE", "1.0"))
        self.debug_stream_fps = self._parse_stream_fps(os.environ.get("MEKK4_DEBUG_STREAM_FPS", "10.0"))
        self.debug_stream_bitrate_bps = int(os.environ.get("MEKK4_DEBUG_STREAM_BITRATE", "800000"))
        self.debug_stream_encoder = os.environ.get("MEKK4_DEBUG_STREAM_ENCODER", "x264").strip().lower()

        self.model = YOLO(self.model_path, task="detect")
        self.pub = self.create_publisher(String, "/teddy_detector/status", 10)

        # One-slot frame buffer: the camera reader overwrites old frames, YOLO uses newest.
        self.proc = None
        self.debug_stream_proc = None
        self.frame_bytes = self.width * self.height * 3
        self._buf = bytearray()
        self._frame_cond = threading.Condition()
        self._latest_frame = None
        self._latest_seq = 0
        self._last_warn = 0.0
        self._last_debug_stream = 0.0
        self._last_debug_stream_cmd = ""
        self._last_status_log = 0.0
        self._last_infer_end = None
        self._infer_fps = 0.0
        self._stop = False

        if not self.gst_source:
            self.get_logger().error("MEKK4_CAM_SOURCE_GST is required for UDP input")
            return

        # Two threads keep latency down: drain GStreamer continuously, infer separately.
        self.get_logger().info(f"GStreamer source: {self.gst_source}")
        self.worker = threading.Thread(target=self._gst_loop, daemon=True)
        self.worker.start()
        self.infer_worker = threading.Thread(target=self._infer_loop, daemon=True)
        self.infer_worker.start()

        self.get_logger().info(f"conf={self.conf} imgsz={self.imgsz}")
        if self.stream_debug_video and self.debug_stream_host:
            fps_label = "auto(detector-limited)" if self.debug_stream_fps is None else f"{self.debug_stream_fps}"
            self.get_logger().info(
                f"debug stream -> udp://{self.debug_stream_host}:{self.debug_stream_port} "
                f"scale={self.debug_stream_scale} fps={fps_label} bitrate={self.debug_stream_bitrate_bps}"
            )
        elif self.stream_debug_video:
            self.get_logger().warning("debug stream enabled, but no MEKK4_DEBUG_STREAM_HOST is set")
        if self.show_gui:
            self.get_logger().info("GUI enabled (MEKK4_SHOW=1)")

    def _warn_throttled(self, message: str, interval_sec: float = 5.0):
        if self._stop or not rclpy.ok():
            return
        now = time.monotonic()
        if now - self._last_warn >= interval_sec:
            self.get_logger().warning(message)
            self._last_warn = now

    @staticmethod
    def _parse_positive_float(value, *, default):
        try:
            parsed = float(str(value).strip())
        except Exception:
            return default
        return parsed if parsed > 0.0 else default

    @staticmethod
    def _parse_stream_fps(value):
        text = str(value).strip().lower()
        if text in {"", "0", "auto", "detector", "yolo", "none", "off"}:
            return None
        try:
            parsed = float(text)
        except Exception:
            return 10.0
        return parsed if parsed > 0.0 else None

    def _gst_loop(self):
        """Decode incoming H264/RTP to raw BGR frames and keep only the newest."""
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
                if self._stop or not rclpy.ok():
                    break
                self._warn_throttled("failed to read frame")
                self._stop_input_stream()
                self.proc = None
                time.sleep(0.1)
                continue

            self._buf.extend(chunk)
            while len(self._buf) >= self.frame_bytes:
                data = bytes(self._buf[: self.frame_bytes])
                del self._buf[: self.frame_bytes]
                frame = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3))
                with self._frame_cond:
                    self._latest_frame = frame
                    self._latest_seq += 1
                    self._frame_cond.notify()

    def _infer_loop(self):
        """Run YOLO on the newest frame available; older frames are intentionally skipped."""
        seen_seq = 0
        while not self._stop:
            with self._frame_cond:
                self._frame_cond.wait_for(lambda: self._stop or self._latest_seq != seen_seq, timeout=0.5)
                if self._stop:
                    return
                if self._latest_seq == seen_seq:
                    continue
                frame = self._latest_frame
                seen_seq = self._latest_seq

            if frame is not None:
                self._infer_frame(frame)

    def _infer_frame(self, frame):
        if self._stop or not rclpy.ok():
            return

        count, debug_boxes, best_box = self._detect_teddy(frame)
        dx, dy, centered = self._box_center_state(best_box)
        infer_end = time.monotonic()
        fps_text = self._update_inference_fps(infer_end)
        if not self._publish_status(count, dx, dy, centered, fps_text):
            return

        if self.show_gui or self.stream_debug_video:
            annotated = self._render_debug_view(frame, debug_boxes, best_box, centered, fps_text)
            if self.stream_debug_video:
                self._stream_debug_video(annotated)
            if self.show_gui:
                cv2.imshow("teddy_detector", annotated)
                cv2.waitKey(1)

    def _detect_teddy(self, frame):
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf,
            classes=[TEDDY_CLASS_ID],
            verbose=False,
        )
        boxes = [] if not results or results[0].boxes is None else results[0].boxes
        debug_boxes = []
        best_box = None
        best_area = -1.0

        # Largest detection is used for centering; all detections are drawn.
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0]) if box.conf is not None else 0.0
            debug_boxes.append((int(x1), int(y1), int(x2), int(y2), conf))
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area = area
                best_box = (int(x1), int(y1), int(x2), int(y2))

        return len(boxes), debug_boxes, best_box

    def _box_center_state(self, box):
        if box is None:
            return None, None, False

        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        dx = (cx - (self.width / 2.0)) / (self.width / 2.0)
        dy = (cy - (self.height / 2.0)) / (self.height / 2.0)
        return dx, dy, abs(dx) <= self.center_tol and abs(dy) <= self.center_tol

    def _publish_status(self, count, dx, dy, centered, fps_text):
        if dx is None or dy is None:
            status = f"teddy_count={count} centered=false"
        else:
            state = str(centered).lower()
            status = f"teddy_count={count} dx={dx:.3f} dy={dy:.3f} centered={state}"

        msg = String()
        msg.data = f"{status} fps={fps_text}"
        try:
            self.pub.publish(msg)
        except _rclpy.RCLError:
            return False

        if self._should_log_status() and not self._stop and rclpy.ok():
            self.get_logger().info(msg.data)
        return True

    def _render_debug_view(self, frame, debug_boxes, best_box, centered, fps_text):
        view = frame.copy()
        for x1, y1, x2, y2, conf in debug_boxes:
            color = (255, 200, 0)
            thickness = 2
            if best_box == (x1, y1, x2, y2):
                color = (0, 255, 0) if centered else (0, 200, 255)
                thickness = 3
            cv2.rectangle(view, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(
                view,
                f"teddy {conf:.2f}",
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        cv2.circle(view, (self.width // 2, self.height // 2), 4, (0, 0, 255), -1)
        self._draw_overlay_label(view, f"YOLO {fps_text} FPS", (12, 28))
        return view

    def _update_inference_fps(self, infer_end):
        if self._last_infer_end is None:
            self._last_infer_end = infer_end
            self._infer_fps = 0.0
            return "--"

        dt = infer_end - self._last_infer_end
        self._last_infer_end = infer_end
        if dt <= 0.0:
            return "--" if self._infer_fps <= 0.0 else f"{self._infer_fps:.1f}"

        inst_fps = 1.0 / dt
        if self._infer_fps <= 0.0:
            self._infer_fps = inst_fps
        else:
            self._infer_fps = (0.8 * self._infer_fps) + (0.2 * inst_fps)
        return f"{self._infer_fps:.1f}"

    def _should_log_status(self):
        if self.status_log_period_sec <= 0.0:
            return False

        now = time.monotonic()
        if now - self._last_status_log >= self.status_log_period_sec:
            self._last_status_log = now
            return True
        return False

    @staticmethod
    def _draw_overlay_label(image, text, origin):
        x, y = origin
        for color, thickness in [((0, 0, 0), 4), ((255, 255, 255), 2)]:
            cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, thickness, cv2.LINE_AA)

    def _stream_debug_video(self, annotated):
        """Encode the annotated BGR frame as low-latency H264/RTP to the PC."""
        if not self.debug_stream_host:
            return
        if self.debug_stream_fps is not None:
            min_period = 1.0 / self.debug_stream_fps
            now = time.monotonic()
            if now - self._last_debug_stream < min_period:
                return
            self._last_debug_stream = now

        frame = annotated
        if self.debug_stream_scale > 0.0 and self.debug_stream_scale != 1.0:
            new_width = max(1, int(round(self.width * self.debug_stream_scale)))
            new_height = max(1, int(round(self.height * self.debug_stream_scale)))
            frame = cv2.resize(annotated, (new_width, new_height), interpolation=cv2.INTER_AREA)

        proc = self._ensure_debug_stream_process(frame.shape[1], frame.shape[0])
        if proc is None or proc.stdin is None:
            return
        try:
            proc.stdin.write(frame.tobytes())
            proc.stdin.flush()
        except Exception:
            self._stop_debug_stream()

    def _ensure_debug_stream_process(self, width, height):
        if not self.debug_stream_host:
            return None
        if self.debug_stream_proc is not None and self.debug_stream_proc.poll() is None:
            return self.debug_stream_proc

        if self.debug_stream_proc is not None:
            self._warn_throttled(
                f"debug stream process exited with code {self.debug_stream_proc.poll()} "
                f"(encoder={self.debug_stream_encoder})"
            )
        self._stop_debug_stream()
        fps_hint = self.debug_stream_fps if self.debug_stream_fps is not None else self.camera_fps
        fps = max(1, int(round(fps_hint)))
        bitrate_kbps = max(100, int(round(self.debug_stream_bitrate_bps / 1000.0)))
        key_int = max(1, fps)
        encoder = self._debug_stream_encoder(bitrate_kbps, key_int)
        pipeline = (
            "fdsrc ! "
            f"videoparse format=bgr width={width} height={height} framerate={fps}/1 ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "videoconvert ! "
            f"{encoder} ! "
            "h264parse ! "
            "rtph264pay pt=96 config-interval=1 ! "
            f"udpsink host={self.debug_stream_host} port={self.debug_stream_port} sync=false async=false"
        )
        cmd = ["gst-launch-1.0", "-q"] + shlex.split(pipeline)
        cmd_text = " ".join(cmd)
        if cmd_text != self._last_debug_stream_cmd:
            self.get_logger().info(f"debug stream pipeline: {cmd_text}")
            self._last_debug_stream_cmd = cmd_text
        try:
            self.debug_stream_proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except Exception:
            self.debug_stream_proc = None
            return None
        return self.debug_stream_proc

    def _debug_stream_encoder(self, bitrate_kbps, key_int):
        if self.debug_stream_encoder == "openh264":
            return (
                "video/x-raw,format=I420 ! "
                f"openh264enc bitrate={bitrate_kbps * 1000} "
                f"gop-size={key_int} rate-control=bitrate complexity=low"
            )
        return f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} key-int-max={key_int}"

    def _stop_debug_stream(self):
        if self.debug_stream_proc is None:
            return
        with suppress(Exception):
            if self.debug_stream_proc.stdin is not None:
                self.debug_stream_proc.stdin.close()
        self.debug_stream_proc.terminate()
        with suppress(Exception):
            self.debug_stream_proc.wait(timeout=1.0)
        self.debug_stream_proc = None

    def _stop_input_stream(self):
        if self.proc is None:
            return
        self.proc.terminate()
        with suppress(Exception):
            self.proc.wait(timeout=1.0)
        self.proc = None

    def destroy_node(self):
        self._stop = True
        self._stop_input_stream()
        self._stop_debug_stream()
        with self._frame_cond:
            self._frame_cond.notify_all()
        for worker in (getattr(self, "worker", None), getattr(self, "infer_worker", None)):
            if worker is not None and worker.is_alive():
                worker.join(timeout=1.0)
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()

    def _start_gst_process(self):
        """Start the input decoder configured by MEKK4_CAM_SOURCE_GST."""
        pipeline = self.gst_source.replace(", ", ",")
        sink = (
            f"video/x-raw,format=BGR,width={self.width},height={self.height} "
            "! queue leaky=downstream max-size-buffers=1 "
            "! fdsink fd=1"
        )
        if "appsink" in pipeline:
            pipeline = pipeline.split("appsink")[0].rstrip(" !")
        pipeline = f"{pipeline} ! {sink}"
        cmd = ["gst-launch-1.0", "-q"] + shlex.split(pipeline)
        try:
            return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        except Exception:
            return None

def main():
    rclpy.init()
    node = TeddyDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
