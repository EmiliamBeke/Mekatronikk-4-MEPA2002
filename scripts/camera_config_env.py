#!/usr/bin/env python3
import os
import shlex
import sys

try:
    import yaml
except Exception as exc:
    print(
        f"echo '[camera-config] Missing python yaml support: {exc}. Install python3-yaml.' >&2",
        file=sys.stdout,
    )
    print("exit 1", file=sys.stdout)
    sys.exit(0)


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "camera_params.yaml")


def to_shell(value):
    if isinstance(value, bool):
        return "1" if value else "0"
    return str(value)


def pick(env_name, default):
    value = os.environ.get(env_name)
    if value is not None and value != "":
        return value
    return default


def normalize_choice(value, *, false_value=None, true_value=None):
    if isinstance(value, bool):
        if value and true_value is not None:
            return true_value
        if not value and false_value is not None:
            return false_value
    return value


def scaled_size(size_value, scale_value):
    try:
        size = int(size_value)
        scale = float(scale_value)
    except Exception:
        return size_value
    return max(1, int(round(size * scale)))


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("CAMERA_CONFIG_FILE", DEFAULT_CONFIG)
    if not os.path.exists(config_path):
        print(f"echo '[camera-config] Config file not found: {config_path}' >&2", file=sys.stdout)
        print("exit 1", file=sys.stdout)
        return

    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    stream = data.get("camera_stream", {})
    detector = data.get("teddy_detector", {})

    denoise = normalize_choice(stream.get("denoise", "auto"), false_value="off", true_value="auto")
    debug_stream_scale = detector.get("debug_stream_scale", 1.0)

    values = {
        "CAMERA_CONFIG_FILE": config_path,
        "WIDTH": pick("WIDTH", stream.get("width", 1296)),
        "HEIGHT": pick("HEIGHT", stream.get("height", 972)),
        "FPS": pick("FPS", stream.get("fps", 15)),
        "BITRATE": pick("BITRATE", stream.get("bitrate_bps", "")),
        "INTRA": pick("INTRA", stream.get("intra", "")),
        "LOW_LATENCY": pick("LOW_LATENCY", to_shell(stream.get("low_latency", False))),
        "FLUSH_OUTPUT": pick("FLUSH_OUTPUT", to_shell(stream.get("flush_output", True))),
        "PC_JITTER_MS": pick("PC_JITTER_MS", stream.get("pc_jitter_ms", 40)),
        "CAM_PORT": pick("CAM_PORT", stream.get("local_udp_port", 5600)),
        "PORT": pick("PORT", detector.get("debug_stream_port", 5602)),
        "AWB": pick("AWB", stream.get("awb", "auto")),
        "AWB_GAINS": pick("AWB_GAINS", stream.get("awb_gains", "")),
        "BRIGHTNESS": pick("BRIGHTNESS", stream.get("brightness", 0.0)),
        "CONTRAST": pick("CONTRAST", stream.get("contrast", 1.0)),
        "SATURATION": pick("SATURATION", stream.get("saturation", 1.0)),
        "SHARPNESS": pick("SHARPNESS", stream.get("sharpness", 1.0)),
        "EV": pick("EV", stream.get("ev", 0.0)),
        "DENOISE": pick("DENOISE", denoise),
        "METERING": pick("METERING", stream.get("metering", "centre")),
        "TUNING_FILE": pick("TUNING_FILE", stream.get("tuning_file", "")),
        "MEKK4_CAM_WIDTH": pick("MEKK4_CAM_WIDTH", stream.get("width", 1296)),
        "MEKK4_CAM_HEIGHT": pick("MEKK4_CAM_HEIGHT", stream.get("height", 972)),
        "MEKK4_CAM_FPS": pick("MEKK4_CAM_FPS", stream.get("fps", 15)),
        "MEKK4_NCNN_MODEL": pick("MEKK4_NCNN_MODEL", detector.get("model_path", "/ws/models/yolo26n_ncnn_model")),
        "MEKK4_CONF": pick("MEKK4_CONF", detector.get("conf", 0.25)),
        "MEKK4_IMGSZ": pick("MEKK4_IMGSZ", detector.get("imgsz", 640)),
        "MEKK4_CENTER_TOL": pick("MEKK4_CENTER_TOL", detector.get("center_tol", 0.10)),
        "MEKK4_SHOW": pick("MEKK4_SHOW", to_shell(detector.get("show_gui", False))),
        "MEKK4_DEBUG_STREAM": pick("MEKK4_DEBUG_STREAM", to_shell(detector.get("stream_debug_video", True))),
        "MEKK4_DEBUG_STREAM_PORT": pick("MEKK4_DEBUG_STREAM_PORT", detector.get("debug_stream_port", 5602)),
        "MEKK4_DEBUG_STREAM_SCALE": pick("MEKK4_DEBUG_STREAM_SCALE", debug_stream_scale),
        "MEKK4_DEBUG_STREAM_FPS": pick("MEKK4_DEBUG_STREAM_FPS", detector.get("debug_stream_fps", stream.get("fps", 15))),
        "MEKK4_DEBUG_STREAM_BITRATE": pick("MEKK4_DEBUG_STREAM_BITRATE", detector.get("debug_stream_bitrate_bps", 800000)),
        "MEKK4_DEBUG_STREAM_WIDTH": pick("MEKK4_DEBUG_STREAM_WIDTH", scaled_size(stream.get("width", 1296), debug_stream_scale)),
        "MEKK4_DEBUG_STREAM_HEIGHT": pick("MEKK4_DEBUG_STREAM_HEIGHT", scaled_size(stream.get("height", 972), debug_stream_scale)),
    }

    for key, value in values.items():
        print(f"export {key}={shlex.quote(to_shell(value))}")


if __name__ == "__main__":
    main()
