#!/usr/bin/env python3
import argparse
import math
import os
import sys
import time

import serial

try:
    import yaml
except Exception:
    yaml = None


IGNORED_PREFIXES = ("EVENT ", "MEGA_KEYBOARD_READY")
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "robot_calibration.yaml")


def read_line(ser: serial.Serial, timeout: float, ignored_prefixes: tuple[str, ...] = ()) -> str | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            continue
        if any(text.startswith(prefix) for prefix in ignored_prefixes):
            print(f"[mega-cal] .. {text}")
            continue
        return text
    return None


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def expect_reply(
    ser: serial.Serial,
    command: str,
    expected_prefix: str,
    timeout: float,
    ignored_prefixes: tuple[str, ...] = IGNORED_PREFIXES,
) -> str:
    print(f"[mega-cal] -> {command}")
    send_command(ser, command)
    reply = read_line(ser, timeout, ignored_prefixes=ignored_prefixes)
    if reply is None:
        raise RuntimeError(f"timeout waiting for reply to {command!r}")
    print(f"[mega-cal] <- {reply}")
    if not reply.startswith(expected_prefix):
        raise RuntimeError(
            f"unexpected reply to {command!r}: expected prefix {expected_prefix!r}, got {reply!r}"
        )
    return reply


def parse_encoder_reply(reply: str, label: str) -> int:
    parts = reply.split()
    if len(parts) != 2 or parts[0] != label:
        raise RuntimeError(f"failed to parse {label} reply: {reply!r}")
    try:
        return int(parts[1])
    except ValueError as exc:
        raise RuntimeError(f"failed to parse {label} count from {reply!r}") from exc


def read_encoders(ser: serial.Serial, timeout: float, swap_sides: bool = False) -> tuple[int, int]:
    left_reply = expect_reply(ser, "ENC1", "ENC1 ", timeout)
    right_reply = expect_reply(ser, "ENC2", "ENC2 ", timeout)
    first = parse_encoder_reply(left_reply, "ENC1")
    second = parse_encoder_reply(right_reply, "ENC2")
    if swap_sides:
        return second, first
    return first, second


def make_both_command(left_value: int, right_value: int, swap_sides: bool) -> str:
    if swap_sides:
        left_value, right_value = right_value, left_value
    return f"BOTH {left_value} {right_value}"


def scale_signed_pwm(value: int, scale: float) -> int:
    if scale <= 0.0:
        raise RuntimeError("left_cmd_scale and right_cmd_scale must be greater than zero")
    if value == 0:
        return 0

    scaled = int(round(abs(value) * scale))
    scaled = max(1, min(255, scaled))
    return scaled if value > 0 else -scaled


def make_scaled_both_command(
    left_value: int,
    right_value: int,
    left_cmd_scale: float,
    right_cmd_scale: float,
    swap_sides: bool,
) -> str:
    return make_both_command(
        scale_signed_pwm(left_value, left_cmd_scale),
        scale_signed_pwm(right_value, right_cmd_scale),
        swap_sides,
    )


def verify_keyboard_firmware(ser: serial.Serial, timeout: float) -> str:
    firmware = expect_reply(ser, "ID", "MEGA_", timeout)
    if firmware != "MEGA_KEYBOARD_DRIVE":
        raise RuntimeError(
            "mega_calibration.py expects mega_keyboard_drive firmware, "
            f"but board reported {firmware!r}"
        )
    expect_reply(ser, "PING", "PONG", timeout)
    expect_reply(ser, "STOP", "OK STOP", timeout)
    return firmware


def reset_encoders(ser: serial.Serial, timeout: float) -> None:
    expect_reply(ser, "RESET ENC1", "OK RESET ENC1", timeout)
    expect_reply(ser, "RESET ENC2", "OK RESET ENC2", timeout)


def drive_for(
    ser: serial.Serial,
    command: str,
    duration_s: float,
    send_period_s: float,
    settle_time_s: float,
    timeout: float,
) -> None:
    deadline = time.monotonic() + max(0.0, duration_s)
    while True:
        print(f"[mega-cal] -> {command}")
        send_command(ser, command)
        now = time.monotonic()
        if now >= deadline:
            break
        time.sleep(min(send_period_s, max(0.0, deadline - now)))

    expect_reply(ser, "STOP", "OK STOP", timeout)
    time.sleep(max(0.0, settle_time_s))


def print_encoder_summary(start_left: int, start_right: int, end_left: int, end_right: int) -> tuple[int, int]:
    delta_left = end_left - start_left
    delta_right = end_right - start_right
    print()
    print("[mega-cal] Encoder summary")
    print(f"[mega-cal]   start: left={start_left} right={start_right}")
    print(f"[mega-cal]   end:   left={end_left} right={end_right}")
    print(f"[mega-cal]   delta: left={delta_left} right={delta_right}")
    return delta_left, delta_right


def print_straight_calibration(distance_m: float, delta_left: int, delta_right: int) -> dict[str, float]:
    if delta_left == 0 or delta_right == 0:
        raise RuntimeError("encoder delta was zero; cannot compute meters per tick")

    left_m_per_tick = abs(distance_m / delta_left)
    right_m_per_tick = abs(distance_m / delta_right)
    mean_m_per_tick = (left_m_per_tick + right_m_per_tick) / 2.0

    print()
    print("[mega-cal] Straight calibration")
    print(f"[mega-cal]   distance_m={distance_m:.6f}")
    print(f"[mega-cal]   left_m_per_tick={left_m_per_tick:.9f}")
    print(f"[mega-cal]   right_m_per_tick={right_m_per_tick:.9f}")
    print(f"[mega-cal]   mean_m_per_tick={mean_m_per_tick:.9f}")
    print("[mega-cal] YAML snippet:")
    print(f"left_m_per_tick: {left_m_per_tick:.9f}")
    print(f"right_m_per_tick: {right_m_per_tick:.9f}")
    print(f"mean_m_per_tick: {mean_m_per_tick:.9f}")
    return {
        "left_m_per_tick": left_m_per_tick,
        "right_m_per_tick": right_m_per_tick,
        "mean_m_per_tick": mean_m_per_tick,
    }


def print_straight_bidir_calibration(
    forward_distance_m: float,
    reverse_distance_m: float,
    forward_delta_left: int,
    forward_delta_right: int,
    reverse_delta_left: int,
    reverse_delta_right: int,
) -> dict[str, float]:
    if forward_distance_m <= 0.0:
        raise RuntimeError("forward distance must be greater than zero")
    if reverse_distance_m <= 0.0:
        raise RuntimeError("reverse distance must be greater than zero")
    if (
        forward_delta_left == 0
        or forward_delta_right == 0
        or reverse_delta_left == 0
        or reverse_delta_right == 0
    ):
        raise RuntimeError("encoder delta was zero; cannot compute bidirectional meters per tick")

    forward_left_m_per_tick = abs(forward_distance_m / forward_delta_left)
    forward_right_m_per_tick = abs(forward_distance_m / forward_delta_right)
    reverse_left_m_per_tick = abs(reverse_distance_m / reverse_delta_left)
    reverse_right_m_per_tick = abs(reverse_distance_m / reverse_delta_right)

    left_m_per_tick = (forward_left_m_per_tick + reverse_left_m_per_tick) / 2.0
    right_m_per_tick = (forward_right_m_per_tick + reverse_right_m_per_tick) / 2.0

    def pct_diff(a: float, b: float) -> float:
        mean = (abs(a) + abs(b)) / 2.0
        if mean == 0.0:
            return 0.0
        return abs(a - b) / mean * 100.0

    print()
    print("[mega-cal] Bidirectional straight calibration")
    print(f"[mega-cal]   forward_distance_m={forward_distance_m:.6f}")
    print(f"[mega-cal]   reverse_distance_m={reverse_distance_m:.6f}")
    print("[mega-cal]   forward:")
    print(f"[mega-cal]     delta_left={forward_delta_left} delta_right={forward_delta_right}")
    print(f"[mega-cal]     left_m_per_tick={forward_left_m_per_tick:.9f}")
    print(f"[mega-cal]     right_m_per_tick={forward_right_m_per_tick:.9f}")
    print("[mega-cal]   reverse:")
    print(f"[mega-cal]     delta_left={reverse_delta_left} delta_right={reverse_delta_right}")
    print(f"[mega-cal]     left_m_per_tick={reverse_left_m_per_tick:.9f}")
    print(f"[mega-cal]     right_m_per_tick={reverse_right_m_per_tick:.9f}")
    print("[mega-cal]   averaged:")
    print(f"[mega-cal]     left_m_per_tick={left_m_per_tick:.9f}")
    print(f"[mega-cal]     right_m_per_tick={right_m_per_tick:.9f}")
    print(
        f"[mega-cal]   forward_reverse_delta_pct: "
        f"left={pct_diff(forward_left_m_per_tick, reverse_left_m_per_tick):.2f}% "
        f"right={pct_diff(forward_right_m_per_tick, reverse_right_m_per_tick):.2f}%"
    )
    print("[mega-cal] YAML snippet:")
    print(f"left_m_per_tick: {left_m_per_tick:.9f}")
    print(f"right_m_per_tick: {right_m_per_tick:.9f}")

    return {
        "left_m_per_tick": left_m_per_tick,
        "right_m_per_tick": right_m_per_tick,
    }


def print_spin_calibration(
    angle_deg: float,
    left_m_per_tick: float,
    right_m_per_tick: float,
    delta_left: int,
    delta_right: int,
) -> dict[str, float]:
    angle_rad = math.radians(abs(angle_deg))
    if angle_rad == 0.0:
        raise RuntimeError("angle_deg must be non-zero")

    left_distance = abs(delta_left) * left_m_per_tick
    right_distance = abs(delta_right) * right_m_per_tick
    track_width_eff_m = (left_distance + right_distance) / angle_rad

    print()
    print("[mega-cal] Spin calibration")
    print(f"[mega-cal]   angle_deg={angle_deg:.6f}")
    print(f"[mega-cal]   left_distance_m={left_distance:.6f}")
    print(f"[mega-cal]   right_distance_m={right_distance:.6f}")
    print(f"[mega-cal]   track_width_eff_m={track_width_eff_m:.9f}")
    print("[mega-cal] YAML snippet:")
    print(f"track_width_eff_m: {track_width_eff_m:.9f}")
    return {
        "track_width_eff_m": track_width_eff_m,
    }


def print_straight_trim_calibration(
    delta_left: int,
    delta_right: int,
    current_left_cmd_scale: float,
    current_right_cmd_scale: float,
    left_m_per_tick: float,
    right_m_per_tick: float,
    drift_direction: str,
) -> dict[str, float]:
    left_metric = abs(delta_left)
    right_metric = abs(delta_right)
    metric_label = "ticks"

    if left_m_per_tick > 0.0 and right_m_per_tick > 0.0:
        left_metric *= left_m_per_tick
        right_metric *= right_m_per_tick
        metric_label = "meters"

    if left_metric == 0.0 or right_metric == 0.0:
        raise RuntimeError("encoder delta was zero; cannot compute straight-trim suggestion")

    ratio = right_metric / left_metric if left_metric > right_metric else left_metric / right_metric
    suggested_left_cmd_scale = current_left_cmd_scale
    suggested_right_cmd_scale = current_right_cmd_scale
    faster_side = "balanced"
    correction_basis = "encoder_metric"

    if drift_direction == "right":
        faster_side = "left"
        suggested_left_cmd_scale = current_left_cmd_scale * ratio
        correction_basis = "observed_drift"
    elif drift_direction == "left":
        faster_side = "right"
        suggested_right_cmd_scale = current_right_cmd_scale * ratio
        correction_basis = "observed_drift"
    else:
        if left_metric > right_metric:
            faster_side = "left"
            suggested_left_cmd_scale = current_left_cmd_scale * ratio
        elif right_metric > left_metric:
            faster_side = "right"
            suggested_right_cmd_scale = current_right_cmd_scale * ratio

    print()
    print("[mega-cal] Straight trim")
    print(f"[mega-cal]   comparison_metric={metric_label}")
    print(f"[mega-cal]   left_metric={left_metric:.9f}")
    print(f"[mega-cal]   right_metric={right_metric:.9f}")
    print(f"[mega-cal]   observed_drift={drift_direction}")
    print(f"[mega-cal]   correction_basis={correction_basis}")
    print(f"[mega-cal]   faster_side={faster_side}")
    print(
        f"[mega-cal]   current_left_cmd_scale={current_left_cmd_scale:.9f}"
    )
    print(
        f"[mega-cal]   current_right_cmd_scale={current_right_cmd_scale:.9f}"
    )
    print(
        f"[mega-cal]   suggested_left_cmd_scale={suggested_left_cmd_scale:.9f}"
    )
    print(
        f"[mega-cal]   suggested_right_cmd_scale={suggested_right_cmd_scale:.9f}"
    )
    print("[mega-cal] YAML snippet:")
    print(f"left_cmd_scale: {suggested_left_cmd_scale:.9f}")
    print(f"right_cmd_scale: {suggested_right_cmd_scale:.9f}")
    return {
        "left_cmd_scale": suggested_left_cmd_scale,
        "right_cmd_scale": suggested_right_cmd_scale,
    }


def load_config(config_path: str) -> dict:
    if yaml is None:
        raise RuntimeError("Missing python yaml support. Install python3-yaml.")
    if not os.path.exists(config_path):
        return {}
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def save_config(config_path: str, data: dict) -> None:
    if yaml is None:
        raise RuntimeError("Missing python yaml support. Install python3-yaml.")
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    with open(config_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)


def maybe_update_config(args: argparse.Namespace, updates: dict[str, float] | None) -> None:
    if not getattr(args, "write_config", True) or not updates:
        return

    config_path = args.config_file
    data = load_config(config_path)
    mega = data.setdefault("mega_driver", {})

    if "swap_sides" not in mega:
        mega["swap_sides"] = bool(args.swap_sides)
    if "left_cmd_sign" not in mega:
        mega["left_cmd_sign"] = 1
    if "right_cmd_sign" not in mega:
        mega["right_cmd_sign"] = 1
    if "angular_cmd_sign" not in mega:
        mega["angular_cmd_sign"] = 1
    if "left_tick_sign" not in mega:
        mega["left_tick_sign"] = 1
    if "right_tick_sign" not in mega:
        mega["right_tick_sign"] = 1

    if args.mode in ("straight", "straight-bidir", "straight-trim", "spin"):
        mega["swap_sides"] = bool(args.swap_sides)
        mega["left_cmd_scale"] = float(args.left_cmd_scale)
        mega["right_cmd_scale"] = float(args.right_cmd_scale)

    if "left_cmd_scale" in updates:
        mega["left_cmd_scale"] = float(updates["left_cmd_scale"])
    if "right_cmd_scale" in updates:
        mega["right_cmd_scale"] = float(updates["right_cmd_scale"])
    if "left_m_per_tick" in updates:
        mega["left_m_per_tick"] = float(updates["left_m_per_tick"])
    if "right_m_per_tick" in updates:
        mega["right_m_per_tick"] = float(updates["right_m_per_tick"])
    if "track_width_eff_m" in updates:
        mega["track_width_eff_m"] = float(updates["track_width_eff_m"])

    save_config(config_path, data)
    print(f"[mega-cal] Saved calibration values to {config_path}")


def add_common_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--post-open-wait",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the port (Mega often resets on open)",
    )
    parser.add_argument(
        "--reply-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for commands that are expected to reply",
    )
    parser.add_argument(
        "--swap-sides",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Treat Mega M1/ENC1 as robot right and M2/ENC2 as robot left (default: disabled)",
    )
    parser.add_argument(
        "--config-file",
        default=DEFAULT_CONFIG,
        help="Calibration YAML file to read/write",
    )
    parser.add_argument(
        "--write-config",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Write computed calibration values back to the calibration YAML (default: enabled)",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibration helper for mega_keyboard_drive encoder and motion tests."
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    snapshot = subparsers.add_parser("snapshot", help="Print current encoder and state information")
    add_common_serial_args(snapshot)

    straight = subparsers.add_parser(
        "straight",
        help="Reset encoders, drive straight for a fixed duration, and optionally compute meters/tick",
    )
    add_common_serial_args(straight)
    straight.add_argument("--pwm", type=int, default=90, help="PWM magnitude to use (0-255)")
    straight.add_argument("--duration", type=float, default=1.6, help="Seconds to drive straight")
    straight.add_argument(
        "--direction",
        choices=("forward", "reverse"),
        default="forward",
        help="Direction to drive during the calibration run",
    )
    straight.add_argument(
        "--distance-m",
        type=float,
        default=0.0,
        help="Measured traveled distance in meters; if set, meters-per-tick is computed",
    )
    straight.add_argument(
        "--send-period",
        type=float,
        default=0.2,
        help="Seconds between repeated BOTH commands so the Mega watchdog stays armed",
    )
    straight.add_argument(
        "--settle-time",
        type=float,
        default=0.25,
        help="Seconds to wait after STOP before reading final encoders",
    )
    straight.add_argument(
        "--left-cmd-scale",
        type=float,
        default=1.0,
        help="Scale applied to the robot-left command during the run",
    )
    straight.add_argument(
        "--right-cmd-scale",
        type=float,
        default=1.0,
        help="Scale applied to the robot-right command during the run",
    )

    straight_bidir = subparsers.add_parser(
        "straight-bidir",
        help="Run straight forward and reverse, then compute averaged meters/tick and direction asymmetry",
    )
    add_common_serial_args(straight_bidir)
    straight_bidir.add_argument("--pwm", type=int, default=90, help="PWM magnitude to use (0-255)")
    straight_bidir.add_argument("--duration", type=float, default=1.6, help="Seconds per direction")
    straight_bidir.add_argument(
        "--distance-m",
        type=float,
        default=0.0,
        help="Measured absolute traveled distance in meters for both directions",
    )
    straight_bidir.add_argument(
        "--forward-distance-m",
        type=float,
        default=0.0,
        help="Measured absolute traveled distance in meters for the forward run",
    )
    straight_bidir.add_argument(
        "--reverse-distance-m",
        type=float,
        default=0.0,
        help="Measured absolute traveled distance in meters for the reverse run",
    )
    straight_bidir.add_argument(
        "--send-period",
        type=float,
        default=0.2,
        help="Seconds between repeated BOTH commands so the Mega watchdog stays armed",
    )
    straight_bidir.add_argument(
        "--settle-time",
        type=float,
        default=0.25,
        help="Seconds to wait after STOP before reading final encoders",
    )
    straight_bidir.add_argument(
        "--between-runs-pause",
        type=float,
        default=2.0,
        help="Seconds to wait between the forward and reverse runs",
    )
    straight_bidir.add_argument(
        "--left-cmd-scale",
        type=float,
        default=1.0,
        help="Scale applied to the robot-left command during the run",
    )
    straight_bidir.add_argument(
        "--right-cmd-scale",
        type=float,
        default=1.0,
        help="Scale applied to the robot-right command during the run",
    )

    straight_trim = subparsers.add_parser(
        "straight-trim",
        help="Reset encoders, drive straight for a fixed duration, and suggest LEFT_CMD_SCALE/RIGHT_CMD_SCALE",
    )
    add_common_serial_args(straight_trim)
    straight_trim.add_argument("--pwm", type=int, default=90, help="PWM magnitude to use (0-255)")
    straight_trim.add_argument(
        "--duration",
        type=float,
        default=1.6,
        help="Seconds to drive straight while comparing left/right encoder motion",
    )
    straight_trim.add_argument(
        "--direction",
        choices=("forward", "reverse"),
        default="forward",
        help="Direction to drive during the trim run",
    )
    straight_trim.add_argument(
        "--drift-direction",
        choices=("left", "right", "none"),
        default="none",
        help=(
            "Observed robot drift during the run. If set, right drift reduces left_cmd_scale "
            "and left drift reduces right_cmd_scale."
        ),
    )
    straight_trim.add_argument(
        "--left-cmd-scale",
        "--current-left-cmd-scale",
        dest="left_cmd_scale",
        type=float,
        default=1.0,
        help="Current LEFT_CMD_SCALE value, used when suggesting the next value",
    )
    straight_trim.add_argument(
        "--right-cmd-scale",
        "--current-right-cmd-scale",
        dest="right_cmd_scale",
        type=float,
        default=1.0,
        help="Current RIGHT_CMD_SCALE value, used when suggesting the next value",
    )
    straight_trim.add_argument(
        "--left-m-per-tick",
        type=float,
        default=0.0,
        help="Optional calibrated left meters-per-tick; used to compare physical distance instead of raw ticks",
    )
    straight_trim.add_argument(
        "--right-m-per-tick",
        type=float,
        default=0.0,
        help="Optional calibrated right meters-per-tick; used to compare physical distance instead of raw ticks",
    )
    straight_trim.add_argument(
        "--send-period",
        type=float,
        default=0.2,
        help="Seconds between repeated BOTH commands so the Mega watchdog stays armed",
    )
    straight_trim.add_argument(
        "--settle-time",
        type=float,
        default=0.25,
        help="Seconds to wait after STOP before reading final encoders",
    )

    spin = subparsers.add_parser(
        "spin",
        help="Reset encoders, spin in place for a fixed duration, and optionally compute effective track width",
    )
    add_common_serial_args(spin)
    spin.add_argument("--pwm", type=int, default=75, help="PWM magnitude to use (0-255)")
    spin.add_argument("--duration", type=float, default=1.2, help="Seconds to spin in place")
    spin.add_argument(
        "--direction",
        choices=("cw", "ccw"),
        default="cw",
        help="Spin direction: cw sends left forward / right reverse",
    )
    spin.add_argument(
        "--angle-deg",
        type=float,
        default=0.0,
        help="Measured absolute rotation angle in degrees; if set, effective track width is computed",
    )
    spin.add_argument(
        "--left-m-per-tick",
        type=float,
        default=0.0,
        help="Calibrated left meters-per-tick from a straight-line run",
    )
    spin.add_argument(
        "--right-m-per-tick",
        type=float,
        default=0.0,
        help="Calibrated right meters-per-tick from a straight-line run",
    )
    spin.add_argument(
        "--send-period",
        type=float,
        default=0.2,
        help="Seconds between repeated BOTH commands so the Mega watchdog stays armed",
    )
    spin.add_argument(
        "--settle-time",
        type=float,
        default=0.25,
        help="Seconds to wait after STOP before reading final encoders",
    )
    spin.add_argument(
        "--left-cmd-scale",
        type=float,
        default=1.0,
        help="Scale applied to the robot-left command during the run",
    )
    spin.add_argument(
        "--right-cmd-scale",
        type=float,
        default=1.0,
        help="Scale applied to the robot-right command during the run",
    )

    return parser


def handle_snapshot(ser: serial.Serial, args: argparse.Namespace) -> int:
    verify_keyboard_firmware(ser, args.reply_timeout)
    left, right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)
    state = expect_reply(ser, "STATE", "STATE ", args.reply_timeout)
    print()
    print("[mega-cal] Snapshot")
    print(f"[mega-cal]   left_ticks={left}")
    print(f"[mega-cal]   right_ticks={right}")
    print(f"[mega-cal]   {state}")
    return 0


def handle_straight(ser: serial.Serial, args: argparse.Namespace) -> int:
    verify_keyboard_firmware(ser, args.reply_timeout)
    reset_encoders(ser, args.reply_timeout)
    start_left, start_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)

    pwm = max(0, min(255, abs(args.pwm)))
    signed_pwm = pwm if args.direction == "forward" else -pwm
    command = make_scaled_both_command(
        signed_pwm,
        signed_pwm,
        args.left_cmd_scale,
        args.right_cmd_scale,
        args.swap_sides,
    )

    print()
    print("[mega-cal] Straight run")
    print(f"[mega-cal]   command={command}")
    print(f"[mega-cal]   duration_s={args.duration:.3f}")
    print(f"[mega-cal]   send_period_s={args.send_period:.3f}")
    print(f"[mega-cal]   left_cmd_scale={args.left_cmd_scale:.6f}")
    print(f"[mega-cal]   right_cmd_scale={args.right_cmd_scale:.6f}")

    drive_for(ser, command, args.duration, args.send_period, args.settle_time, args.reply_timeout)
    end_left, end_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)
    delta_left, delta_right = print_encoder_summary(start_left, start_right, end_left, end_right)

    if args.distance_m > 0.0:
        updates = print_straight_calibration(args.distance_m, delta_left, delta_right)
        maybe_update_config(args, updates)

    return 0


def run_straight_once(
    ser: serial.Serial,
    args: argparse.Namespace,
    direction: str,
) -> tuple[int, int]:
    reset_encoders(ser, args.reply_timeout)
    start_left, start_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)

    pwm = max(0, min(255, abs(args.pwm)))
    signed_pwm = pwm if direction == "forward" else -pwm
    command = make_scaled_both_command(
        signed_pwm,
        signed_pwm,
        args.left_cmd_scale,
        args.right_cmd_scale,
        args.swap_sides,
    )

    print()
    print(f"[mega-cal] Straight {direction} run")
    print(f"[mega-cal]   command={command}")
    print(f"[mega-cal]   duration_s={args.duration:.3f}")
    print(f"[mega-cal]   send_period_s={args.send_period:.3f}")
    print(f"[mega-cal]   left_cmd_scale={args.left_cmd_scale:.6f}")
    print(f"[mega-cal]   right_cmd_scale={args.right_cmd_scale:.6f}")

    drive_for(ser, command, args.duration, args.send_period, args.settle_time, args.reply_timeout)
    end_left, end_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)
    return print_encoder_summary(start_left, start_right, end_left, end_right)


def handle_straight_bidir(ser: serial.Serial, args: argparse.Namespace) -> int:
    verify_keyboard_firmware(ser, args.reply_timeout)
    forward_distance_m = args.forward_distance_m if args.forward_distance_m > 0.0 else args.distance_m
    reverse_distance_m = args.reverse_distance_m if args.reverse_distance_m > 0.0 else args.distance_m
    if forward_distance_m <= 0.0 or reverse_distance_m <= 0.0:
        raise RuntimeError(
            "straight-bidir needs --distance-m, or both --forward-distance-m and --reverse-distance-m"
        )

    forward_delta_left, forward_delta_right = run_straight_once(ser, args, "forward")

    print()
    print(f"[mega-cal] Waiting {args.between_runs_pause:.3f}s before reverse run")
    time.sleep(max(0.0, args.between_runs_pause))

    reverse_delta_left, reverse_delta_right = run_straight_once(ser, args, "reverse")

    updates = print_straight_bidir_calibration(
        forward_distance_m,
        reverse_distance_m,
        forward_delta_left,
        forward_delta_right,
        reverse_delta_left,
        reverse_delta_right,
    )
    maybe_update_config(args, updates)
    return 0


def handle_spin(ser: serial.Serial, args: argparse.Namespace) -> int:
    verify_keyboard_firmware(ser, args.reply_timeout)
    reset_encoders(ser, args.reply_timeout)
    start_left, start_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)

    pwm = max(0, min(255, abs(args.pwm)))
    left_pwm, right_pwm = (pwm, -pwm) if args.direction == "cw" else (-pwm, pwm)
    command = make_scaled_both_command(
        left_pwm,
        right_pwm,
        args.left_cmd_scale,
        args.right_cmd_scale,
        args.swap_sides,
    )

    print()
    print("[mega-cal] Spin run")
    print(f"[mega-cal]   command={command}")
    print(f"[mega-cal]   duration_s={args.duration:.3f}")
    print(f"[mega-cal]   send_period_s={args.send_period:.3f}")
    print(f"[mega-cal]   left_cmd_scale={args.left_cmd_scale:.6f}")
    print(f"[mega-cal]   right_cmd_scale={args.right_cmd_scale:.6f}")

    drive_for(ser, command, args.duration, args.send_period, args.settle_time, args.reply_timeout)
    end_left, end_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)
    delta_left, delta_right = print_encoder_summary(start_left, start_right, end_left, end_right)

    if args.angle_deg > 0.0:
        if args.left_m_per_tick <= 0.0 or args.right_m_per_tick <= 0.0:
            raise RuntimeError(
                "spin calibration needs --left-m-per-tick and --right-m-per-tick when --angle-deg is set"
            )
        updates = print_spin_calibration(
            args.angle_deg,
            args.left_m_per_tick,
            args.right_m_per_tick,
            delta_left,
            delta_right,
        )
        maybe_update_config(args, updates)

    return 0


def handle_straight_trim(ser: serial.Serial, args: argparse.Namespace) -> int:
    verify_keyboard_firmware(ser, args.reply_timeout)
    reset_encoders(ser, args.reply_timeout)
    start_left, start_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)

    pwm = max(0, min(255, abs(args.pwm)))
    signed_pwm = pwm if args.direction == "forward" else -pwm
    command = make_scaled_both_command(
        signed_pwm,
        signed_pwm,
        args.left_cmd_scale,
        args.right_cmd_scale,
        args.swap_sides,
    )

    print()
    print("[mega-cal] Straight trim run")
    print(f"[mega-cal]   command={command}")
    print(f"[mega-cal]   duration_s={args.duration:.3f}")
    print(f"[mega-cal]   send_period_s={args.send_period:.3f}")
    print(f"[mega-cal]   left_cmd_scale={args.left_cmd_scale:.6f}")
    print(f"[mega-cal]   right_cmd_scale={args.right_cmd_scale:.6f}")

    drive_for(ser, command, args.duration, args.send_period, args.settle_time, args.reply_timeout)
    end_left, end_right = read_encoders(ser, args.reply_timeout, swap_sides=args.swap_sides)
    delta_left, delta_right = print_encoder_summary(start_left, start_right, end_left, end_right)
    updates = print_straight_trim_calibration(
        delta_left,
        delta_right,
        args.left_cmd_scale,
        args.right_cmd_scale,
        args.left_m_per_tick,
        args.right_m_per_tick,
        args.drift_direction,
    )
    maybe_update_config(args, updates)
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.2, write_timeout=1.0) as ser:
            print(f"[mega-cal] Opened {args.port} @ {args.baudrate}")
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            if args.mode == "snapshot":
                return handle_snapshot(ser, args)
            if args.mode == "straight":
                return handle_straight(ser, args)
            if args.mode == "straight-bidir":
                return handle_straight_bidir(ser, args)
            if args.mode == "straight-trim":
                return handle_straight_trim(ser, args)
            if args.mode == "spin":
                return handle_spin(ser, args)

            raise RuntimeError(f"unsupported mode: {args.mode}")
    except serial.SerialException as exc:
        print(f"[mega-cal] Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"[mega-cal] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
