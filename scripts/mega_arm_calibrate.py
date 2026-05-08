#!/usr/bin/env python3
import argparse
import sys
import time

import serial


IGNORED_PREFIXES = (
    "MEGA_KEYBOARD_READY",
    "EVENT ",
    "OK STOP",
)


def send_command(ser: serial.Serial, command: str) -> None:
    print(f"[mega-arm-cal] -> {command}")
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def read_line(ser: serial.Serial, timeout_s: float) -> str | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        if text:
            return text
    return None


def expect_reply(
    ser: serial.Serial,
    command: str,
    expected_prefixes: tuple[str, ...],
    timeout_s: float,
) -> str:
    send_command(ser, command)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        text = read_line(ser, 0.2)
        if text is None:
            continue
        print(f"[mega-arm-cal] <- {text}")
        if text.startswith(expected_prefixes):
            return text
        if any(text.startswith(prefix) for prefix in IGNORED_PREFIXES):
            continue
        if text.startswith("ERR "):
            raise RuntimeError(text)
    expected = " or ".join(expected_prefixes)
    raise RuntimeError(f"timeout waiting for {expected}")


def verify_runtime_firmware(ser: serial.Serial, timeout_s: float) -> None:
    firmware = expect_reply(ser, "ID", ("MEGA_",), timeout_s)
    if firmware != "MEGA_KEYBOARD_DRIVE":
        raise RuntimeError(f"expected MEGA_KEYBOARD_DRIVE firmware, got {firmware!r}")
    expect_reply(ser, "PING", ("PONG",), timeout_s)
    expect_reply(ser, "STOP", ("OK STOP",), timeout_s)


def main() -> int:
    parser = argparse.ArgumentParser(description="One-shot arm stepper homing for Mega runtime firmware.")
    parser.add_argument("mode", choices=("all", "x"), help="all homes X and Z; x homes only X")
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--post-open-wait",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the port because the Mega may reset",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="Seconds to wait for homing to finish",
    )
    args = parser.parse_args()

    command = "HOME ARM" if args.mode == "all" else "HOME X"
    expected = ("OK ARM STARTUP HOME",) if args.mode == "all" else ("OK HOME X",)

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.1, write_timeout=1.0) as ser:
            print(f"[mega-arm-cal] Opened {args.port} @ {args.baudrate}")
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            verify_runtime_firmware(ser, 2.0)
            reply = expect_reply(ser, command, expected, args.timeout)
            print(f"[mega-arm-cal] Success: {reply}")
            return 0
    except serial.SerialException as exc:
        print(f"[mega-arm-cal] Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"[mega-arm-cal] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
