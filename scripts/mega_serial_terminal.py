#!/usr/bin/env python3
import argparse
import select
import sys
import time

import serial


def read_available(ser: serial.Serial) -> None:
    while ser.in_waiting > 0:
        raw = ser.readline()
        if not raw:
            break
        text = raw.decode("utf-8", errors="replace").strip()
        if text:
            print(f"<- {text}")


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()
    print(f"-> {command}")


def read_line(ser: serial.Serial, timeout_s: float) -> str:
    original_timeout = ser.timeout
    ser.timeout = timeout_s
    try:
        raw = ser.readline()
    finally:
        ser.timeout = original_timeout
    return raw.decode("utf-8", errors="replace").strip() if raw else ""


def wait_for_ready(ser: serial.Serial, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        text = read_line(ser, 0.2)
        if not text:
            continue
        print(f"<- {text}")
        if text == "MEGA_KEYBOARD_READY":
            return True
        if text == "ERR MEGA_KEYBOARD_NOT_READY":
            return False
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Manual serial terminal for Arduino Mega motor firmware.")
    parser.add_argument("--port", required=True, help="Serial device, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--post-open-wait", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=120.0)
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.05, write_timeout=1.0) as ser:
            print(f"[mega-terminal] Opened {args.port} @ {args.baudrate}")
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_output_buffer()
            if not wait_for_ready(ser, args.ready_timeout):
                print("[mega-terminal] Warning: did not see MEGA_KEYBOARD_READY; continuing for debug.")
            ser.reset_input_buffer()

            print(
                "[mega-terminal] Type commands like: ID, PING, STATE, LIMITS, "
                "BOTH 100 100, ARM X 20, ARM Z 100, S 90, DIST, STOP"
            )
            print("[mega-terminal] Type q or quit to stop and exit.")

            while True:
                read_available(ser)

                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue

                command = sys.stdin.readline()
                if command == "":
                    break

                command = command.strip()
                if not command:
                    continue
                if command.lower() in ("q", "quit", "exit"):
                    send_command(ser, "STOP")
                    time.sleep(0.1)
                    read_available(ser)
                    break

                send_command(ser, command)
                time.sleep(0.1)
                read_available(ser)

    except serial.SerialException as exc:
        print(f"[mega-terminal] Serial error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
