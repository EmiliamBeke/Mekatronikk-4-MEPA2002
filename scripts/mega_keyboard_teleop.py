#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

import serial


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def clamp_pwm(value: int) -> int:
    return max(-255, min(255, value))


def wheel_command(drive_state: int, turn_state: int, speed: int, turn_speed: int) -> tuple[int, int]:
    left = clamp_pwm(drive_state * speed - turn_state * turn_speed)
    right = clamp_pwm(drive_state * speed + turn_state * turn_speed)
    return left, right


def drain_serial(ser: serial.Serial) -> str:
    latest = ""
    while ser.in_waiting > 0:
        raw = ser.readline()
        if not raw:
            break
        text = raw.decode("utf-8", errors="replace").strip()
        if text and not text.startswith("OK "):
            latest = text
    return latest


def print_status(
    drive_state: int,
    turn_state: int,
    speed: int,
    turn_speed: int,
    left: int,
    right: int,
    latest_message: str,
) -> None:
    drive_label = {1: "forward", 0: "idle", -1: "reverse"}[drive_state]
    turn_label = {1: "left", 0: "straight", -1: "right"}[turn_state]
    message = f" msg={latest_message}" if latest_message else ""
    sys.stdout.write(
        "\r"
        f"[mega-keyboard] drive={drive_label} steer={turn_label} "
        f"speed={speed} turn_speed={turn_speed} cmd=({left}, {right}){message}   "
    )
    sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="Keyboard teleop for Arduino Mega motor test firmware.")
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--speed", type=int, default=90, help="Forward/reverse PWM magnitude (0-255)")
    parser.add_argument("--turn-speed", type=int, default=75, help="Turn PWM magnitude (0-255)")
    parser.add_argument(
        "--post-open-wait",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the port (Mega often resets on open)",
    )
    parser.add_argument(
        "--send-period",
        type=float,
        default=0.05,
        help="Seconds between keepalive commands",
    )
    args = parser.parse_args()

    speed = max(0, min(255, args.speed))
    turn_speed = max(0, min(255, args.turn_speed))

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    drive_state = 0
    turn_state = 0
    latest_message = ""
    last_sent_at = 0.0
    last_sent_command = ""

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.01, write_timeout=1.0) as ser:
            print(f"[mega-keyboard] Opened {args.port} @ {args.baudrate}")
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            tty.setcbreak(fd)
            print(
                "[mega-keyboard] Keys: W forward, S reverse, X drive stop, "
                "A left, D right, C straighten, E/Q speed up/down, "
                "P/O turn speed up/down, SPACE full stop, - quit"
            )
            try:
                while True:
                    now = time.monotonic()
                    message = drain_serial(ser)
                    if message:
                        latest_message = message

                    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if ready:
                        key = sys.stdin.read(1)
                        if key == "-":
                            break
                        if key in ("w", "W"):
                            drive_state = 1
                        elif key in ("s", "S"):
                            drive_state = -1
                        elif key in ("x", "X"):
                            drive_state = 0
                        elif key in ("a", "A"):
                            turn_state = 1
                        elif key in ("d", "D"):
                            turn_state = -1
                        elif key in ("c", "C"):
                            turn_state = 0
                        elif key == " ":
                            drive_state = 0
                            turn_state = 0
                        elif key in ("e", "E"):
                            speed = min(255, speed + 5)
                        elif key in ("q", "Q"):
                            speed = max(0, speed - 5)
                        elif key in ("p", "P"):
                            turn_speed = min(255, turn_speed + 5)
                        elif key in ("o", "O"):
                            turn_speed = max(0, turn_speed - 5)

                    left, right = wheel_command(drive_state, turn_state, speed, turn_speed)
                    command = "STOP" if left == 0 and right == 0 else f"BOTH {left} {right}"

                    if command != last_sent_command or now - last_sent_at >= args.send_period:
                        send_command(ser, command)
                        last_sent_command = command
                        last_sent_at = now

                    print_status(drive_state, turn_state, speed, turn_speed, left, right, latest_message)
            finally:
                try:
                    send_command(ser, "STOP")
                except serial.SerialException:
                    pass
    except serial.SerialException as exc:
        print(f"[mega-keyboard] Serial error: {exc}", file=sys.stderr)
        return 1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print()

    print("[mega-keyboard] Stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
