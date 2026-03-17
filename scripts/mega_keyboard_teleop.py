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


def print_status(speed: int, turn_speed: int, left: int, right: int) -> None:
    sys.stdout.write(
        "\r"
        f"[mega-keyboard] speed={speed} turn={turn_speed} "
        f"command=({left}, {right})  "
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
        "--key-timeout",
        type=float,
        default=0.25,
        help="Stop if no movement key has been seen for this long",
    )
    parser.add_argument(
        "--send-period",
        type=float,
        default=0.1,
        help="Seconds between keepalive commands",
    )
    args = parser.parse_args()

    speed = max(0, min(255, args.speed))
    turn_speed = max(0, min(255, args.turn_speed))

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    current_left = 0
    current_right = 0
    last_motion_key_at = 0.0
    last_sent_at = 0.0
    last_sent_command = ""

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.05, write_timeout=1.0) as ser:
            print(f"[mega-keyboard] Opened {args.port} @ {args.baudrate}")
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            tty.setcbreak(fd)
            print("[mega-keyboard] Keys: W/S forward/back, A/D turn, SPACE stop, +/- speed, q quit")
            try:
                while True:
                    now = time.monotonic()

                    if (current_left != 0 or current_right != 0) and now - last_motion_key_at > args.key_timeout:
                        current_left = 0
                        current_right = 0

                    ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if ready:
                        key = sys.stdin.read(1)
                        now = time.monotonic()

                        if key in ("q", "Q"):
                            break
                        if key in ("w", "W"):
                            current_left = speed
                            current_right = speed
                            last_motion_key_at = now
                        elif key in ("s", "S"):
                            current_left = -speed
                            current_right = -speed
                            last_motion_key_at = now
                        elif key in ("a", "A"):
                            current_left = -turn_speed
                            current_right = turn_speed
                            last_motion_key_at = now
                        elif key in ("d", "D"):
                            current_left = turn_speed
                            current_right = -turn_speed
                            last_motion_key_at = now
                        elif key == " ":
                            current_left = 0
                            current_right = 0
                        elif key in ("+", "="):
                            speed = min(255, speed + 5)
                            turn_speed = min(255, turn_speed + 5)
                        elif key in ("-", "_"):
                            speed = max(0, speed - 5)
                            turn_speed = max(0, turn_speed - 5)

                    command = (
                        "STOP"
                        if current_left == 0 and current_right == 0
                        else f"BOTH {current_left} {current_right}"
                    )

                    if command != last_sent_command or now - last_sent_at >= args.send_period:
                        send_command(ser, command)
                        last_sent_command = command
                        last_sent_at = now

                    print_status(speed, turn_speed, current_left, current_right)
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
