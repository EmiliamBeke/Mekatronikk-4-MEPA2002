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


def scale_signed_pwm(value: int, scale: float, sign: int) -> int:
    if scale <= 0.0:
        raise ValueError("Command scale must be greater than zero.")
    if sign not in (-1, 1):
        raise ValueError("Command sign must be either -1 or 1.")
    if value == 0:
        return 0

    scaled = int(round(abs(value) * scale))
    scaled = max(1, min(255, scaled))
    return scaled * sign if value > 0 else -scaled * sign


def map_robot_commands(
    left_cmd: int,
    right_cmd: int,
    *,
    left_cmd_scale: float,
    right_cmd_scale: float,
    left_cmd_sign: int,
    right_cmd_sign: int,
    swap_sides: bool,
) -> tuple[int, int]:
    left_out = scale_signed_pwm(left_cmd, left_cmd_scale, left_cmd_sign)
    right_out = scale_signed_pwm(right_cmd, right_cmd_scale, right_cmd_sign)
    if swap_sides:
        left_out, right_out = right_out, left_out
    return left_out, right_out


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


def is_active(last_seen: float, now: float, timeout: float) -> bool:
    return last_seen > 0.0 and (now - last_seen) <= timeout


def tank_mix(drive: int, steer: int, speed: int, turn_speed: int) -> tuple[int, int]:
    if drive == 0:
        if steer > 0:
            return -turn_speed, turn_speed
        if steer < 0:
            return turn_speed, -turn_speed
        return 0, 0

    left = drive * speed
    right = drive * speed

    turn_delta = min(speed, turn_speed)
    if steer > 0:
        if drive > 0:
            left = drive * (speed - turn_delta)
            right = drive * speed
        else:
            left = drive * speed
            right = drive * (speed - turn_delta)
    elif steer < 0:
        if drive > 0:
            left = drive * speed
            right = drive * (speed - turn_delta)
        else:
            left = drive * (speed - turn_delta)
            right = drive * speed

    return clamp_pwm(left), clamp_pwm(right)


def print_status(
    forward: bool,
    reverse: bool,
    left_turn: bool,
    right_turn: bool,
    speed: int,
    turn_speed: int,
    left_cmd: int,
    right_cmd: int,
    latest_message: str,
) -> None:
    drive_label = "idle"
    if forward and not reverse:
        drive_label = "forward"
    elif reverse and not forward:
        drive_label = "reverse"

    steer_label = "straight"
    if left_turn and not right_turn:
        steer_label = "left"
    elif right_turn and not left_turn:
        steer_label = "right"

    message = f" msg={latest_message}" if latest_message else ""
    sys.stdout.write(
        "\r"
        f"[mega-keyboard] drive={drive_label} steer={steer_label} "
        f"speed={speed} turn_speed={turn_speed} cmd=({left_cmd}, {right_cmd}){message}   "
    )
    sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="Hold-to-drive keyboard teleop for Arduino Mega motor firmware.")
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--speed", type=int, default=90, help="Forward/reverse PWM magnitude (0-255)")
    parser.add_argument("--turn-speed", type=int, default=55, help="Steering PWM magnitude (0-255)")
    parser.add_argument("--swap-sides", action=argparse.BooleanOptionalAction, default=False, help="Swap robot-left and robot-right when sending BOTH to Mega")
    parser.add_argument("--left-cmd-sign", type=int, default=1, help="Sign to apply to robot-left commands")
    parser.add_argument("--right-cmd-sign", type=int, default=1, help="Sign to apply to robot-right commands")
    parser.add_argument("--left-cmd-scale", type=float, default=1.0, help="Scale to apply to robot-left commands")
    parser.add_argument("--right-cmd-scale", type=float, default=1.0, help="Scale to apply to robot-right commands")
    parser.add_argument(
        "--post-open-wait",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the port (Mega often resets on open)",
    )
    parser.add_argument(
        "--send-period",
        type=float,
        default=0.03,
        help="Seconds between repeated drive commands",
    )
    parser.add_argument(
        "--hold-timeout",
        type=float,
        default=0.45,
        help="How long a repeated key is treated as held in the SSH terminal",
    )
    args = parser.parse_args()

    speed = max(0, min(255, args.speed))
    turn_speed = max(0, min(255, args.turn_speed))

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    last_forward_at = -1.0
    last_reverse_at = -1.0
    last_left_at = -1.0
    last_right_at = -1.0
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
                "[mega-keyboard] Hold W/S/A/D. "
                "E/Q speed up/down. P/O turn speed up/down. "
                "SPACE stop. - quit."
            )

            try:
                while True:
                    now = time.monotonic()
                    message = drain_serial(ser)
                    if message:
                        latest_message = message

                    while True:
                        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                        if not ready:
                            break

                        key = sys.stdin.read(1)
                        key_now = time.monotonic()

                        if key == "-":
                            raise KeyboardInterrupt
                        if key in ("w", "W"):
                            last_forward_at = key_now
                        elif key in ("s", "S"):
                            last_reverse_at = key_now
                        elif key in ("a", "A"):
                            last_left_at = key_now
                        elif key in ("d", "D"):
                            last_right_at = key_now
                        elif key == " ":
                            last_forward_at = -1.0
                            last_reverse_at = -1.0
                            last_left_at = -1.0
                            last_right_at = -1.0
                        elif key in ("e", "E"):
                            speed = min(255, speed + 5)
                        elif key in ("q", "Q"):
                            speed = max(0, speed - 5)
                        elif key in ("p", "P"):
                            turn_speed = min(255, turn_speed + 5)
                        elif key in ("o", "O"):
                            turn_speed = max(0, turn_speed - 5)

                    now = time.monotonic()
                    forward = is_active(last_forward_at, now, args.hold_timeout)
                    reverse = is_active(last_reverse_at, now, args.hold_timeout)
                    left_turn = is_active(last_left_at, now, args.hold_timeout)
                    right_turn = is_active(last_right_at, now, args.hold_timeout)

                    drive = 0
                    if forward and not reverse:
                        drive = 1
                    elif reverse and not forward:
                        drive = -1

                    steer = 0
                    if left_turn and not right_turn:
                        steer = 1
                    elif right_turn and not left_turn:
                        steer = -1

                    left_cmd, right_cmd = tank_mix(drive, steer, speed, turn_speed)
                    send_left, send_right = map_robot_commands(
                        left_cmd,
                        right_cmd,
                        left_cmd_scale=args.left_cmd_scale,
                        right_cmd_scale=args.right_cmd_scale,
                        left_cmd_sign=args.left_cmd_sign,
                        right_cmd_sign=args.right_cmd_sign,
                        swap_sides=args.swap_sides,
                    )
                    command = "STOP" if send_left == 0 and send_right == 0 else f"BOTH {send_left} {send_right}"

                    should_repeat = command != "STOP"
                    should_send = command != last_sent_command or (should_repeat and now - last_sent_at >= args.send_period)

                    if should_send:
                        send_command(ser, command)
                        last_sent_command = command
                        last_sent_at = now

                    print_status(
                        forward,
                        reverse,
                        left_turn,
                        right_turn,
                        speed,
                        turn_speed,
                        send_left,
                        send_right,
                        latest_message,
                    )

                    time.sleep(0.01)
            except KeyboardInterrupt:
                pass
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
