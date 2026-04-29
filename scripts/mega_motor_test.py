#!/usr/bin/env python3
import argparse
import sys
import time

import serial


def read_line(ser: serial.Serial, timeout: float) -> str | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        if text:
            return text
    return None


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def expect_reply(ser: serial.Serial, command: str, expected_prefix: str, timeout: float) -> str:
    print(f"[mega-motor-test] -> {command}")
    send_command(ser, command)
    reply = read_line(ser, timeout)
    if reply is None:
        raise RuntimeError(f"timeout waiting for reply to {command!r}")
    print(f"[mega-motor-test] <- {reply}")
    if not reply.startswith(expected_prefix):
        raise RuntimeError(
            f"unexpected reply to {command!r}: expected prefix {expected_prefix!r}, got {reply!r}"
        )
    return reply


def read_encoder_count(ser: serial.Serial, timeout: float) -> int:
    reply = expect_reply(ser, "ENC1", "ENC1 ", timeout)
    try:
        return int(reply.split()[1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"failed to parse encoder reply: {reply!r}") from exc


def read_encoder_pair(ser: serial.Serial, timeout: float) -> tuple[int, int]:
    enc1 = read_encoder_count(ser, timeout)
    reply = expect_reply(ser, "ENC2", "ENC2 ", timeout)
    try:
        enc2 = int(reply.split()[1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"failed to parse encoder reply: {reply!r}") from exc
    return enc1, enc2


def run_step(ser: serial.Serial, command: str, timeout: float, duration: float) -> None:
    expect_reply(ser, command, "OK", timeout)
    time.sleep(max(0.0, duration))
    expect_reply(ser, "STOP", "OK STOP", timeout)
    time.sleep(0.2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Short DFR0601 motor test over Arduino Mega serial.")
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--pwm", type=int, default=80, help="PWM magnitude to use for the test (0-255)")
    parser.add_argument("--step-duration", type=float, default=0.8, help="Seconds per motor step")
    parser.add_argument("--reply-timeout", type=float, default=2.0, help="Seconds to wait for a reply")
    parser.add_argument(
        "--post-open-wait",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the port (Mega often resets on open)",
    )
    args = parser.parse_args()

    pwm = max(0, min(255, abs(args.pwm)))

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.2, write_timeout=1.0) as ser:
            print(f"[mega-motor-test] Opened {args.port} @ {args.baudrate}")
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            expect_reply(ser, "ID", "MEGA_DFR0601_TEST", args.reply_timeout)
            expect_reply(ser, "PING", "PONG", args.reply_timeout)
            expect_reply(ser, "STOP", "OK STOP", args.reply_timeout)
            expect_reply(ser, "RESET ENC1", "OK RESET ENC1", args.reply_timeout)
            expect_reply(ser, "RESET ENC2", "OK RESET ENC2", args.reply_timeout)
            initial_enc1, initial_enc2 = read_encoder_pair(ser, args.reply_timeout)
            print(f"[mega-motor-test] Initial ENC1={initial_enc1} ENC2={initial_enc2}")

            print("[mega-motor-test] Step 1: M1 forward")
            run_step(ser, f"M1 {pwm}", args.reply_timeout, args.step_duration)
            enc1_after_m1_forward, enc2_after_m1_forward = read_encoder_pair(ser, args.reply_timeout)
            delta1_forward = enc1_after_m1_forward - initial_enc1
            delta2_forward = enc2_after_m1_forward - initial_enc2
            print(
                "[mega-motor-test] Encoder delta after M1 forward: "
                f"ENC1={delta1_forward} ENC2={delta2_forward}"
            )
            if delta1_forward == 0 and delta2_forward == 0:
                raise RuntimeError("encoder counts did not change during M1 forward step")

            print("[mega-motor-test] Step 2: M1 reverse")
            run_step(ser, f"M1 {-pwm}", args.reply_timeout, args.step_duration)
            enc1_after_m1_reverse, enc2_after_m1_reverse = read_encoder_pair(ser, args.reply_timeout)
            delta1_reverse = enc1_after_m1_reverse - enc1_after_m1_forward
            delta2_reverse = enc2_after_m1_reverse - enc2_after_m1_forward
            print(
                "[mega-motor-test] Encoder delta after M1 reverse: "
                f"ENC1={delta1_reverse} ENC2={delta2_reverse}"
            )
            if delta1_reverse == 0 and delta2_reverse == 0:
                raise RuntimeError("encoder counts did not change during M1 reverse step")

            print("[mega-motor-test] Step 3: M2 forward")
            run_step(ser, f"M2 {pwm}", args.reply_timeout, args.step_duration)
            enc1_after_m2_forward, enc2_after_m2_forward = read_encoder_pair(ser, args.reply_timeout)
            delta1_m2_forward = enc1_after_m2_forward - enc1_after_m1_reverse
            delta2_m2_forward = enc2_after_m2_forward - enc2_after_m1_reverse
            print(
                "[mega-motor-test] Encoder delta after M2 forward: "
                f"ENC1={delta1_m2_forward} ENC2={delta2_m2_forward}"
            )
            if delta1_m2_forward == 0 and delta2_m2_forward == 0:
                raise RuntimeError("encoder counts did not change during M2 forward step")

            print("[mega-motor-test] Step 4: M2 reverse")
            run_step(ser, f"M2 {-pwm}", args.reply_timeout, args.step_duration)
            enc1_after_m2_reverse, enc2_after_m2_reverse = read_encoder_pair(ser, args.reply_timeout)
            delta1_m2_reverse = enc1_after_m2_reverse - enc1_after_m2_forward
            delta2_m2_reverse = enc2_after_m2_reverse - enc2_after_m2_forward
            print(
                "[mega-motor-test] Encoder delta after M2 reverse: "
                f"ENC1={delta1_m2_reverse} ENC2={delta2_m2_reverse}"
            )
            if delta1_m2_reverse == 0 and delta2_m2_reverse == 0:
                raise RuntimeError("encoder counts did not change during M2 reverse step")

            print("[mega-motor-test] Step 5: both forward")
            run_step(ser, f"BOTH {pwm} {pwm}", args.reply_timeout, args.step_duration)

            print("[mega-motor-test] Step 6: both reverse")
            run_step(ser, f"BOTH {-pwm} {-pwm}", args.reply_timeout, args.step_duration)

            print("[mega-motor-test] Success: motor controller commands were accepted.")
            return 0
    except serial.SerialException as exc:
        print(f"[mega-motor-test] Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"[mega-motor-test] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
