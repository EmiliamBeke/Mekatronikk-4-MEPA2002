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


def read_encoder_count(ser: serial.Serial, label: str, timeout: float) -> int:
    reply = expect_reply(ser, label, f"{label} ", timeout)
    try:
        return int(reply.split()[1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"failed to parse encoder reply: {reply!r}") from exc


def read_encoder_count_quiet(ser: serial.Serial, label: str, timeout: float) -> int:
    send_command(ser, label)
    reply = read_line(ser, timeout)
    if reply is None:
        raise RuntimeError(f"timeout waiting for reply to {label!r}")
    expected_prefix = f"{label} "
    if not reply.startswith(expected_prefix):
        raise RuntimeError(
            f"unexpected reply to {label!r}: expected prefix {expected_prefix!r}, got {reply!r}"
        )
    try:
        return int(reply.split()[1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"failed to parse encoder reply: {reply!r}") from exc


def read_encoder_pair(ser: serial.Serial, timeout: float, quiet: bool = False) -> tuple[int, int]:
    if quiet:
        enc1 = read_encoder_count_quiet(ser, "ENC1", timeout)
        enc2 = read_encoder_count_quiet(ser, "ENC2", timeout)
        return enc1, enc2

    enc1 = read_encoder_count(ser, "ENC1", timeout)
    enc2 = read_encoder_count(ser, "ENC2", timeout)
    return enc1, enc2


def run_step_with_delta(
    ser: serial.Serial,
    label: str,
    command: str,
    timeout: float,
    duration: float,
    sample_period: float,
    inter_step_pause: float,
    prev_enc1: int,
    prev_enc2: int,
) -> tuple[int, int, int, int, int, int]:
    print(f"[mega-motor-test] Step: {label}")
    expect_reply(ser, command, "OK", timeout)

    min1 = prev_enc1
    max1 = prev_enc1
    min2 = prev_enc2
    max2 = prev_enc2
    deadline = time.monotonic() + max(0.0, duration)

    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        time.sleep(min(sample_period, max(0.0, deadline - now)))
        enc1_now, enc2_now = read_encoder_pair(ser, timeout, quiet=True)
        min1 = min(min1, enc1_now)
        max1 = max(max1, enc1_now)
        min2 = min(min2, enc2_now)
        max2 = max(max2, enc2_now)

    expect_reply(ser, "STOP", "OK STOP", timeout)
    time.sleep(max(0.0, inter_step_pause))

    enc1_now, enc2_now = read_encoder_pair(ser, timeout)
    min1 = min(min1, enc1_now)
    max1 = max(max1, enc1_now)
    min2 = min(min2, enc2_now)
    max2 = max(max2, enc2_now)

    delta1 = enc1_now - prev_enc1
    delta2 = enc2_now - prev_enc2
    span1 = max1 - min1
    span2 = max2 - min2
    print(
        f"[mega-motor-test] Encoder delta after {label}: ENC1={delta1} ENC2={delta2} "
        f"(span ENC1={span1} ENC2={span2})"
    )
    return enc1_now, enc2_now, delta1, delta2, span1, span2


def sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def dominant_encoder_name(delta1_a: int, delta2_a: int, delta1_b: int, delta2_b: int) -> str:
    enc1_score = abs(delta1_a) + abs(delta1_b)
    enc2_score = abs(delta2_a) + abs(delta2_b)
    if enc1_score == 0 and enc2_score == 0:
        return "none"
    return "ENC1" if enc1_score >= enc2_score else "ENC2"


def motion_total(result: tuple[int, int, int, int]) -> int:
    delta1, delta2, span1, span2 = result
    return abs(delta1) + abs(delta2) + span1 + span2


def encoder_value(result: tuple[int, int, int, int], encoder_name: str) -> int:
    if encoder_name == "ENC1":
        return result[0]
    if encoder_name == "ENC2":
        return result[1]
    return 0


def print_fault(message: str) -> None:
    print(f"[mega-motor-test]   FAULT: {message}", file=sys.stderr)


def print_note(message: str) -> None:
    print(f"[mega-motor-test]   Note: {message}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Short DFR0601 motor test over Arduino Mega serial.")
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--pwm", type=int, default=140, help="PWM magnitude to use for the test (0-255)")
    parser.add_argument("--step-duration", type=float, default=2.5, help="Seconds per motor step")
    parser.add_argument(
        "--inter-step-pause",
        type=float,
        default=3.0,
        help="Seconds to pause after STOP before next step",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.15,
        help="Seconds between continuous encoder samples while a motor command is active",
    )
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

            firmware = expect_reply(ser, "ID", "MEGA_", args.reply_timeout)
            if firmware != "MEGA_DFR0601_TEST":
                raise RuntimeError(
                    "wrong firmware for motor wiring test: "
                    f"expected 'MEGA_DFR0601_TEST', got {firmware!r}. "
                    "Run: make mega-upload mega_dfr0601_test"
                )
            expect_reply(ser, "PING", "PONG", args.reply_timeout)
            expect_reply(ser, "STOP", "OK STOP", args.reply_timeout)
            expect_reply(ser, "RESET ENC1", "OK RESET ENC1", args.reply_timeout)
            expect_reply(ser, "RESET ENC2", "OK RESET ENC2", args.reply_timeout)
            initial_enc1, initial_enc2 = read_encoder_pair(ser, args.reply_timeout)
            print(f"[mega-motor-test] Initial ENC1={initial_enc1} ENC2={initial_enc2}")

            current_enc1, current_enc2 = initial_enc1, initial_enc2
            failures: list[str] = []
            step_results: dict[str, tuple[int, int, int, int]] = {}

            steps = [
                ("m1_forward", "M1 forward", f"M1 {pwm}"),
                ("m1_reverse", "M1 reverse", f"M1 {-pwm}"),
                ("m2_forward", "M2 forward", f"M2 {pwm}"),
                ("m2_reverse", "M2 reverse", f"M2 {-pwm}"),
                ("both_forward", "both forward", f"BOTH {pwm} {pwm}"),
                ("both_reverse", "both reverse", f"BOTH {-pwm} {-pwm}"),
            ]

            for key, label, command in steps:
                current_enc1, current_enc2, delta1, delta2, span1, span2 = run_step_with_delta(
                    ser,
                    label,
                    command,
                    args.reply_timeout,
                    args.step_duration,
                    max(0.02, args.sample_period),
                    max(0.0, args.inter_step_pause),
                    current_enc1,
                    current_enc2,
                )
                step_results[key] = (delta1, delta2, span1, span2)
                if delta1 == 0 and delta2 == 0 and span1 == 0 and span2 == 0:
                    failures.append(label)

            m1_f = step_results["m1_forward"]
            m1_r = step_results["m1_reverse"]
            m2_f = step_results["m2_forward"]
            m2_r = step_results["m2_reverse"]
            both_f = step_results["both_forward"]
            both_r = step_results["both_reverse"]

            m1_dom = dominant_encoder_name(m1_f[0], m1_f[1], m1_r[0], m1_r[1])
            m2_dom = dominant_encoder_name(m2_f[0], m2_f[1], m2_r[0], m2_r[1])

            print("[mega-motor-test] Diagnostic summary:")
            print("[mega-motor-test]   Expected electrical mapping: M1 -> ENC1, M2 -> ENC2")
            print(f"[mega-motor-test]   M1 dominant encoder: {m1_dom}")
            print(f"[mega-motor-test]   M2 dominant encoder: {m2_dom}")

            if m1_dom == "ENC1":
                m1_signs = (sign(m1_f[0]), sign(m1_r[0]))
            elif m1_dom == "ENC2":
                m1_signs = (sign(m1_f[1]), sign(m1_r[1]))
            else:
                m1_signs = (0, 0)

            if m2_dom == "ENC1":
                m2_signs = (sign(m2_f[0]), sign(m2_r[0]))
            elif m2_dom == "ENC2":
                m2_signs = (sign(m2_f[1]), sign(m2_r[1]))
            else:
                m2_signs = (0, 0)

            fault_count = 0

            m1_total = motion_total(m1_f) + motion_total(m1_r)
            m2_total = motion_total(m2_f) + motion_total(m2_r)
            if m1_total == 0:
                fault_count += 1
                print_fault(
                    "M1 command produced no encoder movement in either direction. "
                    "Check M1 driver output, M1 motor wires, motor power, and ENC1/ENC2 supply/signal wiring."
                )
            if m2_total == 0:
                fault_count += 1
                print_fault(
                    "M2 command produced no encoder movement in either direction. "
                    "Check M2 driver output, M2 motor wires, motor power, and ENC1/ENC2 supply/signal wiring."
                )

            if m1_dom == "ENC2" and m2_dom == "ENC1":
                fault_count += 1
                print_fault(
                    "M1 and M2 encoder mapping appears crossed: M1 moves ENC2 and M2 moves ENC1. "
                    "Swap the encoder connections or swap the motor channel naming so M1 pairs with ENC1 and M2 pairs with ENC2."
                )
            elif m1_dom == "ENC2":
                fault_count += 1
                print_fault(
                    "M1 appears to move ENC2 instead of ENC1. Check whether M1 motor or encoder wires are on the wrong channel."
                )
            elif m2_dom == "ENC1":
                fault_count += 1
                print_fault(
                    "M2 appears to move ENC1 instead of ENC2. Check whether M2 motor or encoder wires are on the wrong channel."
                )

            if m1_signs[0] != 0 and m1_signs[1] != 0 and m1_signs[0] == m1_signs[1]:
                fault_count += 1
                print_fault(
                    "M1 forward and reverse produced the same encoder sign. "
                    "The motor may not actually reverse, or one encoder phase may be missing/noisy."
                )
            if m2_signs[0] != 0 and m2_signs[1] != 0 and m2_signs[0] == m2_signs[1]:
                fault_count += 1
                print_fault(
                    "M2 forward and reverse produced the same encoder sign. "
                    "The motor may not actually reverse, or one encoder phase may be missing/noisy."
                )

            if (both_f[0] == 0 and both_f[1] == 0 and both_f[2] == 0 and both_f[3] == 0) or (
                both_r[0] == 0 and both_r[1] == 0 and both_r[2] == 0 and both_r[3] == 0
            ):
                fault_count += 1
                print_fault(
                    "BOTH command did not move encoders in at least one direction. "
                    "If M1/M2 worked individually, check shared motor power, driver enable/current limit, and battery voltage sag."
                )

            if m1_dom == m2_dom and m1_dom != "none":
                fault_count += 1
                print_fault(
                    "M1 and M2 appear to affect the same encoder. "
                    "One encoder may be disconnected, or both motor channels are being observed through the same encoder channel."
                )

            if failures:
                fault_count += 1
                print(
                    "[mega-motor-test] No encoder movement detected in: "
                    + ", ".join(failures),
                    file=sys.stderr,
                )
                print(
                    "[mega-motor-test] Try higher PWM/duration, for example: PWM_VALUE=170 STEP_DURATION=2.0 make mega-motor-test",
                    file=sys.stderr,
                )
            else:
                print_note("Every individual step produced at least some encoder movement.")

            if m1_dom == "ENC1" and m2_dom == "ENC2":
                m1_forward_sign = sign(encoder_value(m1_f, "ENC1"))
                m2_forward_sign = sign(encoder_value(m2_f, "ENC2"))
                print_note(
                    "M1/ENC1 and M2/ENC2 electrical pairing matches the firmware pin assumptions."
                )
                print_note(
                    f"Forward sign observed: M1/ENC1={m1_forward_sign}, M2/ENC2={m2_forward_sign}. "
                    "Use calibration tick signs if robot-forward odometry has the wrong sign."
                )

            if fault_count:
                raise RuntimeError(f"motor wiring test found {fault_count} fault(s)")

            print("[mega-motor-test] Success: motor and encoder wiring matches the expected electrical mapping.")
            return 0
    except serial.SerialException as exc:
        print(f"[mega-motor-test] Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"[mega-motor-test] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
