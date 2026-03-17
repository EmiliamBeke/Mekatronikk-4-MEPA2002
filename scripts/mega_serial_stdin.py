#!/usr/bin/env python3
import argparse
import sys
import threading
import time

import serial


def serial_reader(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            print(f"SERIAL_ERROR {exc}", file=sys.stderr, flush=True)
            return

        if not raw:
            continue

        text = raw.decode("utf-8", errors="replace").strip()
        if not text or text.startswith("OK "):
            continue
        print(text, file=sys.stderr, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Forward drive commands from stdin to Arduino Mega serial.")
    parser.add_argument("--port", required=True, help="Serial device path, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--post-open-wait",
        type=float,
        default=2.5,
        help="Seconds to wait after opening the port (Mega often resets on open)",
    )
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baudrate, timeout=0.05, write_timeout=1.0) as ser:
            time.sleep(max(0.0, args.post_open_wait))
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print("READY", flush=True)
            stop_event = threading.Event()
            reader_thread = threading.Thread(target=serial_reader, args=(ser, stop_event), daemon=True)
            reader_thread.start()

            try:
                for raw_line in sys.stdin:
                    command = raw_line.strip()
                    if not command:
                        continue
                    ser.write((command + "\n").encode("utf-8"))
                    ser.flush()
            finally:
                stop_event.set()
                try:
                    ser.write(b"STOP\n")
                    ser.flush()
                except serial.SerialException:
                    pass
                reader_thread.join(timeout=0.2)
        return 0
    except serial.SerialException as exc:
        print(f"SERIAL_ERROR {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
