#!/usr/bin/env python3
import argparse
import queue
import sys
import threading
import time

import serial


def serial_reader(
    ser: serial.Serial,
    stop_event: threading.Event,
    error_queue: "queue.Queue[str]",
) -> None:
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except (serial.SerialException, OSError) as exc:
            error_queue.put(f"SERIAL_ERROR {exc}")
            return

        if not raw:
            continue

        text = raw.decode("utf-8", errors="replace").strip()
        if not text or text.startswith("OK "):
            continue
        print(text, flush=True)


def stdin_reader(command_queue: "queue.Queue[str]", stdin_closed: threading.Event) -> None:
    try:
        for raw_line in sys.stdin:
            command = raw_line.strip()
            if not command:
                continue
            command_queue.put(command)
    finally:
        stdin_closed.set()


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
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=1.0,
        help="Seconds to wait before retrying if the serial port fails",
    )
    args = parser.parse_args()

    command_queue: "queue.Queue[str]" = queue.Queue()
    stdin_closed = threading.Event()
    threading.Thread(target=stdin_reader, args=(command_queue, stdin_closed), daemon=True).start()

    ser: serial.Serial | None = None
    serial_stop_event: threading.Event | None = None
    serial_thread: threading.Thread | None = None
    serial_error_queue: "queue.Queue[str]" = queue.Queue()

    def close_serial() -> None:
        nonlocal ser, serial_stop_event, serial_thread

        if serial_stop_event is not None:
            serial_stop_event.set()

        if ser is not None:
            try:
                ser.write(b"STOP\n")
                ser.flush()
            except (serial.SerialException, OSError):
                pass

        if serial_thread is not None:
            serial_thread.join(timeout=0.2)

        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass

        ser = None
        serial_stop_event = None
        serial_thread = None

    while True:
        if stdin_closed.is_set() and command_queue.empty():
            close_serial()
            return 0

        if ser is None:
            try:
                ser = serial.Serial(args.port, args.baudrate, timeout=0.05, write_timeout=1.0)
                time.sleep(max(0.0, args.post_open_wait))
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                while not serial_error_queue.empty():
                    try:
                        serial_error_queue.get_nowait()
                    except queue.Empty:
                        break
                serial_stop_event = threading.Event()
                serial_thread = threading.Thread(
                    target=serial_reader,
                    args=(ser, serial_stop_event, serial_error_queue),
                    daemon=True,
                )
                serial_thread.start()
                print("READY", flush=True)
            except (serial.SerialException, OSError) as exc:
                print(f"SERIAL_ERROR {exc}", file=sys.stderr, flush=True)
                close_serial()
                time.sleep(max(0.1, args.reconnect_delay))
                continue

        try:
            serial_error = serial_error_queue.get_nowait()
        except queue.Empty:
            serial_error = ""

        if serial_error:
            print(serial_error, file=sys.stderr, flush=True)
            close_serial()
            time.sleep(max(0.1, args.reconnect_delay))
            continue

        try:
            command = command_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        if ser is None:
            continue

        try:
            ser.write((command + "\n").encode("utf-8"))
            ser.flush()
        except (serial.SerialException, OSError) as exc:
            print(f"SERIAL_ERROR {exc}", file=sys.stderr, flush=True)
            close_serial()
            time.sleep(max(0.1, args.reconnect_delay))


if __name__ == "__main__":
    raise SystemExit(main())
