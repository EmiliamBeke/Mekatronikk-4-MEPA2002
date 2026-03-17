#!/usr/bin/env python3
import argparse
import os
import queue
import shlex
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox


def clamp_pwm(value: int) -> int:
    return max(-255, min(255, value))


def tank_mix(drive: int, steer: int, speed: int, turn_speed: int) -> tuple[int, int]:
    if drive == 0:
        if steer > 0:
            return turn_speed, -turn_speed
        if steer < 0:
            return -turn_speed, turn_speed
        return 0, 0

    left = drive * speed
    right = drive * speed
    turn_delta = min(speed, turn_speed)

    if steer > 0:
        left = drive * speed
        right = drive * (speed - turn_delta)
    elif steer < 0:
        left = drive * (speed - turn_delta)
        right = drive * speed

    return clamp_pwm(left), clamp_pwm(right)


class MegaKeyboardGui:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.speed = max(0, min(255, args.speed))
        self.turn_speed = max(0, min(255, args.turn_speed))
        self.pressed_keys: set[str] = set()
        self.last_command = ""
        self.last_sent_at = 0.0
        self.status_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self.remote_ready = threading.Event()
        self.remote_failed = threading.Event()
        self.remote_error = ""
        self.closed = False
        self.reconnect_delay_s = 1.0
        self.next_reconnect_at = 0.0

        self.proc = self._start_remote_bridge()

        self.root = tk.Tk()
        self.root.title("Mega Keyboard Teleop")
        self.root.geometry("540x260")
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = tk.StringVar(value=f"Connecting to {args.host} ...")
        self.command_var = tk.StringVar(value="cmd=(0, 0)")
        self.speed_var = tk.StringVar(value=self._speed_text())
        self.hint_var = tk.StringVar(
            value="Hold W/S/A/D. E/Q speed. P/O turn speed. SPACE stop. - quit."
        )

        self._build_ui()
        self._bind_keys()

        self.root.after(50, self._pump_status_queue)
        self.root.after(20, self._tick)

    def _build_ui(self) -> None:
        title = tk.Label(
            self.root,
            text="Mega Teleop",
            font=("TkDefaultFont", 18, "bold"),
            fg="#f5f5f5",
            bg="#111111",
        )
        title.pack(pady=(16, 8))

        status = tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("TkDefaultFont", 11),
            fg="#d6d6d6",
            bg="#111111",
        )
        status.pack(pady=4)

        command = tk.Label(
            self.root,
            textvariable=self.command_var,
            font=("TkDefaultFont", 14, "bold"),
            fg="#7fe7a2",
            bg="#111111",
        )
        command.pack(pady=8)

        speed = tk.Label(
            self.root,
            textvariable=self.speed_var,
            font=("TkDefaultFont", 12),
            fg="#f5d97b",
            bg="#111111",
        )
        speed.pack(pady=4)

        hint = tk.Label(
            self.root,
            textvariable=self.hint_var,
            font=("TkDefaultFont", 10),
            fg="#bbbbbb",
            bg="#111111",
            wraplength=500,
            justify="center",
        )
        hint.pack(pady=(12, 6))

        focus_hint = tk.Label(
            self.root,
            text="Klikk i vinduet hvis tastene ikke fanges.",
            font=("TkDefaultFont", 10),
            fg="#8f8f8f",
            bg="#111111",
        )
        focus_hint.pack()

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_force()

    def _start_remote_bridge(self) -> subprocess.Popen[str]:
        remote_repo = self.args.remote_repo
        if remote_repo == "~" or remote_repo == "$HOME":
            remote_repo_expr = "$HOME"
        elif remote_repo.startswith("~/"):
            remote_repo_expr = "$HOME/" + shlex.quote(remote_repo[2:])
        elif remote_repo.startswith("$HOME/"):
            remote_repo_expr = "$HOME/" + shlex.quote(remote_repo[len("$HOME/"):])
        else:
            remote_repo_expr = shlex.quote(remote_repo)

        remote_cmd = (
            f"cd {remote_repo_expr} && "
            f"python3 scripts/mega_serial_stdin.py "
            f"--port {shlex.quote(self.args.port)} "
            f"--baudrate {int(self.args.baudrate)}"
        )
        ssh_remote_cmd = f"bash -lc {shlex.quote(remote_cmd)}"
        ssh_options = [
            "-o",
            "ServerAliveInterval=10",
            "-o",
            "ServerAliveCountMax=3",
            "-o",
            "TCPKeepAlive=yes",
        ]

        ssh_cmd = ["ssh", *ssh_options, self.args.host, ssh_remote_cmd]
        env = os.environ.copy()

        if self.args.password:
            env["SSHPASS"] = self.args.password
            ssh_cmd = [
                "sshpass",
                "-e",
                "ssh",
                *ssh_options,
                "-o",
                "PreferredAuthentications=password",
                "-o",
                "PubkeyAuthentication=no",
                self.args.host,
                ssh_remote_cmd,
            ]

        proc = subprocess.Popen(
            ssh_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env,
        )

        threading.Thread(target=self._read_stream, args=(proc.stdout, "stdout"), daemon=True).start()
        threading.Thread(target=self._read_stream, args=(proc.stderr, "stderr"), daemon=True).start()
        return proc

    def _schedule_reconnect(self, reason: str) -> None:
        if self.closed:
            return
        if self.next_reconnect_at and time.monotonic() < self.next_reconnect_at:
            return
        self.remote_ready.clear()
        self.remote_failed.set()
        self.remote_error = reason
        self.status_var.set(f"SSH bridge stopped, reconnecting... ({reason})")
        self.next_reconnect_at = time.monotonic() + self.reconnect_delay_s

    def _restart_remote_bridge_if_needed(self) -> None:
        if self.closed or self.proc.poll() is None:
            return
        if time.monotonic() < self.next_reconnect_at:
            return

        try:
            if self.proc.stdin is not None:
                self.proc.stdin.close()
        except OSError:
            pass

        self.remote_ready.clear()
        self.remote_failed.clear()
        self.remote_error = ""
        self.next_reconnect_at = 0.0
        self.status_var.set(f"Reconnecting to {self.args.host} ...")
        self.proc = self._start_remote_bridge()

    def _read_stream(self, stream, stream_name: str) -> None:
        for line in stream:
            text = line.strip()
            if not text:
                continue
            self.status_queue.put((stream_name, text))
        stream.close()

    def _pump_status_queue(self) -> None:
        while True:
            try:
                stream_name, text = self.status_queue.get_nowait()
            except queue.Empty:
                break

            if stream_name == "stdout" and text == "READY":
                self.remote_ready.set()
                self.next_reconnect_at = 0.0
                self.status_var.set(f"Connected to {self.args.host}")
            elif stream_name == "stderr":
                self.remote_failed.set()
                self.remote_error = text
                self.status_var.set(f"Remote error: {text}")
            elif text:
                self.status_var.set(text)

        if not self.closed:
            self.root.after(50, self._pump_status_queue)

    def _speed_text(self) -> str:
        return f"speed={self.speed}  turn_speed={self.turn_speed}"

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()

        if key == "minus":
            self.close()
            return

        if key == "space":
            self.pressed_keys.clear()
            return

        first_press = key not in self.pressed_keys
        if key in {"w", "a", "s", "d"}:
            self.pressed_keys.add(key)

        if not first_press:
            return

        if key == "e":
            self.speed = min(255, self.speed + 5)
        elif key == "q":
            self.speed = max(0, self.speed - 5)
        elif key == "p":
            self.turn_speed = min(255, self.turn_speed + 5)
        elif key == "o":
            self.turn_speed = max(0, self.turn_speed - 5)

        self.speed_var.set(self._speed_text())

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        self.pressed_keys.discard(key)

    def _send_command(self, command: str) -> None:
        if self.proc.stdin is None or self.proc.poll() is not None:
            if not self.closed:
                self._schedule_reconnect(self.remote_error or "ssh exited")
            return

        try:
            self.proc.stdin.write(command + "\n")
            self.proc.stdin.flush()
        except OSError as exc:
            self._schedule_reconnect(f"write failed: {exc}")

    def _tick(self) -> None:
        if self.closed:
            return

        if self.proc.poll() is not None:
            exit_reason = self.remote_error or f"ssh exited with code {self.proc.returncode}"
            self._schedule_reconnect(exit_reason)
            self._restart_remote_bridge_if_needed()

        drive = 0
        if "w" in self.pressed_keys and "s" not in self.pressed_keys:
            drive = 1
        elif "s" in self.pressed_keys and "w" not in self.pressed_keys:
            drive = -1

        steer = 0
        if "a" in self.pressed_keys and "d" not in self.pressed_keys:
            steer = 1
        elif "d" in self.pressed_keys and "a" not in self.pressed_keys:
            steer = -1

        left, right = tank_mix(drive, steer, self.speed, self.turn_speed)
        command = "STOP" if left == 0 and right == 0 else f"BOTH {left} {right}"

        self.command_var.set(f"cmd=({left}, {right})")
        self.speed_var.set(self._speed_text())

        now = time.monotonic()
        if self.remote_ready.is_set() and (command != self.last_command or now - self.last_sent_at >= self.args.send_period):
            self._send_command(command)
            self.last_command = command
            self.last_sent_at = now

        self.root.after(20, self._tick)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True

        try:
            self._send_command("STOP")
        except Exception:
            pass

        if self.proc.stdin is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass

        try:
            self.proc.terminate()
        except OSError:
            pass

        self.root.after(20, self.root.destroy)

    def run(self) -> int:
        self.root.mainloop()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Desktop keyboard teleop that forwards commands to Pi over SSH.")
    parser.add_argument("--host", required=True, help="SSH host for the Pi")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Mega serial device on the Pi")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--speed", type=int, default=90, help="Forward/reverse PWM magnitude (0-255)")
    parser.add_argument("--turn-speed", type=int, default=55, help="Steering PWM magnitude (0-255)")
    parser.add_argument("--send-period", type=float, default=0.03, help="Seconds between repeated drive commands")
    parser.add_argument(
        "--remote-repo",
        default="~/Mekatronikk-4-MEPA2002",
        help="Repo path on the Pi where the helper script exists",
    )
    parser.add_argument(
        "--password",
        default=os.environ.get("MEGA_PI_PASSWORD", ""),
        help="SSH password for the Pi. If omitted, reads MEGA_PI_PASSWORD from the environment.",
    )
    args = parser.parse_args()

    try:
        app = MegaKeyboardGui(args)
        return app.run()
    except FileNotFoundError as exc:
        messagebox.showerror("Mega Teleop", f"Missing executable: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
