#!/usr/bin/env python3
"""HX-30HM live telemetry dashboard.

Reads CRC-protected snapshots emitted by the dead-man FPGA image and serves a
local dashboard. Control commands are CRC protected and must be refreshed
while RUN is held; the FPGA stops independently after 300 ms of host silence.

    python3 tools/telemetry_dashboard.py
    python3 tools/telemetry_dashboard.py --port /dev/cu.usbserial-...
    python3 tools/telemetry_dashboard.py --demo
"""

from __future__ import annotations

import argparse
import csv
import glob
import io
import json
import math
import os
import threading
import time
import webbrowser
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


OP_TELEMETRY = 0x81
OP_CONTROL = 0x10
PAYLOAD_LEN = 58
STATUS_NAMES = (
    "over-voltage", "under-voltage", "over-temperature",
    "over-current", "overload", "instruction-error",
)
POLL_FIELDS = 21


def crc8_byte(crc: int, data: int) -> int:
    crc ^= data
    for _ in range(8):
        crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameParser:
    """Incremental AA 55 | LEN | SEQ | OP | payload | CRC parser."""

    def __init__(self) -> None:
        self.state = 0
        self.length = self.seq = self.op = self.crc = self.index = 0
        self.payload = bytearray()
        self.crc_errors = 0

    def feed(self, byte: int):
        if self.state == 0:
            if byte == 0xAA:
                self.state = 1
        elif self.state == 1:
            if byte == 0x55:
                self.state = 2
            elif byte != 0xAA:
                self.state = 0
        elif self.state == 2:
            if byte > 64:
                self.state = 0
                return None
            self.length = byte
            self.payload = bytearray()
            self.crc = crc8_byte(0, byte)
            self.state = 3
        elif self.state == 3:
            self.seq = byte
            self.crc = crc8_byte(self.crc, byte)
            self.state = 4
        elif self.state == 4:
            self.op = byte
            self.crc = crc8_byte(self.crc, byte)
            self.state = 6 if self.length == 0 else 5
        elif self.state == 5:
            self.payload.append(byte)
            self.crc = crc8_byte(self.crc, byte)
            if len(self.payload) == self.length:
                self.state = 6
        elif self.state == 6:
            self.state = 0
            if byte == self.crc:
                return self.seq, self.op, bytes(self.payload)
            self.crc_errors += 1
        return None


def u16(p: bytes, offset: int) -> int:
    return p[offset] | p[offset + 1] << 8


def u32(p: bytes, offset: int) -> int:
    return u16(p, offset) | u16(p, offset + 2) << 16


def sign_magnitude(raw: int) -> int:
    magnitude = raw & 0x7FFF
    return -magnitude if raw & 0x8000 else magnitude


def decode_snapshot(seq: int, p: bytes) -> dict:
    if len(p) != PAYLOAD_LEN or p[0] != 1:
        raise ValueError(f"unsupported telemetry payload: version/length {p[:1]!r}/{len(p)}")

    flags = p[1]
    cstate = p[2]
    status = p[31]
    mode = {0: "POSITION", 1: "CONSTANT SPEED", 2: "OPEN-LOOP PWM"}.get(
        p[25] & 0x07, f"UNKNOWN {p[25]}"
    )
    mstate = cstate & 0x03
    macro_stop = bool(cstate & 0x04)
    step = (cstate >> 3) & 0x03
    polling = bool(cstate & 0x20)
    if mstate == 0:
        controller = "IDLE"
    elif polling:
        controller = "POLLING"
    else:
        controller = f"{'STOP' if macro_stop else 'START'} STEP {step + 1}/4"

    valid_mask = u32(p, 8)
    faults = [name for bit, name in enumerate(STATUS_NAMES) if status & (1 << bit)]
    return {
        "host_time": time.time(),
        "seq": seq,
        "schema": p[0],
        "uptime_ms": u32(p, 4),
        "s1_held": bool(flags & 0x01),
        "s2_held": bool(flags & 0x02),
        "reverse": bool(flags & 0x04),
        "hold_qualified": bool(flags & 0x08),
        "fpga_moving": bool(flags & 0x10),
        "click_pending": bool(flags & 0x20),
        "stop_pending": bool(flags & 0x40),
        "servo_bus_busy": bool(flags & 0x80),
        "controller": controller,
        "controller_raw": cstate,
        "macro_step": step,
        "polling": polling,
        "last_servo_error": p[3],
        "diag_valid_mask": valid_mask,
        "diag_valid_count": bin(valid_mask & ((1 << POLL_FIELDS) - 1)).count("1"),
        "good_replies": u16(p, 12),
        "bad_replies": u16(p, 14),
        "timeout_replies": u16(p, 16),
        "servo_error_replies": u16(p, 18),
        "fw_major": p[20],
        "fw_minor": p[21],
        "servo_id": p[22],
        "baud_idx": p[23],
        "response_level": p[24],
        "control_mode_raw": p[25],
        "control_mode": mode,
        "nvs_locked": bool(p[26]),
        "temperature_limit_c": p[27],
        "max_torque": p[28],
        "protection_control": p[29],
        "temperature_c": p[30],
        "status_bits": status,
        "faults": faults,
        "status": ", ".join(faults) if faults else "OK",
        "servo_moving": bool(p[32]),
        "current_limit_ma": u16(p, 34),
        "target_position": u16(p, 36),
        "run_speed_raw": u16(p, 38),
        "run_speed": sign_magnitude(u16(p, 38)),
        "current_position": u16(p, 40),
        "current_speed_raw": u16(p, 42),
        "current_speed": sign_magnitude(u16(p, 42)),
        "current_load_raw": u16(p, 44),
        "current_load": sign_magnitude(u16(p, 44)),
        "voltage_mv": u16(p, 46),
        "voltage_v": u16(p, 46) / 1000.0,
        "current_ma": u16(p, 48),
        "requested_speed_raw": u16(p, 50),
        "requested_speed": sign_magnitude(u16(p, 50)),
        "hold_ms": u16(p, 52),
        "poll_index": p[54] & 0x1F,
        "selected_servo_id": p[55],
        "emergency_stops": u16(p, 56),
    }


class SharedState:
    def __init__(self, limit: int) -> None:
        self.history = deque(maxlen=limit)
        self.condition = threading.Condition()
        self.version = 0
        self.connected = False
        self.port = ""
        self.error = "waiting for telemetry"
        self.host_crc_errors = 0
        self.host_frame_gaps = 0
        self.commands = deque(maxlen=32)
        self.command_seq = 0

    def queue_control(self, motor: int, reverse: bool, run: bool, estop: bool) -> None:
        if motor not in (1, 2):
            raise ValueError("motor must be 1 or 2")
        flags = (1 if run else 0) | (2 if reverse else 0) | (4 if estop else 0)
        with self.condition:
            seq = self.command_seq
            self.command_seq = (self.command_seq + 1) & 0xFF
            body = bytes((2, seq, OP_CONTROL, motor, flags))
            crc = 0
            for byte in body:
                crc = crc8_byte(crc, byte)
            self.commands.append(b"\xAA\x55" + body + bytes((crc,)))
            self.condition.notify_all()

    def take_commands(self) -> list[bytes]:
        with self.condition:
            queued = list(self.commands)
            self.commands.clear()
            return queued

    def publish(self, sample: dict) -> None:
        with self.condition:
            sample["connected"] = self.connected
            sample["serial_port"] = self.port
            sample["host_crc_errors"] = self.host_crc_errors
            sample["host_frame_gaps"] = self.host_frame_gaps
            self.history.append(sample)
            self.version += 1
            self.condition.notify_all()

    def set_connection(self, connected: bool, port: str = "", error: str = "") -> None:
        with self.condition:
            self.connected, self.port, self.error = connected, port, error
            self.version += 1
            self.condition.notify_all()

    def status(self) -> dict:
        with self.condition:
            return {
                "connected": self.connected,
                "serial_port": self.port,
                "error": self.error,
                "host_crc_errors": self.host_crc_errors,
                "host_frame_gaps": self.host_frame_gaps,
                "latest": self.history[-1] if self.history else None,
            }


def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise FileNotFoundError("no USB serial port found")
    return ports[-1]  # FT2232 channel B has the higher suffix on this dock


def serial_worker(state: SharedState, explicit_port: str | None, stop: threading.Event) -> None:
    if serial is None:
        state.set_connection(False, error="pyserial is missing; install it with: python3 -m pip install pyserial")
        return
    last_seq = None
    while not stop.is_set():
        try:
            port = find_port(explicit_port)
            with serial.Serial(port, 115200, timeout=0.02) as device:
                device.reset_input_buffer()
                state.set_connection(True, port=port, error="")
                parser = FrameParser()
                while not stop.is_set():
                    for command in state.take_commands():
                        device.write(command)
                    device.flush()
                    block = device.read(512)
                    for byte in block:
                        frame = parser.feed(byte)
                        state.host_crc_errors = parser.crc_errors
                        if not frame:
                            continue
                        seq, op, payload = frame
                        if op != OP_TELEMETRY:
                            continue
                        if last_seq is not None:
                            expected = (last_seq + 1) & 0xFF
                            state.host_frame_gaps += (seq - expected) & 0xFF
                        last_seq = seq
                        state.publish(decode_snapshot(seq, payload))
        except Exception as exc:
            state.set_connection(False, error=str(exc))
            stop.wait(1.0)


def demo_worker(state: SharedState, stop: threading.Event) -> None:
    seq = 0
    started = time.monotonic()
    state.set_connection(True, port="DEMO", error="")
    while not stop.wait(0.1):
        t = time.monotonic() - started
        phase = t % 12
        held = 2.0 <= phase < 6.0
        reverse = (int(t // 12) & 1) == 1
        direction = -1 if reverse else 1
        qualified = 2.5 <= phase < 6.0
        moving = qualified
        speed = direction * (3400 if moving else 0)
        pos = int(2048 + direction * 280 * max(0, min(phase - 2.5, 3.5)) / 3.5)
        current = int(180 + (520 if moving else 0) + 80 * math.sin(t * 2.2))
        sample = {
            "host_time": time.time(), "seq": seq, "schema": 1,
            "uptime_ms": int(t * 1000), "s1_held": held, "s2_held": False,
            "reverse": reverse, "hold_qualified": qualified,
            "fpga_moving": moving, "click_pending": False,
            "stop_pending": False, "servo_bus_busy": int(t * 20) % 3 == 0,
            "controller": "IDLE" if not moving else "IDLE",
            "controller_raw": 0, "macro_step": 0, "polling": False,
            "last_servo_error": 0, "diag_valid_mask": 0x1FFFFF,
            "diag_valid_count": 21, "good_replies": int(t * 200) & 0xFFFF,
            "bad_replies": 0, "timeout_replies": 0, "servo_error_replies": 0,
            "fw_major": 1, "fw_minor": 7, "servo_id": 1, "baud_idx": 0,
            "response_level": 1, "control_mode_raw": 1 if moving else 0,
            "control_mode": "CONSTANT SPEED" if moving else "POSITION",
            "nvs_locked": True, "temperature_limit_c": 70, "max_torque": 100,
            "protection_control": 31, "temperature_c": int(35 + 2 * math.sin(t / 8)),
            "status_bits": 0, "faults": [], "status": "OK",
            "servo_moving": moving, "current_limit_ma": 3000,
            "target_position": 2048, "run_speed_raw": abs(speed) | (0x8000 if speed < 0 else 0),
            "run_speed": speed, "current_position": pos,
            "current_speed_raw": abs(speed) | (0x8000 if speed < 0 else 0),
            "current_speed": speed, "current_load_raw": 110 if moving else 20,
            "current_load": direction * (110 if moving else 20),
            "voltage_mv": int(11700 - current * 0.12), "voltage_v": (11700 - current * 0.12) / 1000,
            "current_ma": current, "requested_speed_raw": abs(speed) | (0x8000 if speed < 0 else 0),
            "requested_speed": speed, "hold_ms": int(max(0, phase - 2) * 1000) if held else 0,
            "poll_index": int(t * 20) % 21,
            "selected_servo_id": 1 if int(t // 12) % 2 == 0 else 2,
            "emergency_stops": 0,
        }
        state.publish(sample)
        seq = (seq + 1) & 0xFF


def make_handler(state: SharedState, html_path: Path):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send_bytes(self, data: bytes, content_type: str, status=HTTPStatus.OK) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                self.send_bytes(html_path.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.send_bytes(json.dumps(state.status()).encode(), "application/json")
            elif path == "/api/history":
                with state.condition:
                    data = list(state.history)
                self.send_bytes(json.dumps(data, separators=(",", ":")).encode(), "application/json")
            elif path == "/api/export.csv":
                with state.condition:
                    rows = list(state.history)
                fields = sorted({key for row in rows for key in row if key != "faults"})
                output = io.StringIO()
                writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
                for row in rows:
                    writer.writerow(row)
                self.send_bytes(output.getvalue().encode(), "text/csv; charset=utf-8")
            elif path == "/events":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                version = -1
                try:
                    while True:
                        with state.condition:
                            state.condition.wait_for(lambda: state.version != version, timeout=5.0)
                            version = state.version
                            message = state.status()
                        wire = f"data:{json.dumps(message, separators=(',', ':'))}\n\n".encode()
                        self.wfile.write(wire)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_bytes(b"not found\n", "text/plain", HTTPStatus.NOT_FOUND)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            if path != "/api/control":
                self.send_bytes(b"not found\n", "text/plain", HTTPStatus.NOT_FOUND)
                return
            # A custom header forces cross-origin sites through CORS preflight;
            # this localhost-only service deliberately grants no CORS access.
            if self.headers.get("X-HX-Control") != "1":
                self.send_bytes(b"control header required\n", "text/plain", HTTPStatus.FORBIDDEN)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 1024:
                    raise ValueError("invalid request size")
                request = json.loads(self.rfile.read(length))
                state.queue_control(
                    int(request.get("motor", 1)), bool(request.get("reverse", False)),
                    bool(request.get("run", False)), bool(request.get("estop", False)),
                )
                self.send_bytes(b'{"ok":true}', "application/json")
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.send_bytes(json.dumps({"ok": False, "error": str(exc)}).encode(),
                                "application/json", HTTPStatus.BAD_REQUEST)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB UART path (default: auto-detect channel B)")
    parser.add_argument("--http-port", type=int, default=8765, help="local dashboard port")
    parser.add_argument("--history", type=int, default=36000, help="samples retained for charts/CSV")
    parser.add_argument("--demo", action="store_true", help="run with generated data instead of hardware")
    parser.add_argument("--no-browser", action="store_true", help="do not open the dashboard automatically")
    args = parser.parse_args()

    state = SharedState(max(100, args.history))
    stop = threading.Event()
    worker = threading.Thread(
        target=demo_worker if args.demo else serial_worker,
        args=(state, stop) if args.demo else (state, args.port, stop),
        daemon=True,
    )
    worker.start()

    html_path = Path(__file__).with_name("telemetry_dashboard.html")
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(state, html_path))
    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"HX-30HM dashboard: {url}")
    print("Telemetry + hold-to-run control; FPGA host dead-man timeout is 300 ms.")
    if not args.no_browser:
        threading.Timer(0.25, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            latest = state.status().get("latest") or {}
            state.queue_control(int(latest.get("selected_servo_id", 1)), False, False, True)
            time.sleep(0.08)
        except Exception:
            pass
        stop.set()
        server.server_close()


if __name__ == "__main__":
    main()
