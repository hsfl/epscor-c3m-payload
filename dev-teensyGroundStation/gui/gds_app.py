#!/usr/bin/env python3
"""
Ground Station Web GUI
EPSCOR C3M Payload - Ground Station Component

Flask app that mirrors ground_station_serial_cli_teensy.py's functionality
(connect to Teensy over serial, send commands, capture/request/export
thermal + rpicam data) as a localhost web UI with live progress and
side-by-side image viewers.

Run with: python gds_app.py
"""

import io
import json
import os
import re
import sys
import threading
import time
from datetime import datetime
from queue import Queue, Empty

import numpy as np
import serial
from flask import Flask, Response, jsonify, render_template, request, send_from_directory
from PIL import Image
from serial.tools import list_ports

# matplotlib must use a non-interactive backend since this runs headless
# inside a Flask worker thread, not the main thread.
import matplotlib
matplotlib.use("Agg")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ground_station_serial_cli_teensy as gs_cli  # noqa: E402
from thermal_data_viewer import normalize_thermal_grid  # noqa: E402

BAUD_RATE = gs_cli.BAUD_RATE

# gs_cli.CAPTURES_DIR defaults to the relative string "captures", which
# resolves against this process's CWD (ground_station_gui/) rather than the
# dev-teensyGroundStation/ dir the CLI writes to. Pin it to an absolute path
# so captures land next to the CLI's, regardless of where gds_app.py is
# launched from.
gs_cli.CAPTURES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "captures"
)

ROTATION_STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rotation_state.json")
# One slot per physical camera - lepton and boson are separate sensors on
# separate mounts, so each needs its own rotation and its own panel rather
# than sharing a single "thermal" slot.
VIEWER_SOURCES = ("rpicam", "lepton", "boson")


def load_rotation_state():
    if os.path.exists(ROTATION_STATE_PATH):
        try:
            with open(ROTATION_STATE_PATH, "r") as f:
                data = json.load(f)
                return {src: int(data.get(src, 0)) % 360 for src in VIEWER_SOURCES}
        except (json.JSONDecodeError, OSError, ValueError):
            pass
    return {src: 0 for src in VIEWER_SOURCES}


def save_rotation_state(state):
    try:
        with open(ROTATION_STATE_PATH, "w") as f:
            json.dump(state, f)
    except OSError:
        pass


class EventBus:
    """Broadcasts JSON events to all connected SSE clients."""

    def __init__(self):
        self._subscribers = []
        self._lock = threading.Lock()

    def subscribe(self):
        q = Queue()
        with self._lock:
            self._subscribers.append(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            if q in self._subscribers:
                self._subscribers.remove(q)

    def publish(self, event, data):
        payload = json.dumps({"event": event, "data": data})
        with self._lock:
            subs = list(self._subscribers)
        for q in subs:
            q.put(payload)


bus = EventBus()

REQUEST_PROGRESS_RE = re.compile(r"(\d{1,3})%\s*$")
CAPTURE_STATUS_RE = re.compile(r"\b(CAPTURE_DONE|CAPTURE_ERROR|NO_FRAME)\b")


def temperature_range(rows):
    """Return finite Celsius extrema from converted Lepton CSV rows."""
    values = np.loadtxt(io.StringIO("\n".join(rows)), delimiter=",")
    finite = values[np.isfinite(values)]
    if not finite.size:
        return None
    return {"min": float(finite.min()), "max": float(finite.max())}


class GroundStationSession:
    """Owns the serial connection and turns incoming text into web events.

    Mirrors the text-capture state machine in
    ground_station_serial_cli_teensy.process_text_data(), but pushes
    structured events (log lines, capture/request progress, image-ready)
    instead of printing to a terminal / popping up matplotlib windows.
    """

    def __init__(self):
        self.ser = None
        self.port = None
        self.read_thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.raw_buffer = bytearray()
        self.text_buffer = bytearray()
        self.stream_can_close = False
        self.stream_state = "stopped"
        self.last_stream = None

        self.csv_capture_mode = False
        self.csv_data = []
        self.jpg_capture_mode = False
        self.jpg_data = []
        self.pending_export_source = None
        self.current_csv_source = None
        self.current_jpg_source = None
        self.capture_timestamp = datetime.now()

        self.capture_state = "idle"  # idle | capturing | done | error
        self.request_state = "idle"  # idle | requesting | downloading | done | error
        self.request_percent = 0

        # Latest rendered-image source data, kept in memory so rotation
        # changes can be re-rendered without a new capture.
        self.last_thermal = {"lepton": None, "boson": None}  # each: {"rows": [...], "ts": iso, "filename": str}
        self.last_rpicam = None  # {"bytes": jpg_bytes, "ts": iso}

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port):
        with self.lock:
            if self.connected:
                raise RuntimeError("Already connected")
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.2)
            self.port = port
            self.raw_buffer.clear()
            self.text_buffer.clear()
            self.last_stream = None
            self.set_stream_state("stopped")
            self.stop_event.clear()
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
        bus.publish("connection", {"connected": True, "port": port})

    def disconnect(self):
        with self.lock:
            self.stop_event.set()
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=2.0)
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None
            self.port = None
        self.set_stream_state("disconnected")
        bus.publish("connection", {"connected": False, "port": None})

    def send_command(self, cmd):
        if not self.connected:
            raise RuntimeError("Not connected")
        self.ser.write((cmd + "\n").encode())
        parts = cmd.lower().split()
        if parts in (["stream", "start"], ["stream", "1"]):
            self.last_stream = None
            self.set_stream_state("waiting")
        elif parts in (["stream", "stop"], ["stream", "0"]):
            self.set_stream_state("stopping")

    def _read_loop(self):
        lost_connection = False
        while not self.stop_event.is_set():
            try:
                raw = self.ser.read(max(1, min(self.ser.in_waiting, 16384)))
            except (serial.SerialException, OSError):
                lost_connection = True
                break
            if raw:
                self._process_bytes(raw)

        if lost_connection:
            # Reached only on an unexpected serial error, never on a normal
            # disconnect() (which sets stop_event first) - safe to tear down
            # the port here without re-entering disconnect()'s thread join.
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
            self.port = None
            bus.publish("connection", {"connected": False, "port": None})
            self.set_stream_state("disconnected")
            bus.publish("log", {"text": "[connection lost]"})

    def set_stream_state(self, state, can_close=False):
        self.stream_state = state
        self.stream_can_close = can_close
        bus.publish("stream_state", {"state": state, "can_close": can_close})

    def _process_text(self, data):
        self.text_buffer.extend(data)
        while b"\n" in self.text_buffer:
            line, _, rest = self.text_buffer.partition(b"\n")
            self.text_buffer = bytearray(rest)
            text = line.decode("utf-8", errors="replace").rstrip("\r")
            if text:
                self._handle_line(text)

    def _process_bytes(self, data):
        """Demultiplex serial text and fixed-size WRM! frames across reads."""
        self.raw_buffer.extend(data)
        magic = gs_cli.STREAM_FRAME_MAGIC
        while self.raw_buffer:
            pos = self.raw_buffer.find(magic)
            if pos < 0:
                # Retain only a possible partial magic prefix, even across timeouts.
                keep = next((n for n in range(len(magic) - 1, 0, -1)
                             if self.raw_buffer.endswith(magic[:n])), 0)
                count = len(self.raw_buffer) - keep
                self._process_text(self.raw_buffer[:count])
                del self.raw_buffer[:count]
                return
            if pos:
                self._process_text(self.raw_buffer[:pos])
                del self.raw_buffer[:pos]
            if len(self.raw_buffer) < gs_cli.STREAM_FRAME_TOTAL_SIZE:
                return
            packet = bytes(self.raw_buffer[:gs_cli.STREAM_FRAME_TOTAL_SIZE])
            del self.raw_buffer[:gs_cli.STREAM_FRAME_TOTAL_SIZE]
            seq, pkts = packet[4:6]
            pixels = packet[8:]
            checksum = int.from_bytes(packet[6:8], "little")
            if (not gs_cli.STREAM_PACKETS_PER_FRAME // 2 <= pkts <= gs_cli.STREAM_PACKETS_PER_FRAME
                    or sum(pixels) & 0xFFFF != checksum):
                bus.publish("log", {"text": "[stream] Corrupt frame skipped"})
                continue
            if self.stream_state in ("stopping", "stopped", "disconnected"):
                continue
            grid = np.frombuffer(pixels, dtype=np.uint8).reshape(
                gs_cli.STREAM_FRAME_HEIGHT, gs_cli.STREAM_FRAME_WIDTH)
            # Match the still thermal viewer's hot palette, using the stream's
            # fixed 8-bit intensity range so colors remain stable across frames.
            colors = matplotlib.colormaps["hot"](grid / 255.0, bytes=True)
            image = Image.fromarray(colors[:, :, :3])
            out = io.BytesIO()
            image.save(out, format="PNG")
            self.last_stream = {"png": out.getvalue(), "seq": seq, "packets": pkts,
                                "ts": datetime.now().isoformat()}
            if self.stream_state != "live":
                self.set_stream_state("live")
            bus.publish("stream_frame", {k: v for k, v in self.last_stream.items() if k != "png"})

    def _handle_line(self, line):
        bus.publish("log", {"text": line})
        gs_cli.update_sensor_cache(line)
        if "Stream start command sent" in line:
            self.set_stream_state("waiting")
        elif "Stream stop command sent" in line:
            self.set_stream_state("stopping")
        elif self.stream_state == "stopping" and re.search(r"\bRPI STATUS:\s*IDLE\b", line):
            self.set_stream_state("stopped", can_close=True)
        elif "Failed to send stream" in line:
            self.set_stream_state("error")

        if gs_cli.THERMAL_CAPTURE_TIMESTAMP_FLAG in line:
            self.capture_timestamp = datetime.now()

        if "--- UART CAPTURE ---" in line:
            self.capture_state = "capturing"
            bus.publish("capture_progress", {"phase": "start"})
        elif "--- UART CAPTURE COMPLETE ---" in line:
            self.capture_state = "done"
            bus.publish("capture_progress", {"phase": "done"})
        else:
            m = CAPTURE_STATUS_RE.search(line)
            if m:
                if m.group(1) in ("CAPTURE_ERROR", "NO_FRAME"):
                    self.capture_state = "error"
                bus.publish("capture_progress", {"phase": "status", "text": line})

        if "--- REQUESTING THERMAL DATA DOWNLINK ---" in line:
            self.request_state = "requesting"
            self.request_percent = 0
            bus.publish("request_progress", {"phase": "requesting", "percent": 0})
        elif "THERMAL IMAGE HEADER RECEIVED" in line:
            self.request_state = "downloading"
            bus.publish("request_progress", {"phase": "downloading", "percent": self.request_percent})
        elif "Invalid header!" in line or "Failed to send command" in line:
            self.request_state = "error"
            bus.publish("request_progress", {"phase": "error", "percent": self.request_percent})
        else:
            m = REQUEST_PROGRESS_RE.search(line.strip())
            if m and self.request_state == "downloading":
                self.request_percent = min(100, int(m.group(1)))
                bus.publish("request_progress", {"phase": "downloading", "percent": self.request_percent})

        matched_source = next(
            (src for tag, src in gs_cli.EXPORT_SOURCE_TAGS.items() if tag in line),
            "__no_match__",
        )
        if matched_source != "__no_match__":
            self.pending_export_source = matched_source
            return

        if gs_cli.THERMAL_CSV_START in line:
            self.csv_capture_mode = True
            self.csv_data.clear()
            self.current_csv_source = self.pending_export_source
            self.pending_export_source = None
            if gs_cli.THERMAL_CAPTURE_TIMESTAMP_FLAG not in line:
                self.capture_timestamp = datetime.now()
            return

        if gs_cli.THERMAL_CSV_END in line:
            if self.csv_capture_mode:
                self.csv_capture_mode = False
                self._finish_csv()
            return

        if self.csv_capture_mode:
            clean = line.replace("Teensy: ", "").replace("SAT> ", "").rstrip("\r\n")
            if clean:
                self.csv_data.append(clean)
            return

        if gs_cli.THERMAL_JPG_START in line:
            self.jpg_capture_mode = True
            self.jpg_data.clear()
            self.current_jpg_source = self.pending_export_source
            self.pending_export_source = None
            return

        if gs_cli.THERMAL_JPG_END in line:
            if self.jpg_capture_mode:
                self.jpg_capture_mode = False
                self._finish_jpg()
            return

        if self.jpg_capture_mode:
            clean = line.replace("Teensy: ", "").replace("SAT> ", "").rstrip("\r\n")
            if clean:
                self.jpg_data.append(clean)
            return

    def _finish_csv(self):
        source = self.current_csv_source
        rows = list(self.csv_data)
        self.csv_data.clear()
        self.current_csv_source = None

        output_rows = gs_cli.lepton_rows_to_celsius(rows) if source == "lepton" else rows
        metadata_lines = gs_cli.build_sensor_metadata_lines(self.capture_timestamp)
        if metadata_lines:
            raw_content = "\n".join(metadata_lines + [""] + output_rows)
        else:
            raw_content = "\n".join(output_rows)

        filename = gs_cli.get_next_thermal_filename(source)
        gs_cli.save_thermal_data(raw_content, filename)

        self.request_state = "done"
        bus.publish("request_progress", {"phase": "done", "percent": 100})

        if source not in ("lepton", "boson"):
            # Legacy/untagged capture (no 'request <id>' issued this
            # session) - saved to disk above, but there's no sensor-specific
            # panel to route it to.
            bus.publish("log", {"text": f"[saved {filename}, unknown camera source - no viewer to show it in]"})
            return

        self.last_thermal[source] = {
            "rows": output_rows,
            "ts": datetime.now().isoformat(),
            "filename": filename,
            "temperatures": temperature_range(output_rows) if source == "lepton" else None,
        }
        bus.publish("image_ready", {"viewer": source, "source": source, "filename": filename,
                                    "temperatures": self.last_thermal[source]["temperatures"]})

    def _finish_jpg(self):
        import base64

        source = self.current_jpg_source
        b64_data = "".join(self.jpg_data)
        self.jpg_data.clear()
        self.current_jpg_source = None

        try:
            image_bytes = base64.b64decode(b64_data, validate=True)
            if image_bytes[:2] != b"\xff\xd8" or image_bytes[-2:] != b"\xff\xd9":
                raise ValueError("decoded bytes are not a valid JPEG")
        except Exception as e:
            bus.publish("log", {"text": f"Error decoding image: {e}"})
            self.request_state = "error"
            bus.publish("request_progress", {"phase": "error", "percent": self.request_percent})
            return

        filename = gs_cli.get_next_image_filename(source)
        with open(filename, "wb") as f:
            f.write(image_bytes)

        self.last_rpicam = {"bytes": image_bytes, "ts": datetime.now().isoformat(), "filename": filename}
        self.request_state = "done"
        bus.publish("request_progress", {"phase": "done", "percent": 100})
        bus.publish("image_ready", {"viewer": "rpicam", "source": source, "filename": filename})


session = GroundStationSession()
rotation_state = load_rotation_state()

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/ports")
def api_ports():
    ports = [{"device": p.device, "description": p.description or ""} for p in list_ports.comports()]
    default = gs_cli.find_serial_port()
    return jsonify({"ports": ports, "default": default})


@app.route("/api/connect", methods=["POST"])
def api_connect():
    port = (request.get_json(silent=True) or {}).get("port")
    if not port:
        return jsonify({"error": "port is required"}), 400
    try:
        session.connect(port)
    except (serial.SerialException, RuntimeError) as e:
        return jsonify({"error": str(e)}), 400
    return jsonify({"ok": True, "port": port})


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    session.disconnect()
    return jsonify({"ok": True})


@app.route("/api/send", methods=["POST"])
def api_send():
    cmd = (request.get_json(silent=True) or {}).get("cmd", "").strip()
    if not cmd:
        return jsonify({"error": "cmd is required"}), 400
    try:
        session.send_command(cmd)
    except RuntimeError as e:
        return jsonify({"error": str(e)}), 400
    return jsonify({"ok": True})


@app.route("/api/state")
def api_state():
    return jsonify(
        {
            "connected": session.connected,
            "port": session.port,
            "capture_state": session.capture_state,
            "request_state": session.request_state,
            "request_percent": session.request_percent,
            "rotation": rotation_state,
            "lepton_temperatures": (session.last_thermal["lepton"].get("temperatures")
                                    if session.last_thermal["lepton"] else None),
            "stream_state": session.stream_state,
            "stream_can_close": session.stream_can_close,
            "stream_frame": ({k: v for k, v in session.last_stream.items() if k != "png"}
                             if session.last_stream else None),
            "viewers": {
                "rpicam": session.last_rpicam is not None,
                "lepton": session.last_thermal["lepton"] is not None,
                "boson": session.last_thermal["boson"] is not None,
            },
        }
    )


@app.route("/api/rotate", methods=["POST"])
def api_rotate():
    body = request.get_json(silent=True) or {}
    viewer = body.get("viewer")
    delta = int(body.get("delta", 0))
    if viewer not in VIEWER_SOURCES:
        return jsonify({"error": f"viewer must be one of {VIEWER_SOURCES}"}), 400
    rotation_state[viewer] = (rotation_state[viewer] + delta) % 360
    save_rotation_state(rotation_state)
    has_image = (viewer == "rpicam" and session.last_rpicam) or (
        viewer in ("lepton", "boson") and session.last_thermal[viewer]
    )
    if has_image:
        bus.publish("image_ready", {"viewer": viewer, "source": viewer, "rotated": True})
    return jsonify({"ok": True, "rotation": rotation_state[viewer]})


def _render_thermal_png(entry, source, degrees):
    rows = entry["rows"]
    data = np.loadtxt(io.StringIO("\n".join(rows)), delimiter=",")

    k = (-(degrees // 90)) % 4  # clockwise rotation to match rpicam's visual sense
    data = np.rot90(data, k=k)

    display_data = data if source == "lepton" else normalize_thermal_grid(data)
    # Preserve the plot's automatic contrast and hot palette, without axes,
    # margins, or a colorbar: one output pixel per thermal sample.
    normalized = matplotlib.colors.Normalize()(display_data)
    colors = matplotlib.colormaps["hot"](normalized, bytes=True)
    image = Image.fromarray(colors[:, :, :3])
    buf = io.BytesIO()
    image.save(buf, format="PNG")
    return buf.getvalue()


def _render_rpicam_png(entry, degrees):
    img = Image.open(io.BytesIO(entry["bytes"]))
    if degrees:
        img = img.rotate(-degrees, expand=True)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    return buf.getvalue()


@app.route("/api/image/<viewer>")
def api_image(viewer):
    if viewer in ("lepton", "boson"):
        entry = session.last_thermal[viewer]
        if not entry:
            return "", 404
        png_bytes = _render_thermal_png(entry, viewer, rotation_state[viewer])
    elif viewer == "rpicam":
        if not session.last_rpicam:
            return "", 404
        png_bytes = _render_rpicam_png(session.last_rpicam, rotation_state["rpicam"])
    else:
        return "", 404
    return Response(png_bytes, mimetype="image/png")


@app.route("/api/stream/image")
def api_stream_image():
    frame = session.last_stream
    if frame is None:
        return "", 404
    degrees = request.args.get("rotation", default=0, type=int)
    if degrees not in (0, 90, 180, 270):
        return jsonify({"error": "rotation must be 0, 90, 180, or 270"}), 400
    png = frame["png"]
    if degrees:
        image = Image.open(io.BytesIO(png)).rotate(-degrees, expand=True)
        out = io.BytesIO()
        image.save(out, format="PNG")
        png = out.getvalue()
    return Response(png, mimetype="image/png", headers={"Cache-Control": "no-store"})


@app.route("/api/events")
def api_events():
    q = bus.subscribe()

    def stream():
        try:
            yield "retry: 2000\n\n"
            while True:
                try:
                    payload = q.get(timeout=15)
                    yield f"data: {payload}\n\n"
                except Empty:
                    yield ": keepalive\n\n"
        finally:
            bus.unsubscribe(q)

    return Response(stream(), mimetype="text/event-stream")


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5050, debug=False, threaded=True)
