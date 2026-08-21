#!/usr/bin/env python3
"""
Ground Station Web GUI - Outreach Demo Edition
EPSCOR C3M Payload - Ground Station Component

A separate copy of ground_station_gui/gds_app.py for public outreach events:
same connect/capture/request/export flow, plus combining the rpicam + Lepton
images into one keepsake PNG (with HSFL/C3M logos in the corners) and
emailing it to a visitor. Kept as a standalone copy rather than sharing code
with the production GUI, since this one talks to real SMTP credentials and
outbound network calls that the production tool has no reason to carry.

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
from dotenv import load_dotenv
from flask import Flask, Response, jsonify, render_template, request, send_from_directory
from PIL import Image
from serial.tools import list_ports

# matplotlib must use a non-interactive backend since this runs headless
# inside a Flask worker thread, not the main thread.
import matplotlib
matplotlib.use("Agg")
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ground_station_serial_cli_teensy as gs_cli  # noqa: E402
from thermal_data_viewer import normalize_thermal_grid  # noqa: E402

import imaging  # noqa: E402
import outreach  # noqa: E402

load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))

BAUD_RATE = gs_cli.BAUD_RATE

# Outreach captures are kept separate from dev-teensyGroundStation/captures/
# so a booth full of visitors mashing "Capture" doesn't interleave with real
# mission data / numbered capture counters.
gs_cli.CAPTURES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "captures_demo"
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
        bus.publish("connection", {"connected": False, "port": None})

    def send_command(self, cmd):
        if not self.connected:
            raise RuntimeError("Not connected")
        self.ser.write((cmd + "\n").encode())

    def _read_loop(self):
        lost_connection = False
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError):
                lost_connection = True
                break
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="ignore").rstrip("\r\n")
            except UnicodeDecodeError:
                continue
            if line:
                self._handle_line(line)

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
            bus.publish("log", {"text": "[connection lost]"})

    def _handle_line(self, line):
        bus.publish("log", {"text": line})
        gs_cli.update_sensor_cache(line)

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
        }
        bus.publish("image_ready", {"viewer": source, "source": source, "filename": filename})

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
combined_colorbar_enabled = False  # visitor-facing toggle for the keepsake image's temperature scale

app = Flask(__name__)

# matplotlib's pyplot module uses shared global figure state, which isn't
# thread-safe. The combined-preview panel means the browser now regularly
# fires concurrent requests that both render the lepton figure (the
# standalone lepton viewer refresh + the combined-image refresh, from the
# same image_ready event) - serialize figure creation/save/close to avoid a
# race between two threads inside plt.subplots()/savefig()/close().
_thermal_render_lock = threading.Lock()


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


def _combined_ready():
    return bool(session.last_rpicam and session.last_thermal["lepton"])


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
            "viewers": {
                "rpicam": session.last_rpicam is not None,
                "lepton": session.last_thermal["lepton"] is not None,
                "boson": session.last_thermal["boson"] is not None,
                "combined": _combined_ready(),
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


def _thermal_display_data(entry, source, degrees):
    rows = entry["rows"]
    data = np.loadtxt(io.StringIO("\n".join(rows)), delimiter=",")

    k = (-(degrees // 90)) % 4  # clockwise rotation to match rpicam's visual sense
    data = np.rot90(data, k=k)

    is_radiometric = source == "lepton"
    display_data = data if is_radiometric else normalize_thermal_grid(data)
    return display_data, is_radiometric


def _render_thermal_png(entry, source, degrees):
    with _thermal_render_lock:
        display_data, _ = _thermal_display_data(entry, source, degrees)

        fig, ax = plt.subplots(figsize=(5, 4), dpi=110)
        ax.imshow(display_data, cmap="hot", aspect="equal")
        ax.set_axis_off()
        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0)
        plt.close(fig)
        buf.seek(0)
        return buf.getvalue()


def _render_colorbar_png(entry, source, degrees):
    """A standalone colorbar matching the thermal data's value range. Rendered
    separately from the heatmap so its pixel height can be matched exactly to
    the heatmap panel in imaging.py, instead of matplotlib deciding how much
    width/height to give a colorbar embedded in the same figure."""
    with _thermal_render_lock:
        display_data, is_radiometric = _thermal_display_data(entry, source, degrees)
        label = "Temperature (°C)" if is_radiometric else "Intensity (0-255)"

        norm = mcolors.Normalize(vmin=float(display_data.min()), vmax=float(display_data.max()))
        sm = plt.cm.ScalarMappable(norm=norm, cmap="hot")

        fig = plt.figure(figsize=(1.3, 4), dpi=110)
        ax = fig.add_axes((0.3, 0.04, 0.14, 0.92))
        cbar = fig.colorbar(sm, cax=ax)
        cbar.set_label(label, fontsize=9)
        cbar.ax.tick_params(labelsize=8)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0.02)
        plt.close(fig)
        buf.seek(0)
        return buf.getvalue()


def _render_rpicam_png(entry, degrees):
    img = Image.open(io.BytesIO(entry["bytes"]))
    if degrees:
        img = img.rotate(-degrees, expand=True)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    return buf.getvalue()


def _render_combined_png():
    rpicam_png = _render_rpicam_png(session.last_rpicam, rotation_state["rpicam"])
    lepton_png = _render_thermal_png(session.last_thermal["lepton"], "lepton", rotation_state["lepton"])
    colorbar_png = None
    if combined_colorbar_enabled:
        colorbar_png = _render_colorbar_png(session.last_thermal["lepton"], "lepton", rotation_state["lepton"])
    return imaging.compose_visitor_image(rpicam_png, lepton_png, colorbar_png_bytes=colorbar_png)


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
    elif viewer == "combined":
        if not _combined_ready():
            return "", 404
        png_bytes = _render_combined_png()
    else:
        return "", 404
    return Response(png_bytes, mimetype="image/png")


@app.route("/api/outreach/config")
def api_outreach_config():
    return jsonify(
        {
            "email_configured": outreach.email_configured(),
            "website_url": os.environ.get("PROJECT_WEBSITE_URL", ""),
            "logos": imaging.logo_status(),
            "colorbar_enabled": combined_colorbar_enabled,
        }
    )


@app.route("/api/outreach/colorbar", methods=["POST"])
def api_outreach_colorbar():
    global combined_colorbar_enabled
    body = request.get_json(silent=True) or {}
    combined_colorbar_enabled = bool(body.get("enabled"))
    return jsonify({"ok": True, "colorbar_enabled": combined_colorbar_enabled})


@app.route("/api/send-email", methods=["POST"])
def api_send_email():
    body = request.get_json(silent=True) or {}
    name = (body.get("name") or "").strip()
    to_email = (body.get("email") or "").strip()

    if not to_email or "@" not in to_email:
        return jsonify({"error": "a valid email address is required"}), 400
    if not _combined_ready():
        return jsonify({"error": "capture an RPi photo and a Lepton thermal image first"}), 400

    png_bytes = _render_combined_png()
    subject = os.environ.get("EMAIL_SUBJECT", "Your EPSCOR C3M thermal photo!")
    body_text = render_template(
        "email_body.txt",
        visitor_name=name,
        website_url=os.environ.get("PROJECT_WEBSITE_URL", ""),
    )

    try:
        outreach.send_email(to_email, subject, body_text, png_bytes, attachment_filename="epscor_c3m_photo.png")
    except (outreach.OutreachConfigError, outreach.OutreachSendError) as e:
        return jsonify({"error": str(e)}), 400

    bus.publish("log", {"text": f"[emailed combined photo to {to_email}]"})
    return jsonify({"ok": True})


@app.route("/api/combined/save", methods=["POST"])
def api_combined_save():
    if not _combined_ready():
        return jsonify({"error": "capture an RPi photo and a Lepton thermal image first"}), 400

    png_bytes = _render_combined_png()
    directory = os.path.join(gs_cli.CAPTURES_DIR, "combined")
    os.makedirs(directory, exist_ok=True)
    filename = os.path.join(directory, f"combined_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
    with open(filename, "wb") as f:
        f.write(png_bytes)

    bus.publish("log", {"text": f"[saved combined photo to {filename}]"})
    return jsonify({"ok": True, "filename": filename})


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
    app.run(host="127.0.0.1", port=5051, debug=False, threaded=True)
