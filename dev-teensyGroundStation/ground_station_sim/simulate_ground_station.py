#!/usr/bin/env python3
"""
Ground Station Teensy Simulator
EPSCOR C3M Payload - Ground Station Component

Stands in for the real ground_station_teensy.ino firmware's USB-serial
interface so ground_station_gui/gds_app.py (or
ground_station_serial_cli_teensy.py) can be exercised without hardware.

It does not model the RF22 radio link to the satellite - the GUI never
sees that layer directly, only the lines the firmware prints to Serial. This
reproduces those lines for the 'capture', 'request <0|1|2>', and 'export'
commands, using local sample image data in place of a live satellite.

Usage:
    socat -d -d pty,raw,echo=0,link=/tmp/ttyGS pty,raw,echo=0,link=/tmp/ttySIM
    python simulate_ground_station.py --port /tmp/ttySIM

Then connect the GUI (gds_app.py) to /tmp/ttyGS via its manual port field.

Sample data (see sim_data/ next to this script):
    sim_data/lepton.bin   raw 160x120 uint16 LE centi-Kelvin (38400 bytes)
    sim_data/boson.bin    raw 320x256 uint16 LE raw counts   (163840 bytes)
    sim_data/rpicam.jpg   any valid JPEG

.csv alternatives are accepted for lepton/boson: comma-separated integers,
one row per image row. '#'-prefixed comment lines and blank lines are
skipped, so the GUI's own capture exports (metadata header block included)
work directly. The CSV's own row/column shape is used as-is -- it does not
need to match the nominal resolution above. For lepton specifically, values
are sniffed for units: if they look like already-converted Celsius (small
magnitude) rather than raw centi-Kelvin, they're inverted back to raw, so a
CSV the GUI itself exported can be reused without double-converting.

Filenames are also matched against the GUI's own capture-export naming --
e.g. lepton_capture_003.csv, rpicam_capture_002.jpg -- picking the
highest-numbered match if the plain '<name>.<ext>' file isn't present.

Any file that's missing falls back to a synthetic placeholder so the
simulator still runs before all sample data is ready.
"""

import argparse
import base64
import io
import sys
import time
from pathlib import Path

import numpy as np
import serial

BAUD_RATE = 115200

THERMAL_CAPTURE_TIMESTAMP_FLAG = "--- UART THERMAL CAPTURE ---"
THERMAL_CSV_START = "=== START CSV ==="
THERMAL_CSV_END = "=== END CSV ==="
THERMAL_JPG_START = "=== START JPG ==="
THERMAL_JPG_END = "=== END JPG ==="

CAMERAS = {
    0: {
        "name": "lepton",
        "width": 160,
        "height": 120,
        "radiometric": True,
        "tag": "--- EXPORTING LEPTON THERMAL DATA ---",
    },
    1: {
        "name": "boson",
        "width": 320,
        "height": 256,
        "radiometric": False,
        "tag": "--- EXPORTING BOSON THERMAL DATA ---",
    },
}

PACKET_DATA_SIZE = 45  # matches RADIO_PACKET_MAX_SIZE - overhead on the real link


class SimDataError(Exception):
    pass


def synth_grid(width, height):
    """Smooth gradient centered near room-temp centi-Kelvin, for lepton/boson."""
    x = np.linspace(0, 1, width)
    y = np.linspace(0, 1, height)
    xx, yy = np.meshgrid(x, y)
    return (29000 + 900 * (0.5 * xx + 0.5 * yy)).astype(np.uint16)


def synth_jpeg():
    from PIL import Image

    xv = (np.arange(320) % 256).astype(np.uint8)
    yv = (np.arange(240) % 256).astype(np.uint8)
    xx, yy = np.meshgrid(xv, yv)
    blue = (xx.astype(np.int16) + yy.astype(np.int16)).astype(np.uint8)
    arr = np.stack([xx, yy, blue], axis=-1)
    buf = io.BytesIO()
    Image.fromarray(arr, mode="RGB").save(buf, format="JPEG")
    return buf.getvalue()


def find_source_file(data_dir, name, exts):
    """Look for '<name><ext>' first, then fall back to the GUI's own export
    naming ('<name>_capture_NNN<ext>', picking the highest-numbered one) so
    files already sitting in a captures/ folder can be used directly."""
    for ext in exts:
        path = data_dir / f"{name}{ext}"
        if path.exists():
            return path
    candidates = [p for ext in exts for p in data_dir.glob(f"{name}_capture_*{ext}")]
    return sorted(candidates)[-1] if candidates else None


def load_grid(data_dir, cam):
    name, width, height = cam["name"], cam["width"], cam["height"]
    bin_path = find_source_file(data_dir, name, (".bin",))
    csv_path = None if bin_path else find_source_file(data_dir, name, (".csv",))

    if bin_path:
        # .bin is a flat byte dump with no shape of its own, so it must match
        # the documented resolution exactly to be reshaped correctly.
        raw = bin_path.read_bytes()
        expected = width * height * 2
        if len(raw) != expected:
            raise SimDataError(
                f"{bin_path} is {len(raw)} bytes, expected {expected} "
                f"({width}x{height} uint16)"
            )
        grid = np.frombuffer(raw, dtype="<u2").reshape(height, width)
        origin = str(bin_path)
        inferred = "raw (binary dump)"
    elif csv_path:
        # CSV is self-describing (each line is one image row), so the grid's
        # shape is taken from the file itself rather than enforced against
        # width/height -- real capture exports don't always match the
        # nominal camera resolution (e.g. a transposed row/col orientation).
        float_rows = []
        row_len = None
        for line in csv_path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cells = [c.strip() for c in line.split(",")]
            if row_len is None:
                row_len = len(cells)
            elif len(cells) != row_len:
                raise SimDataError(
                    f"{csv_path}: ragged row {len(float_rows) + 1} has "
                    f"{len(cells)} values, expected {row_len} (from earlier rows)"
                )
            float_rows.append([float(c) for c in cells])
        if not float_rows:
            raise SimDataError(
                f"{csv_path}: no data rows found (after stripping '#' comment/blank lines)"
            )

        if cam["radiometric"]:
            # Sniff whether this CSV holds raw centi-Kelvin ints (thousands)
            # or already-converted Celsius (tens/low hundreds) -- e.g. a
            # capture the GUI itself exported, which lepton_rows_to_celsius()
            # already converted. Re-emitting Celsius as if it were raw would
            # get double-converted downstream, so invert it back to raw.
            flat = sorted(v for row in float_rows for v in row)
            median = flat[len(flat) // 2]
            if median < 1000:
                inferred = "already-converted Celsius -- inverted back to raw centi-Kelvin"
                float_rows = [[round((v + 273.15) * 100) for v in row] for row in float_rows]
            else:
                inferred = "raw centi-Kelvin"
        else:
            inferred = "raw counts"

        grid = np.array(float_rows, dtype=np.uint16)
        origin = str(csv_path)
    else:
        grid = synth_grid(width, height)
        origin = "synthetic placeholder"
        inferred = "synthetic"
        print(
            f"[sim] {name}: no {name}.bin/{name}.csv (or {name}_capture_*.csv) "
            f"in {data_dir} -- using synthetic placeholder"
        )

    lo, hi = int(grid.min()), int(grid.max())
    hint = ""
    if cam["radiometric"]:
        hint = f" (~{lo / 100 - 273.15:.1f}-{hi / 100 - 273.15:.1f} degC)"
    print(
        f"[sim] {name}: {grid.shape[1]}x{grid.shape[0]} from {origin} "
        f"[{inferred}], raw range [{lo}, {hi}]{hint}"
    )
    return grid


def load_jpeg(data_dir):
    path = find_source_file(data_dir, "rpicam", (".jpg", ".jpeg"))
    if path:
        data = path.read_bytes()
        if data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
            raise SimDataError(
                f"{path} does not look like a valid JPEG (missing SOI/EOI markers)"
            )
        print(f"[sim] rpicam: {len(data)} bytes from {path}")
        return data

    data = synth_jpeg()
    print(
        f"[sim] rpicam: no rpicam.jpg/.jpeg (or rpicam_capture_*.jpg) in "
        f"{data_dir} -- using synthetic placeholder ({len(data)} bytes)"
    )
    return data


class GroundStationSim:
    def __init__(self, ser, data_dir):
        self.ser = ser
        self.grids = {cam_id: load_grid(data_dir, cam) for cam_id, cam in CAMERAS.items()}
        self.jpeg = load_jpeg(data_dir)
        self.last_camera_id = None

    def send(self, text=""):
        self.ser.write((text + "\n").encode())
        self.ser.flush()

    def run(self):
        print("[sim] ready, waiting for commands...")
        while True:
            raw = self.ser.readline()
            if not raw:
                continue
            self.dispatch(raw.decode("utf-8", "ignore").strip())

    def dispatch(self, line):
        if not line:
            return
        print(f"[sim] << {line}")
        parts = line.split(None, 1)
        cmd = parts[0].lower()
        args = parts[1] if len(parts) > 1 else ""

        if cmd == "capture":
            self.cmd_capture()
        elif cmd == "request":
            self.cmd_request(args)
        elif cmd == "export":
            self.cmd_export()
        else:
            self.send(f"[sim] '{cmd}' not simulated")

    def cmd_capture(self):
        self.send(THERMAL_CAPTURE_TIMESTAMP_FLAG)
        self.send("Forwarding command to satellite...")
        self.send("Command sent successfully")
        time.sleep(1.0)
        self.send("SAT> --- UART CAPTURE ---")
        self.send("SAT> Triggering RPI capture...")
        self.send("SAT> Waiting for capture status from RPI...")
        time.sleep(1.0)
        self.send("SAT> CAPTURE_DONE")
        self.send("SAT> --- UART CAPTURE COMPLETE ---")

    def cmd_request(self, args):
        arg = args.strip()
        if len(arg) != 1 or arg not in "012":
            self.send("Usage: request <camera_id>  (0=lepton, 1=boson, 2=rpicam)")
            return

        camera_id = int(arg)
        self.last_camera_id = camera_id

        self.send()
        self.send("--- REQUESTING THERMAL DATA DOWNLINK ---")
        self.send(f"Forwarding command to satellite: request {camera_id}")
        self.send("Command sent successfully")
        time.sleep(0.4)

        if camera_id == 2:
            length = len(self.jpeg)
        else:
            length = self.grids[camera_id].size * 2  # actual loaded grid, not the nominal resolution
        packets = -(-length // PACKET_DATA_SIZE)  # ceil

        self.send()
        self.send("\U0001F4E6 THERMAL IMAGE HEADER RECEIVED!")
        self.send(f"Expected size: {length} bytes")
        self.send(f"Expected packets: {packets}")
        self.send("✓ Header valid - receiving thermal data...")

        for pct in (25, 50, 75, 100):
            time.sleep(0.3)
            self.send(f"{pct}%")

        self.send()
        self.send("=== RECEPTION SUMMARY ===")
        self.send(f"Received {packets} of {packets} packets (100.0%)")
        self.send()
        self.send("✅ PERFECT RECEPTION!")

        # Real firmware auto-exports after a 'request' (autoExportOnComplete).
        self.export_camera(camera_id)

    def cmd_export(self):
        if self.last_camera_id is None:
            self.send("No complete thermal image to export")
            return
        self.export_camera(self.last_camera_id)

    def export_camera(self, camera_id):
        if camera_id == 2:
            self.send()
            self.send("--- EXPORTING RPICAM IMAGE ---")
            self.send(THERMAL_JPG_START)
            encoded = base64.encodebytes(self.jpeg).decode("ascii")
            for line in encoded.splitlines():
                self.send(line)
            self.send(THERMAL_JPG_END)
            return

        cam = CAMERAS[camera_id]
        grid = self.grids[camera_id]
        self.send()
        self.send(cam["tag"])
        self.send("Copying data below to a 'thermal_image.csv'")
        self.send(THERMAL_CSV_START)
        for row in grid:
            self.send(",".join(str(int(v)) for v in row))
        self.send(THERMAL_CSV_END)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port / PTY path to listen on (the simulator's end of the socat pair)",
    )
    parser.add_argument(
        "--data-dir",
        default=None,
        help="Directory holding lepton.bin/boson.bin/rpicam.jpg (default: sim_data/ next to this script)",
    )
    args = parser.parse_args()

    data_dir = Path(args.data_dir) if args.data_dir else Path(__file__).resolve().parent / "sim_data"
    if not data_dir.is_dir():
        print(f"[sim] data directory {data_dir} does not exist", file=sys.stderr)
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.5)
    except serial.SerialException as e:
        print(f"[sim] failed to open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"[sim] listening on {args.port} at {BAUD_RATE} baud")

    try:
        sim = GroundStationSim(ser, data_dir)
    except SimDataError as e:
        print(f"[sim] {e}", file=sys.stderr)
        sys.exit(1)

    try:
        sim.run()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
