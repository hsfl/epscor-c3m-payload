#!/usr/bin/env python3
"""
RPi Multicam Controller with UART Data Transfer
EPSCOR C3M Payload - Satellite Raspberry Pi Component

This Python script runs on a Raspberry Pi in the satellite payload to capture
thermal images and transmit the data to a Teensy microcontroller via UART for
radio transmission to the ground station.

Unlike lepton_controller.py / boson_controller.py (which are each hardwired to
one camera), this controller probes every camera in CAMERA_CLASSES at startup
and drives whichever ones respond through a common interface
(initialize/start_streaming/capture/get_stream_frame/cleanup) defined by
lepton_camera.LeptonCamera, boson_camera.BosonCamera, and rpi_camera.RpiCamera.
A TRIGGER captures from every connected camera in sequence; livestream is
serviced by the first connected camera that supports it.

Communication Protocol:
- UART: 115200 baud, 8N1
- Trigger: "TRIGGER\n" command from Teensy
- Header: Magic bytes (0xDE 0xAD 0xBE 0xEF) + camera-ID (1 byte, see CAMERA_IDS) + 24-bit length
- Data: Raw thermal image data
- End: Magic bytes (0xFF 0xFF)

@author EPSCOR C3M Team
@date 2026-08-04
@version 1.0.0
"""

__version__ = "1.0.0"
__build_date__ = "2026-08-04"

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import serial

from lepton_camera import LeptonCamera
from boson_camera import BosonCamera
from rpi_camera import RpiCamera

# Verbose logging: on TRIGGER, also dump each captured frame's raw values as a
# CSV and a rendered PNG/JPG onto the Pi's disk under DEBUG_SAVE_DIR.
# Set from the --debug CLI flag in main() - don't edit directly.
DEBUG = False
DEBUG_SAVE_DIR = Path("captures")

# UART Configuration for Teensy communication
UART_PORT = '/dev/serial0'  # Primary UART (GPIO14/15, pins 8/10)
UART_BAUD = 115200         # High-speed UART to match Teensy baud rate
TRIGGER_COMMAND = b'TRIGGER\n'  # UART command from Teensy to initiate capture
STREAM_START_COMMAND = b'STREAM_START\n'  # Command to start livestream mode
STREAM_STOP_COMMAND = b'STREAM_STOP\n'    # Command to stop livestream mode
FRAME_REQUEST_COMMAND = b'FRAME\n'        # Command to request a single stream frame

STREAM_MAGIC = bytes([0xCA, 0xFE, 0xBA, 0xBE])  # Magic header for livestream frames

# Leave stream mode if the Teensy stops asking for frames (it may have reset).
STREAM_IDLE_TIMEOUT_S = 30.0

# Every camera this Pi might have wired up. detect_connected_cameras() probes
# each of these at startup; only the ones whose hardware responds are used.
CAMERA_CLASSES = {
    'lepton': LeptonCamera,
    'boson': BosonCamera,
    'rpicam': RpiCamera,
}

# Wire-protocol camera-ID byte sent to the satellite Teensy (and relayed through
# to the ground station) so a capture can be attributed to its source camera.
# Must match the CameraId enum in satellite_teensy.ino / ground_station_teensy.ino
# exactly - these are not independently choosable per side.
CAMERA_IDS = {
    'lepton': 0,
    'boson': 1,
    'rpicam': 2,
}
CAMERA_ID_NONE = 0xFF  # sentinel for non-image UART payloads (e.g. STATUS messages)


def detect_connected_cameras():
    """
    Try to bring up every registered camera; return {name: instance} for the
    ones whose hardware actually responds (initialize()/start_streaming()
    succeed). Detection only runs once at startup - the payload's cameras are
    fixed at boot, not hot-pluggable.
    """
    connected = {}
    for name, camera_cls in CAMERA_CLASSES.items():
        camera = camera_cls()
        try:
            camera.initialize()
            camera.start_streaming()
        except Exception as e:
            print(f"{name}: not connected ({e})")
            try:
                camera.cleanup()
            except Exception:
                pass
            continue

        print(f"{name}: connected")
        connected[name] = camera

    return connected


def save_debug_frame(camera_name, camera):
    """
    Dump the camera's last captured frame to disk: raw pixel values as a CSV,
    plus a rendered PNG/JPG (whichever encodes smaller) for quick visual
    inspection. Requires camera.last_frame to have been set by capture().
    """
    frame = getattr(camera, "last_frame", None)
    if frame is None:
        print(f"{camera_name}: no frame array available to save for debug")
        return

    DEBUG_SAVE_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    stem = DEBUG_SAVE_DIR / f"{camera_name}_{ts}"

    # Raw values as CSV. A 2D (grayscale) frame is written as-is; a 3D (e.g.
    # BGR) frame is flattened so each row is still one image row.
    csv_path = stem.with_suffix(".csv")
    csv_frame = frame.reshape(frame.shape[0], -1) if frame.ndim > 2 else frame
    np.savetxt(csv_path, csv_frame, delimiter=",", fmt="%d")

    # Rendered image. Thermal frames are raw uint16 counts, not 0-255, so they
    # need normalizing before either encoder can touch them.
    render = frame if frame.dtype == np.uint8 else cv2.normalize(
        frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
    )

    ok_png, png_bytes = cv2.imencode(".png", render)
    ok_jpg, jpg_bytes = cv2.imencode(".jpg", render, [cv2.IMWRITE_JPEG_QUALITY, 90])

    image_path = None
    if ok_png and (not ok_jpg or len(png_bytes) <= len(jpg_bytes)):
        image_path = stem.with_suffix(".png")
        image_path.write_bytes(png_bytes.tobytes())
    elif ok_jpg:
        image_path = stem.with_suffix(".jpg")
        image_path.write_bytes(jpg_bytes.tobytes())

    print(f"{camera_name}: saved debug frame -> {csv_path.name}"
          + (f", {image_path.name}" if image_path else " (image render failed)"))


def pick_streaming_camera(connected):
    """
    Pick which connected camera services STREAM_START/FRAME requests.

    Only cameras with a defined STREAM_FRAME_SIZE support the livestream
    protocol today (e.g. RpiCamera doesn't yet) - the first such camera, in
    CAMERA_CLASSES order, wins.
    """
    for name, camera in connected.items():
        if camera.STREAM_FRAME_SIZE is not None:
            return name, camera
    return None, None


def run_trigger(connected, uart_port):
    """
    Capture once from every connected camera.

    If uart_port is given, each capture is sent over UART and status is
    reported back to the Teensy, same as before. If uart_port is None
    (--no-teensy), the same capture/debug-save pipeline runs locally and
    frames are dropped instead of transmitted - lets the camera side be
    bench-tested with no serial hardware attached at all.
    """
    for name, camera in connected.items():
        send_status_uart(f"CAPTURE_START:{name}", uart_port)

        frame_data = camera.capture()

        if frame_data is None:
            print(f"{name}: no frame available.")
            send_status_uart(f"NO_FRAMES:{name}", uart_port)
            continue

        if DEBUG:
            save_debug_frame(name, camera)

        if uart_port is None:
            print(f"{name}: captured {len(frame_data)} bytes (not sent - --no-teensy)")
            continue

        print(f"\n{name}: waiting 3 seconds for Teensy to prepare...")
        time.sleep(3)
        success = send_data_uart(frame_data, uart_port, camera_id=CAMERA_IDS[name])
        if success:
            print(f"{name}: data transmission successful!")
            send_status_uart(f"CAPTURE_DONE:{name}", uart_port)
        else:
            print(f"{name}: data transmission failed!")
            send_status_uart(f"TX_FAIL:{name}", uart_port)


def send_status_uart(message: str, uart_port):
    """
    Send a human-readable STATUS payload using the same header/length/end scheme.

    uart_port may be None (--no-teensy) - in that case the status is just
    printed locally instead of framed and transmitted.
    """
    if uart_port is None:
        print(f"[STATUS] {message}")
        return True

    try:
        payload = f"STATUS:{message}".encode("ascii", "ignore")
        return send_data_uart(payload, uart_port, camera_id=CAMERA_ID_NONE)
    except Exception as e:
        print(f"UART status send error: {e}")
        return False


def send_data_uart(data, uart_port, camera_id=CAMERA_ID_NONE):
    """
    Send thermal image data via UART with header and end markers

    Uses a 24-bit length field (matching satellite_teensy.ino's current parser)
    so this works for both the Lepton's 38,400-byte frames and the Boson's
    larger ones without overflowing a 16-bit length. The camera-ID byte lets
    the satellite Teensy attribute this payload to a source camera when it
    later relays it over radio (see CAMERA_IDS) - STATUS payloads use the
    CAMERA_ID_NONE sentinel since they aren't tied to one camera.

    Args:
        data: Thermal image data as bytes
        uart_port: Serial port object for UART communication
        camera_id: one of CAMERA_IDS' values, or CAMERA_ID_NONE

    Returns:
        bool: True if transmission successful, False otherwise
    """
    if not data:
        print("UART: nothing to send (len=0).")
        return False

    try:
        # Create header with magic bytes, camera-ID, and length
        header = bytearray()
        header.extend([0xDE, 0xAD, 0xBE, 0xEF])  # Magic bytes for validation
        header.append(camera_id & 0xFF)

        # Add data length (little-endian, 24-bit)
        length = len(data)
        header.extend([length & 0xFF, (length >> 8) & 0xFF, (length >> 16) & 0xFF])

        uart_port.write(header)
        uart_port.flush()  # Ensure header is transmitted immediately

        # Small delay for header processing on receiver
        time.sleep(0.1)

        start_time = time.time()

        # Send data in chunks for better throughput and progress tracking
        chunk_size = 1024  # 1KB chunks for optimal transmission
        total_sent = 0

        while total_sent < length:
            chunk_end = min(total_sent + chunk_size, length)
            chunk = data[total_sent:chunk_end]

            uart_port.write(chunk)
            total_sent += len(chunk)

        # Send end markers to signal completion
        end_markers = bytearray([0xFF, 0xFF])
        uart_port.write(end_markers)
        uart_port.flush()

        # Display transmission summary
        elapsed = time.time() - start_time
        print(f"\nUART transmission complete!")
        print(f"Sent {length} bytes in {elapsed:.2f} seconds")
        print(f"Average rate: {length/elapsed:.0f} bytes/second")

        return True

    except Exception as e:
        print(f"UART transmission error: {e}")
        return False


def send_stream_frame_uart(frame_8bit, frame_seq, frame_size, uart_port):
    """
    Send a downsampled stream frame via UART with livestream header.

    Uses STREAM_MAGIC header (0xCA 0xFE 0xBA 0xBE) to distinguish from
    regular thermal captures. The Teensy forwards these frames to the
    ground station for live viewing (not saving).

    Args:
        frame_8bit: numpy array of downsampled frame data
        frame_seq: frame sequence number (0-255, wrapping)
        frame_size: expected byte length of frame_8bit.tobytes() (camera-specific)
        uart_port: Serial port object for UART communication

    Returns:
        bool: True if transmission successful, False otherwise
    """
    if frame_8bit is None:
        return False

    try:
        data = frame_8bit.tobytes()
        if len(data) != frame_size:
            print(f"Warning: stream frame size {len(data)}, expected {frame_size}")
            return False

        # [STREAM_MAGIC 4B][Frame Seq 1B][Frame Size 2B][Frame Data][End 2B]
        header = bytearray()
        header.extend(STREAM_MAGIC)
        header.append(frame_seq & 0xFF)
        header.extend([frame_size & 0xFF, (frame_size >> 8) & 0xFF])

        uart_port.write(header)
        uart_port.write(data)
        uart_port.write(bytearray([0xFF, 0xFF]))
        uart_port.flush()

        return True

    except Exception as e:
        print(f"Stream frame UART error: {e}")
        return False


def wait_for_uart_command(uart_port, timeout=None):
    """
    Wait for UART command from Teensy (TRIGGER or STREAM_START/STOP)

    Args:
        uart_port: Serial port object for UART communication
        timeout: Optional timeout in seconds (None for blocking)

    Returns:
        str: 'trigger', 'stream_start', 'stream_stop', or None on timeout
    """
    start_time = time.time()
    buffer = b''

    while True:
        if timeout is not None:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                return None

        if uart_port.in_waiting > 0:
            chunk = uart_port.read(uart_port.in_waiting)
            buffer += chunk

            if TRIGGER_COMMAND in buffer:
                return 'trigger'
            if STREAM_START_COMMAND in buffer:
                return 'stream_start'
            if STREAM_STOP_COMMAND in buffer:
                return 'stream_stop'

            # Keep buffer size manageable (only last 100 bytes)
            if len(buffer) > 100:
                buffer = buffer[-100:]
        else:
            time.sleep(0.01)  # Small delay to avoid busy-waiting


def run_stream_loop(camera, uart_port):
    """
    Run request-response streaming loop: wait for FRAME request -> send latest frame.

    Uses explicit flow control - the Teensy requests each frame when ready.
    This prevents serial buffer overflow since the Pi only sends when the
    Teensy asks.

    Protocol:
    1. Teensy sends STREAM_START to enter stream mode
    2. Teensy sends FRAME to request each frame
    3. Pi grabs the freshest frame from `camera`, downsamples, and sends it
    4. Teensy receives frame, transmits over radio, then requests next
    5. Teensy sends STREAM_STOP to exit

    Args:
        camera: active camera instance (LeptonCamera or BosonCamera)
        uart_port: Serial port object for UART communication

    Returns:
        str: Reason for exit ('stop_command', 'timeout', 'interrupted', 'error')
    """
    print("Stream mode: waiting for frame requests from Teensy...")

    frame_seq = 0
    frames_sent = 0
    start_time = time.time()
    last_request = time.time()
    rx_buffer = b''  # Buffer for accumulating incoming UART data

    try:
        while True:
            if uart_port.in_waiting > 0:
                rx_buffer += uart_port.read(uart_port.in_waiting)

            if STREAM_STOP_COMMAND in rx_buffer or b'STREAM_STOP' in rx_buffer:
                print("Stream stop command received")
                return 'stop_command'

            if b'FRAME' not in rx_buffer:
                # Give up if the Teensy has gone quiet - it may have reset, and
                # without this the Pi would sit in stream mode indefinitely.
                if time.time() - last_request > STREAM_IDLE_TIMEOUT_S:
                    print("Stream: no frame requests received, leaving stream mode")
                    return 'timeout'

                if len(rx_buffer) > 256:
                    rx_buffer = rx_buffer[-128:]
                time.sleep(0.01)
                continue

            # Consume the request, tolerating a missing trailing newline
            idx = rx_buffer.find(b'FRAME')
            rx_buffer = rx_buffer[idx + len(b'FRAME'):]
            if rx_buffer.startswith(b'\n'):
                rx_buffer = rx_buffer[1:]
            last_request = time.time()

            frame_8bit = camera.get_stream_frame(max_age_s=2.0)

            # If no recent frame, wait briefly and retry once.
            if frame_8bit is None:
                time.sleep(0.1)
                frame_8bit = camera.get_stream_frame(max_age_s=2.0)

            if frame_8bit is None:
                print("No frame available from camera")
                continue

            if send_stream_frame_uart(frame_8bit, frame_seq, camera.STREAM_FRAME_SIZE, uart_port):
                frames_sent += 1
                frame_seq = (frame_seq + 1) & 0xFF

                if frames_sent % 10 == 0:
                    elapsed = time.time() - start_time
                    fps = frames_sent / elapsed if elapsed > 0 else 0
                    print(f"Streaming: {frames_sent} frames, {fps:.1f} fps")
            else:
                print("Failed to send stream frame")

            if len(rx_buffer) > 256:
                rx_buffer = rx_buffer[-128:]

    except KeyboardInterrupt:
        print("Stream interrupted by user")
        return 'interrupted'

    except Exception as e:
        print(f"Stream error: {e}")
        import traceback
        traceback.print_exc()
        return 'error'


def parse_args():
    parser = argparse.ArgumentParser(description="RPi Multicam Controller")
    parser.add_argument("--debug", action="store_true",
                         help="Save each captured frame's raw values (CSV) and a rendered "
                              f"PNG/JPG under {DEBUG_SAVE_DIR}/")
    parser.add_argument("--no-teensy", action="store_true",
                         help="Run the capture pipeline locally without opening UART or "
                              "sending to the Teensy: detects cameras, captures once from "
                              "each, then exits")
    return parser.parse_args()


def main():
    global DEBUG

    args = parse_args()
    DEBUG = args.debug

    print("RPi Multicam Controller - UART Output")
    print(f"Debug frame dumps: {'ON -> ' + str(DEBUG_SAVE_DIR) if DEBUG else 'OFF'}")
    print("="*40)

    if args.no_teensy:
        print("--no-teensy: running capture pipeline locally, no UART will be opened")
        connected = {}
        try:
            connected = detect_connected_cameras()
            if not connected:
                print("No cameras responded.")
                return

            time.sleep(2)  # allow cameras to stabilize
            print(f"Connected cameras: {', '.join(connected.keys())}")

            run_trigger(connected, None)
        finally:
            for camera in connected.values():
                try:
                    camera.cleanup()
                except Exception:
                    pass
        return

    # UART first (so we can notify Teensy if no cameras are present)
    try:
        uart = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"UART initialized: {UART_PORT} at {UART_BAUD} baud")
    except Exception as e:
        print(f"UART initialization failed: {e}")
        print("Make sure UART is enabled in raspi-config!")
        return

    send_status_uart("BOOT", uart)

    connected = {}

    try:
        connected = detect_connected_cameras()
        if not connected:
            print("No cameras responded; notifying Teensy.")
            send_status_uart("NO_CAMERA", uart)
            # Clean exit so systemd can retry later
            return

        time.sleep(2)  # allow cameras to stabilize
        send_status_uart(f"CAM_READY:{','.join(connected.keys())}", uart)

        print(f"\nSystem ready. Connected cameras: {', '.join(connected.keys())}")
        print("Waiting for UART commands:")
        print(f"  - {TRIGGER_COMMAND.decode('ascii').strip()}: Capture from every connected camera")
        print(f"  - {STREAM_START_COMMAND.decode('ascii').strip()}: Start livestream")
        print(f"  - {STREAM_STOP_COMMAND.decode('ascii').strip()}: Stop livestream")
        print(f"Data will be sent via UART: {UART_PORT} at {UART_BAUD} baud")
        send_status_uart("IDLE", uart)

        while True:
            command = wait_for_uart_command(uart)

            if command == 'trigger':
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] Capture triggered via UART!")
                print("="*40)

                run_trigger(connected, uart)

                print("\nReady for next command...")
                print("="*40)
                send_status_uart("IDLE", uart)

            elif command == 'stream_start':
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] Stream mode started via UART!")
                print("="*40)

                stream_name, stream_camera = pick_streaming_camera(connected)
                if stream_camera is None:
                    print("No connected camera supports streaming.")
                    send_status_uart("NO_STREAM_CAMERA", uart)
                else:
                    print(f"Streaming from: {stream_name}")
                    exit_reason = run_stream_loop(stream_camera, uart)
                    print(f"Stream ended: {exit_reason}")

                print("\nReady for next command...")
                print("="*40)
                send_status_uart("IDLE", uart)

            elif command == 'stream_stop':
                # Stop command received outside of stream mode - just acknowledge
                print("Stream stop received (not in stream mode)")
                send_status_uart("IDLE", uart)

    except KeyboardInterrupt:
        print("\nShutting down...")
        send_status_uart("SHUTDOWN", uart)

    except Exception as exc:
        print(f"Unexpected error: {exc}")
        send_status_uart("ERROR", uart)

    finally:
        for camera in connected.values():
            try:
                camera.cleanup()
            except Exception:
                pass
        try:
            uart.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
