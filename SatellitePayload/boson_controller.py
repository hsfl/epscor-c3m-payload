#!/usr/bin/env python3
"""
RPi Boson Controller with UART Data Transfer
EPSCOR C3M Payload

This Python script runs on a Raspberry Pi in the satellite payload to capture
thermal images using the USB thermal camera and transmit the data to a Teensy
microcontroller via UART for radio transmission to the ground station.

Key Features:
- Captures thermal images using USB thermal camera
- Transmits data via UART with header/end markers
- Provides real-time progress updates and data validation
- Listens for UART trigger commands from Teensy microcontroller

Hardware Requirements:
- Raspberry Pi (3B+ or 4 recommended)
- USB thermal camera (built for Boson camera specifically)
- UART connection to Teensy microcontroller

Communication Protocol:
- UART: 115200 baud, 8N1
- Trigger: "TRIGGER\n" command from Teensy
- Header: Magic bytes (0xDE 0xAD 0xBE 0xEF) + 24-bit length
- Data: Raw thermal image data (24-bit per pixel)
- End: Magic bytes (0xFF 0xFF)

@author EPSCOR C3M Team
@date 2026-07-06
@version 1.0.0
"""

__version__ = "1.0.0"
__build_date__ = "2026-07-06"

import numpy as np
import time
import serial
import cv2
import pyudev
import re
import os
import sys
from pathlib import Path

# UART Configuration for Teensy communication
UART_PORT = '/dev/serial0'  # Primary UART (GPIO14/15, pins 8/10)
UART_BAUD = 115200         # High-speed UART to match Teensy baud rate
TRIGGER_COMMAND = b'TRIGGER\n'  # UART command from Teensy to initiate capture
STREAM_START_COMMAND = b'STREAM_START\n'  # Command to start livestream mode
STREAM_STOP_COMMAND = b'STREAM_STOP\n'    # Command to stop livestream mode
FRAME_REQUEST_COMMAND = b'FRAME\n'        # Command to request a single stream frame

BOSON_IMAGE_HEIGHT = 256
BOSON_TELEMETRY_ROWS = 2

# Livestream frame configuration.
# The Teensy and ground station both hardcode a 100x80 8-bit frame (8000 bytes),
# so these values must not change without updating satellite_teensy.ino and
# ground_station_serial_cli_teensy.py together.
STREAM_MAGIC = bytes([0xCA, 0xFE, 0xBA, 0xBE])  # Distinguishes stream frames from captures
STREAM_FRAME_WIDTH = 100
STREAM_FRAME_HEIGHT = 80
STREAM_FRAME_SIZE = STREAM_FRAME_WIDTH * STREAM_FRAME_HEIGHT  # 8000 bytes per frame

# Leave stream mode if the Teensy stops asking for frames (it may have reset).
STREAM_IDLE_TIMEOUT_S = 30.0

# Scan /dev/video* devices and return the index whose USB vendor ID matches the FLIR Boson Camera 
def find_boson_index(vendor_id="09cb"):
    context = pyudev.Context()
    candidates = []

    for device in context.list_devices(subsystem='video4linux'):
        node = device.device_node
        if not node:
            continue

        usb_device = device.find_parent('usb', 'usb_device')
        if usb_device is None:
            continue

        vid = usb_device.get('ID_VENDOR_ID')

        if vid == vendor_id:
            index = int(re.search(r'\d+$', node).group())
            candidates.append((index, node, vid))

    if not candidates:
        raise RuntimeError(f"Could not find Boson camera in usb devices. Confirm that the vendor id is {vendor_id}")

    candidates.sort()
    return candidates[0][0]

# Takes in 16-bit thermal data and normalizes it to 8-bit for image generation
def normalize_thermal(frame16, low_pct=1, high_pct=99):
    lo, hi = np.percentile(frame16, (low_pct, high_pct))
    if hi <= lo:
        hi = lo + 1  # avoid div-by-zero on a flat frame
    clipped = np.clip(frame16, lo, hi)
    norm = ((clipped - lo) / (hi - lo) * 255).astype(np.uint8)
    return norm

# Takes one thermal capture and returns the image data in bytes
# If debug is enabled, this function will also generate a png image of the thermal capture 
def record_boson_frame(camera_index, uart_port, debug=False):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

    try:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'Y16 '))

        if not cap.isOpened():
            print(f'Error: Could not open camera at index {camera_index}.')
            return None
    
        ret, frame = cap.read()
        if not ret:
            print('Error: Failed to read frame.')
            return None
        
        # Store raw data (convert to 16-bit if needed)
        if frame.dtype != np.uint16:
            frame16 = np.left_shift(frame.astype(np.uint16), 8)
        else:
            frame16 = frame

        send_status_uart(f"Frame16 type {frame16.dtype}", uart_port)
        send_status_uart(f"Frame16 min {frame16.min()} max {frame16.max()} mean {frame16.mean():.1f}", uart_port)

        send_status_uart(f"Initial frame shape: {frame16.shape}", uart_port)

        # Remove telemetry rows appended to the top of the frame
        if frame16.shape[0] > BOSON_IMAGE_HEIGHT:
            frame16 = frame16[-BOSON_IMAGE_HEIGHT:, :]
        elif frame16.shape[0] != BOSON_IMAGE_HEIGHT:
                send_status_uart(f"Unexpected frame height: {frame16.shape}, "f"Expected {BOSON_IMAGE_HEIGHT}", uart_port)

        send_status_uart(f"Final frame shape: {frame16.shape}", uart_port)

        if debug:
            dir = '/home/artemis4/debug'
            path = os.path.join(dir, "boson_image.png")
            os.makedirs(dir, exist_ok=True)

            norm = normalize_thermal(frame16)
            write = cv2.imwrite(path, norm)

            if write:
                print(f"Image saved to {path}")
            else:
                print("Failed to save thermal image")
        
        print(f"Frame shape: {frame16.shape}")

        bytes = frame16.tobytes()
        send_status_uart(f"frame16 bytes: {len(bytes)}", uart_port)

        return bytes
    finally:
        cap.release()

# Open the Boson for capture with the pixel format the payload expects.
# Caller is responsible for releasing the capture.
def open_boson_capture(camera_index):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'Y16 '))
    # Ask for the shallowest buffer available; V4L2 may ignore this, which is
    # why grab_fresh_frame also drains explicitly.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

# Drop the telemetry rows the Boson prepends to each frame
def strip_telemetry_rows(frame16):
    if frame16.shape[0] > BOSON_IMAGE_HEIGHT:
        return frame16[-BOSON_IMAGE_HEIGHT:, :]
    return frame16

# Return the newest frame available from an already-open capture, as 16-bit data
# with telemetry rows removed. V4L2 hands back the OLDEST buffered frame, and the
# radio relays only about one frame per second, so the driver's queue fills with
# stale frames between requests. Draining the queue first keeps the preview live.
# Note cap.grab() blocks when the queue is empty, so this costs a few frame
# intervals (~70ms) at worst - negligible against the radio's pace.
# Kept low deliberately: the Teensy only waits STREAM_HEADER_TIMEOUT_MS (1s) after
# sending FRAME, and each drained grab costs one frame interval - 111ms on a 9Hz
# export-restricted Boson. Raising this risks blowing that budget.
STREAM_MAX_DRAIN = 2

def grab_fresh_frame(cap, max_drain=STREAM_MAX_DRAIN):
    for _ in range(max_drain):
        if not cap.grab():
            break

    ret, frame = cap.read()
    if not ret:
        return None

    frame = np.squeeze(frame)
    if frame.dtype != np.uint16:
        frame = np.left_shift(frame.astype(np.uint16), 8)

    return strip_telemetry_rows(frame)

# Reduce a full-resolution 16-bit Boson frame to a 100x80 8-bit stream frame.
# The source is centre-cropped to the stream's aspect ratio before resizing so the
# preview is not distorted, then area-averaged down. Both steps are derived from
# the frame's own shape, so this holds for the 320x256 and 640x512 Boson variants.
def downsample_for_stream(frame16):
    if frame16 is None:
        return None

    if frame16.ndim != 2:
        print(f"Stream: unexpected frame shape {frame16.shape}")
        return None

    height, width = frame16.shape
    target_aspect = STREAM_FRAME_WIDTH / STREAM_FRAME_HEIGHT

    if width / height > target_aspect:
        crop_w = round(height * target_aspect)
        x0 = (width - crop_w) // 2
        frame16 = frame16[:, x0:x0 + crop_w]
    else:
        crop_h = round(width / target_aspect)
        y0 = (height - crop_h) // 2
        frame16 = frame16[y0:y0 + crop_h, :]

    small = cv2.resize(frame16,
                       (STREAM_FRAME_WIDTH, STREAM_FRAME_HEIGHT),
                       interpolation=cv2.INTER_AREA)

    # Per-frame percentile normalization. The Boson's Y16 output is raw counts
    # rather than radiometric Kelvin, so a fixed temperature window would clip
    # badly; the trade-off is that the preview's contrast floats with the scene.
    return normalize_thermal(small)

def send_stream_frame_uart(frame_8bit, frame_seq, uart_port):
    """
    Send one downsampled stream frame via UART.

    Uses STREAM_MAGIC rather than the capture header so the Teensy can tell
    livestream frames from a thermal capture. The Teensy relays these to the
    ground station for live viewing and does not store them.

    Args:
        frame_8bit: numpy array of shape (80, 100) with uint8 values
        frame_seq: frame sequence number (0-255, wrapping)
        uart_port: Serial port object for UART communication

    Returns:
        bool: True if transmission successful, False otherwise
    """
    if frame_8bit is None:
        return False

    try:
        data = frame_8bit.tobytes()
        if len(data) != STREAM_FRAME_SIZE:
            print(f"Warning: stream frame size {len(data)}, expected {STREAM_FRAME_SIZE}")
            return False

        # [STREAM_MAGIC 4B][Frame Seq 1B][Frame Size 2B][Frame Data 8000B][End 2B]
        header = bytearray()
        header.extend(STREAM_MAGIC)
        header.append(frame_seq & 0xFF)
        header.extend([STREAM_FRAME_SIZE & 0xFF, (STREAM_FRAME_SIZE >> 8) & 0xFF])

        uart_port.write(header)
        uart_port.write(data)
        uart_port.write(bytearray([0xFF, 0xFF]))
        uart_port.flush()

        return True

    except Exception as e:
        print(f"Stream frame UART error: {e}")
        return False

def run_stream_loop(uart_port, camera_index):
    """
    Run the request-response livestream loop until the Teensy stops it.

    The Teensy sets the pace: it sends FRAME for each frame it is ready to relay,
    which stops the Pi from overrunning the UART. The camera is opened once for
    the whole session because opening a V4L2 capture costs far more than a frame
    interval - it must not happen per request.

    Protocol:
    1. Teensy sends STREAM_START to enter stream mode
    2. Teensy sends FRAME to request each frame
    3. Pi grabs the freshest frame, downsamples it, and sends it
    4. Teensy relays the frame over radio, then requests the next
    5. Teensy sends STREAM_STOP to exit

    Args:
        uart_port: Serial port object for UART communication
        camera_index: /dev/video index of the Boson

    Returns:
        str: Reason for exit ('stop_command', 'timeout', 'camera_error',
             'interrupted', 'error')
    """
    cap = open_boson_capture(camera_index)
    if not cap.isOpened():
        print(f"Stream: could not open camera at index {camera_index}")
        return 'camera_error'

    print("Stream mode: waiting for frame requests from Teensy...")

    frame_seq = 0
    frames_sent = 0
    start_time = time.time()
    last_request = time.time()
    rx_buffer = b''

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

            frame = grab_fresh_frame(cap)
            if frame is None:
                print("Stream: failed to read frame from camera")
                continue

            frame_8bit = downsample_for_stream(frame)
            if frame_8bit is None:
                print("Stream: downsample failed")
                continue

            if send_stream_frame_uart(frame_8bit, frame_seq, uart_port):
                frames_sent += 1
                frame_seq = (frame_seq + 1) & 0xFF

                if frames_sent % 10 == 0:
                    elapsed = time.time() - start_time
                    fps = frames_sent / elapsed if elapsed > 0 else 0
                    print(f"Streaming: {frames_sent} frames, {fps:.1f} fps")
            else:
                print("Stream: failed to send frame")

    except KeyboardInterrupt:
        print("Stream interrupted by user")
        return 'interrupted'

    except Exception as e:
        print(f"Stream error: {e}")
        return 'error'

    finally:
        cap.release()

def send_status_uart(message: str, uart_port):
    """
    Send STATUS of payload using the same header/length/end scheme.
    """
    try:
        payload = f"STATUS:{message}".encode("ascii", "ignore")
        return send_data_uart(payload, uart_port)
    except Exception as e:
        print(f"UART status send error: {e}")
        return False

def send_data_uart(data, uart_port):
    """
    Send thermal image data via UART with header and end markers
    
    Transmits thermal data with proper protocol formatting including
    magic bytes, length information, and end markers. Provides
    real-time progress updates and transmission statistics.
    
    Args:
        data: Thermal image data as bytes
        uart_port: Serial port object for UART communication
        
    Returns:
        bool: True if transmission successful, False otherwise
    """
    if not data:
        print("UART: nothing to send (len=0).")
        return False
        
    try:
        # Create header with magic bytes and length
        header = bytearray()
        header.extend([0xDE, 0xAD, 0xBE, 0xEF])  # Magic bytes for validation
        
        # Changed from 2 to 3 byte length in header
        # Add data length (little-endian, 24-bit)
        length = len(data)
        # header.extend([length & 0xFF, (length >> 8) & 0xFF])
        header.extend([length & 0xFF, (length >> 8) & 0xFF, (length >> 16) & 0xFF])
        
        # print("Sending header...")
        uart_port.write(header)
        uart_port.flush()  # Ensure header is transmitted immediately
        
        # Small delay for header processing on receiver
        time.sleep(0.1)
        
        # print("Sending thermal data...")
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

def wait_for_uart_command(uart_port, timeout=None):
    """
    Wait for UART command from Teensy (TRIGGER or STREAM_START/STOP)

    Reads UART data and returns the command type when recognized.

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

            # Check for commands
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


def wait_for_uart_trigger(uart_port, timeout=None):
    """
    Wait for UART trigger command from Teensy (legacy wrapper)

    Returns:
        bool: True if trigger received, False on timeout
    """
    result = wait_for_uart_command(uart_port, timeout)
    return result == 'trigger'

def main():
    print("Boson camera initialized")
    print("RPi Thermal Camera - UART Output")
    print("="*40)

    # UART first (so we can notify Teensy if camera isn't present)
    try:
        uart = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"UART initialized: {UART_PORT} at {UART_BAUD} baud")
    except Exception as e:
        print(f"UART initialization failed: {e}")
        print("Make sure UART is enabled in raspi-config!")
        return

    send_status_uart("BOOT", uart)

    try:
        print(f"\nSystem ready. Waiting for UART commands:")
        print(f"  - {TRIGGER_COMMAND.decode('ascii').strip()}: Single capture")
        print(f"Data will be sent via UART: {UART_PORT} at {UART_BAUD} baud")
        send_status_uart("IDLE", uart)

        while True:
            # Wait for command from Teensy via UART
            command = wait_for_uart_command(uart)

            if command == 'trigger':
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] Capture triggered via UART!")
                print("="*40)
                send_status_uart("CAPTURE_START", uart)

                # NEW CODE HERE (07-07-2026)
                try:
                    index = find_boson_index()
                    print(f"Found boson camera at index {index}")
                except Exception as index_error:
                    print(f"Camera not found: {index_error}")
                    send_status_uart("NO_CAMERA", uart)
                    continue

                try:
                    thermal_data = record_boson_frame(index, uart, True)
                except Exception as cap_error:
                    print(f"Capture error: {cap_error}")
                    thermal_data = None
                    send_status_uart("CAPTURE_ERROR", uart)
                    continue

                # END OF NEW CODE
                
                if thermal_data is None:
                    print("No thermal data available; notifying Teensy.")
                    send_status_uart("NO_FRAMES", uart)
                else:
                    print("\nWaiting 3 seconds for Teensy to prepare...")
                    time.sleep(3)
                    success = send_data_uart(thermal_data, uart)
                    if success:
                        print("✓ Data transmission successful!")
                        send_status_uart("CAPTURE_DONE", uart)
                    else:
                        print("✗ Data transmission failed!")
                        send_status_uart("TX_FAIL", uart)

                print("\nReady for next command...")
                print("="*40)
                send_status_uart("IDLE", uart)

            elif command == 'stream_start':
                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] Stream mode started via UART!")
                print("="*40)

                try:
                    index = find_boson_index()
                    print(f"Found boson camera at index {index}")
                except Exception as index_error:
                    print(f"Camera not found: {index_error}")
                    send_status_uart("NO_CAMERA", uart)
                    send_status_uart("IDLE", uart)
                    continue

                exit_reason = run_stream_loop(uart, index)
                print(f"Stream ended: {exit_reason}")

                print("\nReady for next command...")
                print("="*40)
                send_status_uart("IDLE", uart)

            elif command == 'stream_stop':
                # Stop arrived outside stream mode - nothing to unwind, just ack
                print("Stream stop received (not in stream mode)")
                send_status_uart("IDLE", uart)

    except KeyboardInterrupt:
        print("\nShutting down...")
        send_status_uart("SHUTDOWN", uart)

    except Exception as exc:
        print(f"Unexpected error: {exc}")
        send_status_uart("ERROR", uart)

    finally:
        try:
            uart.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
