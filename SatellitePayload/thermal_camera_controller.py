#!/usr/bin/env python3
"""
RPi Thermal Camera Controller with UART Data Transfer
EPSCOR C3M Payload - Satellite Raspberry Pi Component

This Python script runs on a Raspberry Pi in the satellite payload to capture
thermal images using a USB thermal camera and transmit the data to a Teensy
microcontroller via UART for radio transmission to the ground station.

Key Features:
- Captures thermal images using USB thermal camera (FLIR/Seek Thermal)
- Averages multiple frames for noise reduction
- Transmits data via UART with header/end markers
- Provides real-time progress updates and data validation
- Responds to trigger signals from Teensy microcontroller

Hardware Requirements:
- Raspberry Pi (3B+ or 4 recommended)
- USB thermal camera (FLIR ONE, Seek Thermal, etc.)
- UART connection to Teensy microcontroller
- GPIO trigger input from Teensy

Communication Protocol:
- UART: 921600 baud, 8N1
- Header: Magic bytes (0xDE 0xAD 0xBE 0xEF) + 16-bit length
- Data: Raw thermal image data (16-bit per pixel)
- End: Magic bytes (0xFF 0xFF)

@author EPSCOR C3M Team
@date 8/25/2025
@version 1.0
"""

import RPi.GPIO as GPIO
import numpy as np
import time
import struct
import serial
from uvctypes import *
import cv2
from queue import Queue, Empty, Full
import ctypes

# GPIO Pin Configuration
TRIGGER_PIN = 25	# Input from Teensy for capture trigger

# UART Configuration for Teensy communication
UART_PORT = '/dev/serial0'  # Primary UART (GPIO14/15, pins 8/10)
UART_BAUD = 115200         # High-speed UART to match Teensy baud rate

# Camera Configuration
MAX_FRAMES = 10           # Number of frames to capture and average
THERMAL_QUEUE_SIZE = 2     # Size of frame queue for thermal data
thermal_queue = Queue(THERMAL_QUEUE_SIZE)  # Queue for thermal frame data

FRAME_CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.POINTER(uvc_frame), ctypes.c_void_p)

import sys


class ThermalCamera:
    def __init__(self):
        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()
        self.frame_callback = FRAME_CALLBACK_TYPE(self._frame_callback)
        self.streaming = False

    def _frame_callback(self, frame_ptr, userptr):
        if not frame_ptr:
            return

        try:
            frame = frame_ptr.contents
            if not bool(frame.data):
                return

            size = frame.width * frame.height
            if size <= 0:
                return

            array_pointer = ctypes.cast(
                frame.data, ctypes.POINTER(ctypes.c_uint16 * size)
            )
            thermal = np.frombuffer(array_pointer.contents, dtype=np.uint16).reshape(
                frame.height, frame.width
            ).copy()

            try:
                thermal_queue.put_nowait(thermal)
            except Full:
                # Drop the oldest frame and retry to keep the queue flowing
                try:
                    thermal_queue.get_nowait()
                except Empty:
                    pass
                try:
                    thermal_queue.put_nowait(thermal)
                except Full:
                    pass
        except Exception:
            # Never propagate exceptions across the C callback boundary.
            return

    def initialize(self):
        # Context setup mirrors known-good original implementation
        res = libuvc.uvc_init(byref(self.ctx), 0)
        if res < 0 or not bool(self.ctx):
            raise RuntimeError(f"uvc_init failed: {res}")

        vid = globals().get("PT_USB_VID", 0)
        pid = globals().get("PT_USB_PID", 0)

        res = libuvc.uvc_find_device(self.ctx, byref(self.dev), vid, pid, 0)
        if res < 0 or not bool(self.dev):
            raise FileNotFoundError("No UVC thermal camera found (uvc_find_device).")

        res = libuvc.uvc_open(self.dev, byref(self.devh))
        if res < 0 or not bool(self.devh):
            raise RuntimeError(f"uvc_open failed: {res}")

        frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_Y16)
        if not frame_formats:
            raise RuntimeError("No Y16 frame formats available.")

        selected_format = frame_formats[0]
        if selected_format.wWidth == 0 or selected_format.wHeight == 0:
            raise RuntimeError("Invalid Y16 format: width/height is zero.")

        default_interval = selected_format.dwDefaultFrameInterval
        if not default_interval:
            default_interval = int(1e7 / 9)

        fps = int(1e7 / default_interval) if default_interval else 9
        if fps <= 0:
            fps = 9

        res = libuvc.uvc_get_stream_ctrl_format_size(
            self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
            selected_format.wWidth, selected_format.wHeight, fps
        )
        if res < 0:
            raise RuntimeError(f"uvc_get_stream_ctrl_format_size failed: {res}")

        print(f"Camera initialized: {selected_format.wWidth}x{selected_format.wHeight}")
        return selected_format.wWidth, selected_format.wHeight

    def start_streaming(self):
        res = libuvc.uvc_start_streaming(self.devh, byref(self.ctrl), self.frame_callback, None, 0)
        if res < 0:
            raise RuntimeError(f"uvc_start_streaming failed: {res}")
        self.streaming = True
        print("Camera streaming started")

    def cleanup(self):
        try:
            if self.streaming and bool(self.devh):
                libuvc.uvc_stop_streaming(self.devh)
        except Exception:
            pass
        finally:
            self.streaming = False

        try:
            if bool(self.dev):
                libuvc.uvc_unref_device(self.dev)
        except Exception:
            pass

        try:
            if bool(self.ctx):
                libuvc.uvc_exit(self.ctx)
        except Exception:
            pass

def capture_and_average(timeout_s=8.0):
    """
    Capture and average multiple thermal frames for noise reduction
    
    Collects MAX_FRAMES thermal frames from the camera queue, averages them
    to reduce noise, and returns the averaged data as bytes. Provides
    real-time progress updates and temperature statistics.
    
    Returns:
        bytes: Averaged thermal image data as raw bytes
    """
    frames = []

    while True:
        try:
            thermal_queue.get_nowait()
        except Empty:
            break

    print(f"Capturing up to {MAX_FRAMES} frames (timeout {timeout_s:.1f}s)...")
    deadline = time.time() + timeout_s

    while len(frames) < MAX_FRAMES:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        try:
            frame = thermal_queue.get(timeout=min(0.5, max(0.05, remaining)))
            frames.append(frame)
            print(f"Captured {len(frames)}/{MAX_FRAMES} frames")
        except Empty:
            continue

    if not frames:
        print("No frames captured before timeout.")
        return None

    if len(frames) < MAX_FRAMES:
        print(f"Only captured {len(frames)} frame(s); averaging nonetheless.")

    print("Averaging frames...")
    stacked = np.stack(frames, axis=0).astype(np.float64)
    averaged = np.mean(stacked, axis=0).astype(np.uint16)
    
    # Calculate and display temperature statistics
    min_val = np.min(averaged)
    max_val = np.max(averaged)
    mean_val = np.mean(averaged)
    
    print(f"Temperature stats:")
    print(f"  Min: {(min_val - 27315) / 100.0:.1f}°C")  # Convert from Kelvin*100
    print(f"  Max: {(max_val - 27315) / 100.0:.1f}°C")
    print(f"  Mean: {(mean_val - 27315) / 100.0:.1f}°C")
    
    return averaged.tobytes()  # Return as raw bytes for UART transmission

def send_status_uart(message: str, uart_port):
    """
    Send a human-readable STATUS payload using the same header/length/end scheme.
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
    
    print(f"\nSending {len(data)} bytes via UART...")
    print(f"UART port: {UART_PORT} at {UART_BAUD} baud")
    
    try:
        # Create header with magic bytes and length
        header = bytearray()
        header.extend([0xDE, 0xAD, 0xBE, 0xEF])  # Magic bytes for validation
        
        # Add data length (little-endian, 16-bit)
        length = len(data)
        header.extend([length & 0xFF, (length >> 8) & 0xFF])
        
        print("Sending header...")
        uart_port.write(header)
        uart_port.flush()  # Ensure header is transmitted immediately
        
        # Small delay for header processing on receiver
        time.sleep(0.1)
        
        print("Sending thermal data...")
        start_time = time.time()
        
        # Send data in chunks for better throughput and progress tracking
        chunk_size = 1024  # 1KB chunks for optimal transmission
        total_sent = 0
        
        while total_sent < length:
            chunk_end = min(total_sent + chunk_size, length)
            chunk = data[total_sent:chunk_end]
            
            uart_port.write(chunk)
            total_sent += len(chunk)
            
            # Progress update every 4KB
            if total_sent % 4096 == 0 or total_sent == length:
                elapsed = time.time() - start_time
                rate = total_sent / elapsed if elapsed > 0 else 0
                eta = (length - total_sent) / rate if rate > 0 else 0
                
                print(f"Progress: {total_sent}/{length} bytes "
                      f"({total_sent/length*100:.1f}%) - "
                      f"{rate:.0f} bytes/sec - ETA: {eta:.1f}s")
        
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

def main():
    print("RPi Thermal Camera - UART Output")
    print("="*40)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # UART first (so we can notify Teensy if camera isn't present)
    try:
        uart = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"UART initialized: {UART_PORT} at {UART_BAUD} baud")
    except Exception as e:
        print(f"UART initialization failed: {e}")
        print("Make sure UART is enabled in raspi-config!")
        return

    send_status_uart("BOOT", uart)

    camera = ThermalCamera()

    try:
        # Try to bring up the camera; if it fails, notify and exit gracefully.
        try:
            camera.initialize()
            camera.start_streaming()
            time.sleep(2)  # allow to stabilize
            send_status_uart("CAM_READY", uart)
        except Exception as cam_err:
            print(f"Camera not ready: {cam_err}")
            # Send a STATUS packet to Teensy so it can display/log the reason
            send_status_uart("NO_CAMERA", uart)
            # Clean exit so systemd can retry later
            return

        print(f"\nSystem ready. Waiting for trigger on GPIO{TRIGGER_PIN}")
        print(f"Data will be sent via UART: {UART_PORT} at {UART_BAUD} baud")
        send_status_uart("IDLE", uart)

        while True:
            GPIO.wait_for_edge(TRIGGER_PIN, GPIO.FALLING)
            timestamp = time.strftime("%H:%M:%S")
            print(f"\n[{timestamp}] Capture triggered!")
            print("="*40)
            send_status_uart("CAPTURE_START", uart)

            thermal_data = capture_and_average(timeout_s=8.0)

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

            print("\nReady for next trigger...")
            print("="*40)
            send_status_uart("IDLE", uart)

    except KeyboardInterrupt:
        print("\nShutting down...")
        send_status_uart("SHUTDOWN", uart)

    except Exception as exc:
        print(f"Unexpected error: {exc}")
        send_status_uart("ERROR", uart)

    finally:
        try:
            camera.cleanup()
        except Exception:
            pass
        try:
            uart.close()
        except Exception:
            pass
        GPIO.cleanup()

if __name__ == "__main__":
    main()
