'''Lepton camera class for capturing thermal images from a FLIR Lepton module.

Defines a LeptonCamera class that interfaces with the Lepton camera using libuvc.
basic functionality:
1. Initialize the camera and start streaming.
2. Capture frames via a callback and store them in a queue.
3. Provide methods to retrieve the latest frame for streaming and to capture a single still frame.

@author: Samantha Mallari
@date: 2026-08-04
'''

import numpy as np
import time
import struct
import serial
from uvctypes import *
import cv2 # type: ignore
from queue import Queue, Empty, Full
import ctypes
import threading

# Camera Configuration
THERMAL_QUEUE_SIZE = 20   # Size of frame queue for thermal data

FRAME_CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.POINTER(uvc_frame), ctypes.c_void_p)

STREAM_FRAME_WIDTH = 80   # Downsampled width (160/2)
STREAM_FRAME_HEIGHT = 60  # Downsampled height (120/2)
STREAM_FRAME_SIZE = STREAM_FRAME_WIDTH * STREAM_FRAME_HEIGHT  # 4,800 bytes per frame


def is_valid_frame(frame):
    """Skip FFC frames"""
    zero_count = np.sum(frame < 1000)
    return zero_count < (frame.size * 0.8)


def downsample_frame(frame_16bit):
    """
    Downsample 160x120 16-bit thermal frame to 80x60 8-bit for streaming.

    Uses 2x2 block averaging and scales 16-bit values to 8-bit range.
    The 16-bit thermal values are in Kelvin*100 (e.g., 29315 = 20°C).
    We map a reasonable temperature range (0-100°C = 27315-37315) to 0-255.

    Args:
        frame_16bit: numpy array of shape (120, 160) with uint16 values

    Returns:
        numpy array of shape (60, 80) with uint8 values
    """
    if frame_16bit is None:
        return None

    # Ensure correct input shape
    if frame_16bit.shape != (120, 160):
        print(f"Warning: unexpected frame shape {frame_16bit.shape}, expected (120, 160)")
        return None

    # Reshape to (60, 2, 80, 2) to get 2x2 blocks, then average
    # This groups pixels into 2x2 blocks for averaging
    reshaped = frame_16bit.reshape(60, 2, 80, 2)
    averaged = reshaped.mean(axis=(1, 3)).astype(np.float32)

    # Map from Kelvin*100 to 0-255 range
    # Temperature range: 0°C (27315) to 100°C (37315)
    min_kelvin = 27315.0  # 0°C in Kelvin*100
    max_kelvin = 37315.0  # 100°C in Kelvin*100

    # Clip to valid range and scale to 0-255
    clipped = np.clip(averaged, min_kelvin, max_kelvin)
    scaled = ((clipped - min_kelvin) / (max_kelvin - min_kelvin) * 255.0)

    return scaled.astype(np.uint8)


class LeptonCamera:
    """
    FLIR Lepton thermal camera, accessed via libuvc.

    Each instance owns its own frame queue and latest-frame buffer, so
    multiple LeptonCamera instances (or a LeptonCamera alongside other
    camera types) can run in the same process without cross-contaminating
    each other's frames.
    """

    STREAM_FRAME_WIDTH = STREAM_FRAME_WIDTH
    STREAM_FRAME_HEIGHT = STREAM_FRAME_HEIGHT
    STREAM_FRAME_SIZE = STREAM_FRAME_SIZE

    def __init__(self):
        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()
        self.frame_callback = FRAME_CALLBACK_TYPE(self._frame_callback)
        self.streaming = False

        self.last_frame = None  # most recent capture() array, for debug CSV/image dumps
        self.thermal_queue = Queue(THERMAL_QUEUE_SIZE)
        self._latest_frame = None
        self._latest_frame_lock = threading.Lock()
        self._latest_frame_timestamp = 0

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

            # Update latest frame buffer (for streaming - always the freshest frame)
            with self._latest_frame_lock:
                self._latest_frame = thermal
                self._latest_frame_timestamp = time.time()

            # Also put in queue (for frame averaging in single captures)
            try:
                self.thermal_queue.put_nowait(thermal)
            except Full:
                # Drop the oldest frame and retry to keep the queue flowing
                try:
                    self.thermal_queue.get_nowait()
                except Empty:
                    pass

                try:
                    self.thermal_queue.put_nowait(thermal)
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

    def get_latest_frame(self, max_age_s=1.0):
        """
        Get the most recent frame from the camera (for streaming).

        This is more efficient than using the queue for streaming because:
        1. No queue overhead (put/get operations)
        2. Always returns the freshest frame, not a stale queued one
        3. Simple lock-based access

        Args:
            max_age_s: Maximum age of frame in seconds (default 1.0s)

        Returns:
            numpy array of frame data, or None if no recent frame available
        """
        with self._latest_frame_lock:
            if self._latest_frame is None:
                return None

            # Check if frame is too old
            age = time.time() - self._latest_frame_timestamp
            if age > max_age_s:
                return None

            # Return a copy to avoid race conditions
            return self._latest_frame.copy()

    def get_stream_frame(self, max_age_s=1.0):
        """
        Get the latest frame, downsampled and ready for send_stream_frame_uart.

        Returns:
            numpy array of shape (60, 80) uint8, or None if unavailable/invalid.
        """
        frame = self.get_latest_frame(max_age_s=max_age_s)
        if frame is None or not is_valid_frame(frame):
            return None
        return downsample_frame(frame)

    def capture(self, timeout_s=2.0):
        """
        Capture and return a single thermal frame.

        Pulls one valid (non-FFC) frame from the camera queue and returns
        it as raw bytes.

        Returns:
            bytes: Thermal image data as raw bytes, or None if no valid
            frame arrived before timeout_s.
        """
        # Empty the queue first so we grab the freshest frame, not a stale one.
        while True:
            try:
                self.thermal_queue.get_nowait()
            except Empty:
                break

        print(f"Capturing 1 frame (timeout {timeout_s:.1f}s)...")
        deadline = time.time() + timeout_s

        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                print("No frame captured before timeout.")
                return None
            try:
                frame = self.thermal_queue.get(timeout=min(0.5, max(0.05, remaining)))
            except Empty:
                continue
            if is_valid_frame(frame):
                self.last_frame = frame  # kept for debug CSV/image dumps, not UART
                return frame.tobytes()  # Return as raw bytes for UART transmission

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
