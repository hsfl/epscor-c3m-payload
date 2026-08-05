'''Boson camera class for capturing thermal images from a FLIR Boson module.

Wraps the same capture logic from legacy boson_controller.py in a BosonCamera class,
essentially decoupling the camera related functions from the UART protocol 
in boson_controller.py.

@author: Samantha Mallari
@date: 2026-08-04
'''

import numpy as np
import cv2
import pyudev
import re

BOSON_IMAGE_HEIGHT = 256

# The Teensy and ground station both hardcode a 100x80 8-bit frame (8000 bytes),
# so these values must not change without updating satellite_teensy.ino and
# ground_station_serial_cli_teensy.py together.
STREAM_FRAME_WIDTH = 100
STREAM_FRAME_HEIGHT = 80
STREAM_FRAME_SIZE = STREAM_FRAME_WIDTH * STREAM_FRAME_HEIGHT  # 8000 bytes per frame

# Kept low deliberately: the Teensy only waits STREAM_HEADER_TIMEOUT_MS (1s) after
# sending FRAME, and each drained grab costs one frame interval - 111ms on a 9Hz
# export-restricted Boson. Raising this risks blowing that budget.
STREAM_MAX_DRAIN = 2


def find_boson_index(vendor_id="09cb"):
    """Scan /dev/video* devices and return the index whose USB vendor ID matches the FLIR Boson camera."""
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


def normalize_thermal(frame16, low_pct=1, high_pct=99):
    """Normalize 16-bit thermal data to 8-bit using percentile clipping."""
    lo, hi = np.percentile(frame16, (low_pct, high_pct))
    if hi <= lo:
        hi = lo + 1  # avoid div-by-zero on a flat frame
    clipped = np.clip(frame16, lo, hi)
    norm = ((clipped - lo) / (hi - lo) * 255).astype(np.uint8)
    return norm


def strip_telemetry_rows(frame16):
    """Drop the telemetry rows the Boson prepends to each frame."""
    if frame16.shape[0] > BOSON_IMAGE_HEIGHT:
        return frame16[-BOSON_IMAGE_HEIGHT:, :]
    return frame16


def downsample_for_stream(frame16):
    """
    Reduce a full-resolution 16-bit Boson frame to a 100x80 8-bit stream frame.

    The source is centre-cropped to the stream's aspect ratio before resizing so the
    preview is not distorted, then area-averaged down. Both steps are derived from
    the frame's own shape, so this holds for the 320x256 and 640x512 Boson variants.
    """
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


class BosonCamera:
    """
    FLIR Boson thermal camera, accessed via V4L2/OpenCV.

    Mirrors LeptonCamera's interface so multicam_controller.py can drive
    either camera through the same initialize/start_streaming/capture/
    get_stream_frame/cleanup calls.
    """

    STREAM_FRAME_WIDTH = STREAM_FRAME_WIDTH
    STREAM_FRAME_HEIGHT = STREAM_FRAME_HEIGHT
    STREAM_FRAME_SIZE = STREAM_FRAME_SIZE

    def __init__(self):
        self.camera_index = None
        self.cap = None
        self.streaming = False
        self.last_frame = None  # most recent capture() array, for debug CSV/image dumps

    def initialize(self):
        self.camera_index = find_boson_index()
        print(f"Boson camera found at index {self.camera_index}")
        return self.camera_index

    def start_streaming(self):
        # There's no push-frame callback for the Boson - "streaming" here just
        # means opening a persistent V4L2 capture so capture()/get_stream_frame()
        # don't pay the cv2.VideoCapture open cost on every call.
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'Y16 '))
        # Ask for the shallowest buffer available; V4L2 may ignore this, which is
        # why _grab_fresh_frame also drains explicitly.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open Boson camera at index {self.camera_index}")

        self.streaming = True
        print("Boson camera streaming started")

    def _grab_fresh_frame(self, max_drain=STREAM_MAX_DRAIN):
        """
        Return the newest frame available, as 16-bit data with telemetry rows
        removed. V4L2 hands back the OLDEST buffered frame, so the queue is
        drained first to keep captures/previews current.
        """
        for _ in range(max_drain):
            if not self.cap.grab():
                break

        ret, frame = self.cap.read()
        if not ret:
            return None

        frame = np.squeeze(frame)
        if frame.dtype != np.uint16:
            frame = np.left_shift(frame.astype(np.uint16), 8)

        return strip_telemetry_rows(frame)

    def capture(self, timeout_s=2.0):
        """
        Capture a single Boson frame (no averaging - the Boson has much less
        per-frame noise than the Lepton) and return it as raw bytes.

        Returns:
            bytes: thermal image data as raw bytes, or None on failure.
        """
        if self.cap is None or not self.cap.isOpened():
            print("Boson capture: camera not streaming")
            return None

        frame16 = self._grab_fresh_frame()
        if frame16 is None:
            print("Boson capture: failed to read frame")
            return None

        print(f"Boson frame shape: {frame16.shape}")
        self.last_frame = frame16  # kept for debug CSV/image dumps, not UART
        return frame16.tobytes()

    def get_stream_frame(self, max_age_s=1.0):
        """
        Grab the freshest frame, downsampled and ready for send_stream_frame_uart.

        Returns:
            numpy array of shape (80, 100) uint8, or None if unavailable.
        """
        if self.cap is None or not self.cap.isOpened():
            return None

        frame16 = self._grab_fresh_frame()
        if frame16 is None:
            return None

        return downsample_for_stream(frame16)

    def cleanup(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        finally:
            self.cap = None
            self.streaming = False
