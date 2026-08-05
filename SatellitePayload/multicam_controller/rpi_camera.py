'''RPi camera module class for capturing visible-light images via Picamera2.

Mirrors LeptonCamera/BosonCamera's interface (initialize/start_streaming/
capture/get_stream_frame/cleanup) so multicam_controller.py can drive it the
same way. Unlike the thermal cameras, there is no livestream protocol defined
for this camera yet, so get_stream_frame() is a stub that always returns None.

@author: Samantha Mallari
@date: 2026-08-04
'''

import time

import cv2  # type: ignore
from picamera2 import Picamera2  # type: ignore

# Full 1080p still capture. Raw RGB888 at this resolution is ~6.2MB, far too
# large to downlink as-is at the radio's ~38.4kbps link - capture() JPEG-
# compresses before returning, which is what actually keeps this practical.
CAPTURE_WIDTH = 1920
CAPTURE_HEIGHT = 1080
JPEG_QUALITY = 85  # cv2.IMWRITE_JPEG_QUALITY (0-100)

AWB_SETTLE_S = 0.2  # time to let AEC/AWB converge after starting the camera


class RpiCamera:
    """
    RPi camera module (visible light), accessed via Picamera2.

    Capture-only for now: get_stream_frame() returns None since there's no
    existing livestream protocol for RGB/JPEG frames to match against.
    """

    STREAM_FRAME_WIDTH = None
    STREAM_FRAME_HEIGHT = None
    STREAM_FRAME_SIZE = None

    def __init__(self):
        self.picam = None
        self.last_frame = None  # most recent capture() array (BGR, pre-JPEG), for debug dumps

    def initialize(self):
        self.picam = Picamera2()
        self.picam.configure(
            self.picam.create_still_configuration(
                main={"format": "RGB888", "size": (CAPTURE_WIDTH, CAPTURE_HEIGHT)}
            )
        )
        print(f"RPi camera initialized: {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}")

    def start_streaming(self):
        self.picam.start()
        time.sleep(AWB_SETTLE_S)  # allow AEC/AWB to converge
        print("RPi camera started")

    def capture(self, timeout_s=2.0):
        """
        Capture a single frame and return it JPEG-encoded as bytes.

        Returns:
            bytes: JPEG-encoded image data, or None on failure.
        """
        if self.picam is None:
            print("RPi camera capture: camera not started")
            return None

        try:
            # Picamera2's "RGB888" format is actually BGR-ordered, matching cv2,
            # so this can be handed straight to cv2.imencode.
            frame = self.picam.capture_array()
        except Exception as e:
            print(f"RPi camera capture error: {e}")
            return None

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            print("RPi camera capture: JPEG encode failed")
            return None

        print(f"RPi camera frame captured: {frame.shape}, {len(encoded)} bytes JPEG")
        self.last_frame = frame  # kept for debug CSV/image dumps, not UART
        return encoded.tobytes()

    def get_stream_frame(self, max_age_s=1.0):
        # No livestream protocol defined for this camera yet.
        return None

    def cleanup(self):
        try:
            if self.picam is not None:
                self.picam.stop()
                self.picam.close()
        except Exception:
            pass
        finally:
            self.picam = None
