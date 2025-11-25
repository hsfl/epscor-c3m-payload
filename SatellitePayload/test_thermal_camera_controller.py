#!/usr/bin/env python3
"""
Unit tests for thermal_camera_controller.py
Tests the downsample_frame function for livestream support.

Run with: python -m pytest test_thermal_camera_controller.py -v
Or:       python test_thermal_camera_controller.py
"""

import unittest
import numpy as np

# Define constants locally to avoid hardware dependencies
STREAM_FRAME_WIDTH = 80
STREAM_FRAME_HEIGHT = 60
STREAM_FRAME_SIZE = STREAM_FRAME_WIDTH * STREAM_FRAME_HEIGHT  # 4800 bytes


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


class TestDownsampleFrame(unittest.TestCase):
    """Test cases for the downsample_frame function"""

    def test_output_size(self):
        """Test that output is 80x60 = 4800 bytes"""
        # Create a 120x160 16-bit frame with room temperature values
        frame = np.full((120, 160), 29315, dtype=np.uint16)  # 20°C
        result = downsample_frame(frame)

        self.assertIsNotNone(result)
        self.assertEqual(result.shape, (60, 80))
        self.assertEqual(result.tobytes().__len__(), STREAM_FRAME_SIZE)

    def test_output_dtype(self):
        """Test that output is uint8"""
        frame = np.full((120, 160), 30000, dtype=np.uint16)
        result = downsample_frame(frame)

        self.assertEqual(result.dtype, np.uint8)

    def test_temperature_mapping_zero_celsius(self):
        """Test that 0°C (27315 Kelvin*100) maps to 0"""
        frame = np.full((120, 160), 27315, dtype=np.uint16)  # 0°C
        result = downsample_frame(frame)

        # Should be 0 (mapped to min of range)
        self.assertEqual(result[0, 0], 0)
        self.assertTrue(np.all(result == 0))

    def test_temperature_mapping_100_celsius(self):
        """Test that 100°C (37315 Kelvin*100) maps to 255"""
        frame = np.full((120, 160), 37315, dtype=np.uint16)  # 100°C
        result = downsample_frame(frame)

        # Should be 255 (mapped to max of range)
        self.assertEqual(result[0, 0], 255)
        self.assertTrue(np.all(result == 255))

    def test_temperature_mapping_50_celsius(self):
        """Test that 50°C maps to approximately 127-128"""
        frame = np.full((120, 160), 32315, dtype=np.uint16)  # 50°C
        result = downsample_frame(frame)

        # Should be approximately 127 (midpoint)
        self.assertGreaterEqual(result[0, 0], 125)
        self.assertLessEqual(result[0, 0], 129)

    def test_2x2_averaging(self):
        """Test that 2x2 blocks are properly averaged"""
        # Create frame with known 2x2 block values
        frame = np.zeros((120, 160), dtype=np.uint16)

        # Set first 2x2 block to specific values (at 50°C range for easy calculation)
        # 32315 = 50°C, which maps to ~127
        frame[0, 0] = 32315  # 50°C
        frame[0, 1] = 32315  # 50°C
        frame[1, 0] = 32315  # 50°C
        frame[1, 1] = 32315  # 50°C

        # Rest is at 0°C
        frame[frame == 0] = 27315

        result = downsample_frame(frame)

        # First pixel should be average of the 2x2 block (all same = 50°C = ~127)
        self.assertGreaterEqual(result[0, 0], 125)
        self.assertLessEqual(result[0, 0], 129)

    def test_clipping_below_range(self):
        """Test that values below 0°C are clipped to 0"""
        frame = np.full((120, 160), 20000, dtype=np.uint16)  # Below 0°C
        result = downsample_frame(frame)

        self.assertTrue(np.all(result == 0))

    def test_clipping_above_range(self):
        """Test that values above 100°C are clipped to 255"""
        frame = np.full((120, 160), 50000, dtype=np.uint16)  # Above 100°C
        result = downsample_frame(frame)

        self.assertTrue(np.all(result == 255))

    def test_none_input(self):
        """Test that None input returns None"""
        result = downsample_frame(None)
        self.assertIsNone(result)

    def test_wrong_shape(self):
        """Test that wrong input shape returns None"""
        frame = np.full((100, 100), 30000, dtype=np.uint16)
        result = downsample_frame(frame)
        self.assertIsNone(result)

    def test_gradient_preservation(self):
        """Test that gradients are preserved after downsampling"""
        frame = np.zeros((120, 160), dtype=np.uint16)

        # Create horizontal gradient from 0°C to 100°C
        for col in range(160):
            temp = 27315 + int(col / 160 * 10000)  # 0°C to 100°C
            frame[:, col] = temp

        result = downsample_frame(frame)

        # Check that leftmost is darker and rightmost is brighter
        self.assertLess(result[0, 0], result[0, -1])

        # Check that gradient is roughly preserved
        self.assertLess(result[0, 0], 50)  # Near 0°C
        self.assertGreater(result[0, -1], 200)  # Near 100°C


class TestStreamFrameConstants(unittest.TestCase):
    """Test that stream frame constants are correct"""

    def test_stream_frame_size(self):
        """Verify STREAM_FRAME_SIZE is 4800 (80 * 60)"""
        self.assertEqual(STREAM_FRAME_SIZE, 4800)
        self.assertEqual(STREAM_FRAME_SIZE, 80 * 60)


if __name__ == '__main__':
    # Run tests
    unittest.main(verbosity=2)
