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
from queue import Queue
import ctypes

# GPIO Pin Configuration
TRIGGER_PIN = 25	# Input from Teensy for capture trigger

# UART Configuration for Teensy communication
UART_PORT = '/dev/serial0'  # Primary UART (GPIO14/15, pins 8/10)
UART_BAUD = 921600         # High-speed UART to match Teensy baud rate

# Camera Configuration
MAX_FRAMES = 1           # Number of frames to capture and average
THERMAL_QUEUE_SIZE = 2     # Size of frame queue for thermal data
thermal_queue = Queue(THERMAL_QUEUE_SIZE)  # Queue for thermal frame data

class ThermalCamera:
    """
    Thermal camera interface class for USB thermal cameras
    
    Provides a high-level interface to USB thermal cameras using the libuvc
    library. Handles camera initialization, frame capture, and data processing.
    Supports frame averaging for noise reduction and real-time data streaming.
    
    Attributes:
        ctx: UVC context pointer
        dev: UVC device pointer  
        devh: UVC device handle pointer
        ctrl: Stream control structure
        frame_callback: Frame callback function
    """
    
    def __init__(self):
        """Initialize thermal camera object and set up frame callback"""
        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()
        # Set up frame callback function for receiving thermal data
        self.frame_callback = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(self._frame_callback)

    def _frame_callback(self, frame, userptr):
        """
        Callback function called when a new thermal frame is available
        
        Converts raw frame data to numpy array and adds it to the thermal queue
        for processing. Handles data type conversion and memory management.
        
        Args:
            frame: Pointer to UVC frame structure containing thermal data
            userptr: User pointer (unused)
        """
        try:
            # Cast frame data to uint16 array pointer
            array_pointer = cast(
                frame.contents.data,
                POINTER(c_uint16 * (frame.contents.width * frame.contents.height))
            )
            # Convert to numpy array and reshape to image dimensions
            data = np.frombuffer(
                array_pointer.contents, dtype=np.uint16
            ).reshape(
                frame.contents.height, frame.contents.width
            ).copy()
            
            # Add frame to queue if space available
            if not thermal_queue.full():
                thermal_queue.put(data)
        except:
            pass  # Ignore frame processing errors

    def initialize(self):
        """
        Initialize the thermal camera and configure streaming
        
        Sets up UVC context, finds and opens the thermal camera device,
        configures frame format and streaming parameters.
        
        Returns:
            tuple: (width, height) of the thermal camera resolution
        """
        # Initialize UVC library
        libuvc.uvc_init(byref(self.ctx), 0)
        # Find thermal camera device by vendor/product ID
        libuvc.uvc_find_device(self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
        # Open device and get handle
        libuvc.uvc_open(self.dev, byref(self.devh))
        
        # Get available frame formats for Y16 (16-bit grayscale)
        frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_Y16)
        selected_format = frame_formats[0]
        
        # Configure stream control for selected format
        libuvc.uvc_get_stream_ctrl_format_size(
            self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
            selected_format.wWidth, selected_format.wHeight,
            int(1e7 / selected_format.dwDefaultFrameInterval)
        )
        
        print(f"Camera initialized: {selected_format.wWidth}x{selected_format.wHeight}")
        return selected_format.wWidth, selected_format.wHeight

    def start_streaming(self):
        """Start thermal camera streaming with frame callback"""
        libuvc.uvc_start_streaming(self.devh, byref(self.ctrl),
                                 self.frame_callback, None, 0)
        print("Camera streaming started")

    def cleanup(self):
        """Clean up camera resources and stop streaming"""
        libuvc.uvc_stop_streaming(self.devh)
        libuvc.uvc_unref_device(self.dev)
        libuvc.uvc_exit(self.ctx)

def capture_and_average():
    """
    Capture and average multiple thermal frames for noise reduction
    
    Collects MAX_FRAMES thermal frames from the camera queue, averages them
    to reduce noise, and returns the averaged data as bytes. Provides
    real-time progress updates and temperature statistics.
    
    Returns:
        bytes: Averaged thermal image data as raw bytes
    """
    frames = []
    
    # Clear any existing frames in queue
    while not thermal_queue.empty():
        thermal_queue.get()
    
    print(f"Capturing {MAX_FRAMES} frames...")
    while len(frames) < MAX_FRAMES:
        try:
            # Get frame from queue with timeout
            data = thermal_queue.get(timeout=2.0)
            frames.append(data)
            if len(frames) % 20 == 0:
                print(f"Progress: {len(frames)}/{MAX_FRAMES}")
        except:
            continue  # Continue if timeout occurs
    
    print("Averaging frames...")
    # Convert frames to float64 for averaging, then back to uint16
    averaged = np.mean(np.array(frames, dtype=np.float64), axis=0).astype(np.uint16)
    
    # Calculate and display temperature statistics
    min_val = np.min(averaged)
    max_val = np.max(averaged)
    mean_val = np.mean(averaged)
    
    print(f"Temperature stats:")
    print(f"  Min: {(min_val - 27315) / 100.0:.1f}°C")  # Convert from Kelvin*100
    print(f"  Max: {(max_val - 27315) / 100.0:.1f}°C")
    print(f"  Mean: {(mean_val - 27315) / 100.0:.1f}°C")
    
    return averaged.tobytes()  # Return as raw bytes for UART transmission

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
    """
    Main function - initializes system and runs capture loop
    
    Sets up GPIO, UART, and thermal camera. Enters continuous loop
    waiting for trigger signals from Teensy, capturing thermal images,
    and transmitting data via UART.
    """
    print("RPi Thermal Camera - UART Output")
    print("="*40)
    
    # GPIO setup for trigger input from Teensy
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Initialize UART communication with Teensy
    try:
        uart = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"UART initialized: {UART_PORT} at {UART_BAUD} baud")
    except Exception as e:
        print(f"UART initialization failed: {e}")
        print("Make sure UART is enabled in raspi-config!")
        return
    
    # Initialize thermal camera
    camera = ThermalCamera()
    
    try:
        camera.initialize() #TODO: revise so it starts streaming ONCE you get the signal from Teensy.
        camera.start_streaming()
        time.sleep(2)  # Allow camera to stabilize
        
        print(f"\nSystem ready. Waiting for trigger on GPIO{TRIGGER_PIN}")
        print(f"Data will be sent via UART: {UART_PORT} at {UART_BAUD} baud")
        
        # Main capture loop
        while True:
            # Wait for trigger signal from Teensy (falling edge)
            GPIO.wait_for_edge(TRIGGER_PIN, GPIO.FALLING)
            timestamp = time.strftime("%H:%M:%S")
            print(f"\n[{timestamp}] Capture triggered!")
            print("="*40)
            
            # Capture and average thermal frames
            thermal_data = capture_and_average()
            
            if thermal_data:
                # Wait for Teensy to be ready for data reception
                print("\nWaiting 3 seconds for Teensy to prepare...")
                time.sleep(3)
                
                # Send thermal data via UART
                success = send_data_uart(thermal_data, uart)
                
                if success:
                    print("✓ Data transmission successful!")
                else:
                    print("✗ Data transmission failed!")
            
            print("\nReady for next trigger...")
            print("="*40)
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    finally:
        # Clean up resources
        camera.cleanup()
        uart.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
