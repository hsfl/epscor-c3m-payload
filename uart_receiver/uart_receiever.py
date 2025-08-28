#!/usr/bin/env python3
"""
RPi Thermal Camera with UART Data Transfer
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
TRIGGER_PIN = 25	# Input from Teensy

# UART Configuration
UART_PORT = '/dev/serial0'  # Primary UART (GPIO14/15, pins 8/10)
UART_BAUD = 921600         # Match Teensy baud rate

# Camera Configuration
MAX_FRAMES = 100
THERMAL_QUEUE_SIZE = 2
thermal_queue = Queue(THERMAL_QUEUE_SIZE)

class ThermalCamera:
    def __init__(self):
        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()
        self.frame_callback = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(self._frame_callback)

    def _frame_callback(self, frame, userptr):
        try:
            array_pointer = cast(
                frame.contents.data,
                POINTER(c_uint16 * (frame.contents.width * frame.contents.height))
            )
            data = np.frombuffer(
                array_pointer.contents, dtype=np.uint16
            ).reshape(
                frame.contents.height, frame.contents.width
            ).copy()
            
            if not thermal_queue.full():
                thermal_queue.put(data)
        except:
            pass

    def initialize(self):
        libuvc.uvc_init(byref(self.ctx), 0)
        libuvc.uvc_find_device(self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
        libuvc.uvc_open(self.dev, byref(self.devh))
        
        frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_Y16)
        selected_format = frame_formats[0]
        
        libuvc.uvc_get_stream_ctrl_format_size(
            self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
            selected_format.wWidth, selected_format.wHeight,
            int(1e7 / selected_format.dwDefaultFrameInterval)
        )
        
        print(f"Camera initialized: {selected_format.wWidth}x{selected_format.wHeight}")
        return selected_format.wWidth, selected_format.wHeight

    def start_streaming(self):
        libuvc.uvc_start_streaming(self.devh, byref(self.ctrl),
                                 self.frame_callback, None, 0)
        print("Camera streaming started")

    def cleanup(self):
        libuvc.uvc_stop_streaming(self.devh)
        libuvc.uvc_unref_device(self.dev)
        libuvc.uvc_exit(self.ctx)

def capture_and_average():
    """Capture and average thermal frames"""
    frames = []
    
    # Clear queue
    while not thermal_queue.empty():
        thermal_queue.get()
    
    print(f"Capturing {MAX_FRAMES} frames...")
    while len(frames) < MAX_FRAMES:
        try:
            data = thermal_queue.get(timeout=2.0)
            frames.append(data)
            if len(frames) % 20 == 0:
                print(f"Progress: {len(frames)}/{MAX_FRAMES}")
        except:
            continue
    
    print("Averaging frames...")
    averaged = np.mean(np.array(frames, dtype=np.float64), axis=0).astype(np.uint16)
    
    # Calculate stats
    min_val = np.min(averaged)
    max_val = np.max(averaged)
    mean_val = np.mean(averaged)
    
    print(f"Temperature stats:")
    print(f"  Min: {(min_val - 27315) / 100.0:.1f}°C")
    print(f"  Max: {(max_val - 27315) / 100.0:.1f}°C")
    print(f"  Mean: {(mean_val - 27315) / 100.0:.1f}°C")
    
    return averaged.tobytes()

def send_data_uart(data, uart_port):
    """Send data via UART"""
    print(f"\nSending {len(data)} bytes via UART...")
    print(f"UART port: {UART_PORT} at {UART_BAUD} baud")
    
    try:
        # Create header
        header = bytearray()
        header.extend([0xDE, 0xAD, 0xBE, 0xEF])  # Magic bytes
        
        # Add length (little-endian, 16-bit)
        length = len(data)
        header.extend([length & 0xFF, (length >> 8) & 0xFF])
        
        print("Sending header...")
        uart_port.write(header)
        uart_port.flush()
        
        # Small delay for header processing
        time.sleep(0.1)
        
        print("Sending thermal data...")
        start_time = time.time()
        
        # Send data in chunks for better throughput
        chunk_size = 1024  # 1KB chunks
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
        
        # Send end markers
        end_markers = bytearray([0xFF, 0xFF])
        uart_port.write(end_markers)
        uart_port.flush()
        
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
    
    # GPIO setup for trigger
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Initialize UART
    try:
        uart = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"UART initialized: {UART_PORT} at {UART_BAUD} baud")
    except Exception as e:
        print(f"UART initialization failed: {e}")
        print("Make sure UART is enabled in raspi-config!")
        return
    
    # Camera setup
    camera = ThermalCamera()
    
    try:
        camera.initialize()
        camera.start_streaming()
        time.sleep(2)
        
        print(f"\nSystem ready. Waiting for trigger on GPIO{TRIGGER_PIN}")
        print(f"Data will be sent via UART: {UART_PORT} at {UART_BAUD} baud")
        
        while True:
            # Wait for trigger
            GPIO.wait_for_edge(TRIGGER_PIN, GPIO.FALLING)
            timestamp = time.strftime("%H:%M:%S")
            print(f"\n[{timestamp}] Capture triggered!")
            print("="*40)
            
            # Capture and average
            thermal_data = capture_and_average()
            
            if thermal_data:
                # Wait for Teensy to be ready
                print("\nWaiting 3 seconds for Teensy to prepare...")
                time.sleep(3)
                
                # Send data via UART
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
        camera.cleanup()
        uart.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
