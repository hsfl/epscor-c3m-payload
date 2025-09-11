#!/usr/bin/env python3
"""
Simple Serial Port Reader/Writer
Reads data from serial port and allows user to send commands back
"""

import serial
import threading
import sys
import time
import signal
from serial.tools import list_ports

BAUD_RATE = 115200

def find_serial_port():
    """Find available serial ports"""
    ports = list_ports.comports()
    if not ports:
        return None
    
    # Look for common Teensy/Arduino patterns
    for port in ports:
        desc = (port.description or "").lower()
        if any(keyword in desc for keyword in ["serial", "dual serial", "usbmodem", "arduino", "teensy"]):
            return port.device
    
    # If no specific match, return first available port
    return ports[0].device

def read_serial(ser, stop_event):
    """Read data from serial port in a separate thread"""
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore')
                if data:
                    # Print the data as-is, preserving original newlines
                    print(f"Teensy: {data}", end='', flush=True)
                    
                    # Check for reset message
                    if "Resetting system..." in data:
                        print("\nDetected system reset. Press enter twice to exit.")
                        stop_event.set()
                        # Send SIGINT to interrupt the main thread
                        signal.raise_signal(signal.SIGINT)
                        break  # Exit the read thread immediately
                        
            else:
                # Small sleep to prevent busy waiting
                time.sleep(0.01)
        except Exception as e:
            print(f"Read error: {e}")
            break

def main():
    # Find and connect to serial port
    port = find_serial_port()
    
    if not port:
        print("No serial ports found!")
        return
    
    print(f"Available ports:")
    for p in list_ports.comports():
        print(f"  {p.device}: {p.description}")
    
    # Allow manual port selection
    user_port = input(f"Select port (Press Enter for default: {port}): ").strip()
    if user_port:
        port = user_port
    
    ser = None
    read_thread = None
    stop_event = threading.Event()
    
    try:
        # Open serial connection
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"Connected to {port} at {BAUD_RATE} baud")
        print("Type commands and press Enter to send. Type 'quit' to exit.\n")
        
        # Start reading thread
        read_thread = threading.Thread(target=read_serial, args=(ser, stop_event), daemon=True)
        read_thread.start()
        
        # Main loop for user input
        while True:
            try:
                # Check if reset was detected
                if stop_event.is_set():
                    break
                
                # Print a newline to separate from Teensy output, then get user input
                print()  # This ensures we're on a new line after Teensy output
                user_input = input().strip()
                
                if user_input.lower() in ['quit', 'exit']:
                    break
                
                if user_input:
                    ser.write((user_input + '\n').encode())
                    #useful debug print(f"Sent: {user_input}")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Send error: {e}")
        
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Signal thread to stop
        if stop_event:
            stop_event.set()
        
        # Wait for thread to finish (with timeout)
        if read_thread and read_thread.is_alive():
            print("Stopping read thread...")
            read_thread.join(timeout=2.0)
            if read_thread.is_alive():
                print("Warning: Read thread did not stop cleanly")
        
        # Close serial connection
        if ser and ser.is_open:
            ser.close()
            print("\nSerial connection closed.")

if __name__ == "__main__":
    main()
