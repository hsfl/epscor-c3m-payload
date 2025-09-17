#!/usr/bin/env python3
"""
Simple Serial Port Reader/Writer
Reads data from serial port and allows user to send commands back
Enhanced with CSV export detection and thermal data visualization
"""

import serial
import threading
import sys
import time
import signal
import os
import subprocess
import glob
from serial.tools import list_ports

BAUD_RATE = 115200

# CSV export detection constants
CSV_START_MARKER = "=== START CSV ==="
CSV_END_MARKER = "=== END CSV ==="

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

def get_next_thermal_filename():
    """Get the next available thermal data filename with incrementing number"""
    # Look for existing thermal data files
    existing_files = glob.glob("thermal_data_*.csv")
    
    if not existing_files:
        return "thermal_data_001.csv"
    
    # Extract numbers from existing filenames and find the highest
    max_num = 0
    for filename in existing_files:
        try:
            # Extract number from filename like "thermal_data_001.csv"
            num_str = filename.split('_')[2].split('.')[0]
            num = int(num_str)
            if num > max_num:
                max_num = num
        except (IndexError, ValueError):
            continue
    
    # Return next filename with zero-padded number
    next_num = max_num + 1
    return f"thermal_data_{next_num:03d}.csv"

def save_thermal_data(csv_data, filename):
    """Save CSV data to file"""
    try:
        with open(filename, 'w') as f:
            f.write(csv_data)
        print(f"\n✅ Thermal data saved to: {filename}")
        return True
    except Exception as e:
        print(f"\n❌ Error saving thermal data: {e}")
        return False

def run_thermal_viewer(filename):
    """Run the thermal data viewer with the specified file"""
    try:
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        viewer_script = os.path.join(script_dir, "thermal_data_viewer.py")
        
        if not os.path.exists(viewer_script):
            print(f"❌ Thermal viewer script not found: {viewer_script}")
            return False
        
        # Run the thermal viewer with the filename as argument
        print(f"🖼️  Opening thermal data viewer for: {filename}")
        subprocess.Popen([sys.executable, viewer_script, filename])
        return True
    except Exception as e:
        print(f"❌ Error running thermal viewer: {e}")
        return False

def read_serial(ser, stop_event):
    """Read data from serial port in a separate thread"""
    csv_capture_mode = False
    csv_data = []
    
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore')
                if data:
                    # Print the data as-is, preserving original newlines
                    print(f"{data}", end='', flush=True)
                    
                    # Check for CSV start marker
                    if CSV_START_MARKER in data:
                        print(f"\n🎯 CSV export detected! Starting data capture...")
                        csv_capture_mode = True
                        csv_data = []
                        continue
                    
                    # Check for CSV end marker
                    if CSV_END_MARKER in data:
                        if csv_capture_mode:
                            print(f"\n🏁 CSV export complete! Processing data...")
                            csv_capture_mode = False
                            
                            # Save the captured data
                            csv_content = '\n'.join(csv_data)
                            filename = get_next_thermal_filename()
                            
                            if save_thermal_data(csv_content, filename):
                                # Run the thermal viewer
                                run_thermal_viewer(filename)
                            
                            csv_data = []  # Clear for next capture
                        continue
                    
                    # If in CSV capture mode, collect the data
                    if csv_capture_mode:
                        # Remove the "Teensy: " prefix and store clean data
                        clean_data = data.replace("Teensy: ", "").rstrip('\n\r')
                        if clean_data:  # Only add non-empty lines
                            csv_data.append(clean_data)
                    
                    # Check for reset message
                    if "Resetting system..." in data:
                        print("\nDetected system reset. Press 'Enter' twice to exit.")
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
