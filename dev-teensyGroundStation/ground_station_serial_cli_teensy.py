#!/usr/bin/env python3
"""
Ground Station Serial CLI for Teensy
EPSCOR C3M Payload - Ground Station Component

Reads data from serial port and allows user to send commands back
Enhanced with CSV export detection and thermal data visualization

@author EPSCOR C3M Team
@date 2025-10-14
@version 1.0.0
"""

__version__ = "1.0.0"
__build_date__ = "2025-10-14"

import serial
import threading
import sys
import time
import signal
import os
import subprocess
import glob
import re
from serial.tools import list_ports
from datetime import datetime

BAUD_RATE = 115200

# CSV export detection constants
THERMAL_CSV_START = "=== START CSV ==="
THERMAL_CSV_END = "=== END CSV ==="
THERMAL_CAPTURE_TIMESTAMP_FLAG = "--- UART THERMAL CAPTURE ---"
ThermalCaptureTimestamp = datetime.now()

# Sensor telemetry cache
sensor_cache = {"gps": {}, "imu": {}}
_current_sensor_section = None


def _reset_sensor_section(section: str):
    """Prepare cache for a fresh sensor block."""
    if section not in sensor_cache:
        return
    sensor_cache[section] = {
        "last_update": datetime.now().isoformat(timespec="seconds") + "Z"
    }


def _ensure_section(section: str):
    if section not in sensor_cache or not isinstance(sensor_cache[section], dict):
        sensor_cache[section] = {}
    sensor_cache[section].setdefault(
        "last_update", datetime.now().isoformat(timespec="seconds") + "Z"
    )
    return sensor_cache[section]


NUMBER_PATTERN = re.compile(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")


def _safe_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _extract_first_float(text):
    match = NUMBER_PATTERN.search(text)
    if match:
        return _safe_float(match.group(0))
    return None


def _sanitize_key(label: str) -> str:
    label = label.strip().lower()
    label = label.replace("(", " ").replace(")", " ")
    label = re.sub(r"[^a-z0-9]+", "_", label)
    return label.strip("_")


def _store_generic(section: str, label: str, value: str):
    key = _sanitize_key(label)
    if not key:
        return
    section_dict = _ensure_section(section)
    section_dict[key] = value.strip()


def _parse_triple(line):
    match = re.search(
        r"x\s*=\s*([-+0-9.eE]+).*,\s*y\s*=\s*([-+0-9.eE]+).*,\s*z\s*=\s*([-+0-9.eE]+)",
        line,
    )
    if not match:
        return None
    try:
        return tuple(float(match.group(i)) for i in range(1, 4))
    except ValueError:
        return None


def update_sensor_cache(line: str):
    """Track GPS/IMU telemetry blocks emitted by the satellite."""
    global _current_sensor_section

    if not line:
        return

    text = line.strip("\r\n")
    if text.startswith("SAT> "):
        text = text[5:]
    if text.startswith("Teensy: "):
        text = text[8:]

    stripped = text.strip()
    if not stripped:
        return

    upper = stripped.upper()

    if "SENSOR DATA" in upper and ("---" in stripped or "===" in stripped):
        _current_sensor_section = None
        return

    if "GPS DATA" in upper and ("---" in stripped or "===" in stripped):
        _reset_sensor_section("gps")
        _current_sensor_section = "gps"
        return

    if "IMU DATA" in upper and ("---" in stripped or "===" in stripped):
        _reset_sensor_section("imu")
        _current_sensor_section = "imu"
        return

    if stripped.startswith("===") and "DATA" not in upper:
        _current_sensor_section = None
        return

    if _current_sensor_section is None:
        return

    handled = False
    if _current_sensor_section == "gps":
        handled = _parse_gps_line(stripped)
    elif _current_sensor_section == "imu":
        handled = _parse_imu_line(stripped)

    if handled:
        return

    if ":" in stripped:
        label, value = stripped.split(":", 1)
        _store_generic(_current_sensor_section, label, value)


def _parse_gps_line(line: str):
    gps = _ensure_section("gps")

    if line.startswith("GPS not connected"):
        gps["status"] = "not_connected"
        gps.pop("fix_status", None)
        return True

    if line.startswith("No GPS fix"):
        gps["fix_status"] = "no_fix"
        gps["status"] = "searching_fix"
        return True

    if line.startswith("Satellites visible:"):
        value = _extract_first_float(line)
        if value is not None:
            gps["satellites_visible"] = int(value)
        return True

    if line.startswith("Time (UTC):"):
        gps["time_utc"] = line.split(":", 1)[1].strip()
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Date:"):
        gps["date"] = line.split(":", 1)[1].strip()
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Location:"):
        gps["location_raw"] = line.split(":", 1)[1].strip()
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Decimal:"):
        try:
            _, payload = line.split(":", 1)
            lat_str, lon_str = payload.split(",", 1)
            lat = _safe_float(lat_str.strip())
            lon = _safe_float(lon_str.strip())
            if lat is not None:
                gps["latitude_deg"] = lat
            if lon is not None:
                gps["longitude_deg"] = lon
        except ValueError:
            pass
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Altitude:"):
        value = _extract_first_float(line)
        if value is not None:
            gps["altitude_m"] = value
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Speed:"):
        value = _extract_first_float(line)
        if value is not None:
            gps["speed_knots"] = value
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Satellites:"):
        value = _extract_first_float(line)
        if value is not None:
            gps["satellites_in_use"] = int(value)
        gps.setdefault("status", "ok")
        return True

    if line.startswith("Fix quality:"):
        value = _extract_first_float(line)
        if value is not None:
            gps["fix_quality"] = int(value)
            if value > 0:
                gps["fix_status"] = "fix"
        gps.setdefault("status", "ok")
        return True

    return False


def _parse_imu_line(line: str):
    imu = _ensure_section("imu")

    if line.startswith("IMU not connected"):
        imu["status"] = "not_connected"
        return True

    triple = _parse_triple(line)
    if triple:
        if "ACCELERATION" in line.upper():
            imu["accel_x_mps2"], imu["accel_y_mps2"], imu["accel_z_mps2"] = triple
            imu.setdefault("status", "ok")
            return True
        if "GYRO" in line.upper():
            imu["gyro_x_dps"], imu["gyro_y_dps"], imu["gyro_z_dps"] = triple
            imu.setdefault("status", "ok")
            return True
        if "MAGNETIC" in line.upper():
            imu["mag_x_uT"], imu["mag_y_uT"], imu["mag_z_uT"] = triple
            imu.setdefault("status", "ok")
            return True

    if line.startswith("Temperature:"):
        value = _extract_first_float(line)
        if value is not None:
            imu["temperature_c"] = value
        imu.setdefault("status", "ok")
        return True

    return False


def build_sensor_metadata_lines(capture_timestamp=None):
    """Convert cached telemetry into CSV comment rows."""
    lines = []
    if capture_timestamp is not None:
        lines.append(f"# CAPTURED_AT,{capture_timestamp.isoformat(timespec='seconds')}")

    for section in ("gps", "imu"):
        data = sensor_cache.get(section)
        if not data:
            continue
        section_prefix = section.upper()
        lines.append(f"# {section_prefix}_DATA_START")
        for key in sorted(data.keys()):
            value = data[key]
            if isinstance(value, float):
                value_str = f"{value:.6f}".rstrip("0").rstrip(".")
            else:
                value_str = str(value)
            lines.append(f"# {section_prefix}_{key},{value_str}")
        lines.append(f"# {section_prefix}_DATA_END")
    return lines

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
    global ThermalCaptureTimestamp
    csv_capture_mode = False
    csv_data = []
    
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore')
                if data:
                    # Print the data as-is, preserving original newlines
                    print(f"{data}", end='', flush=True)

                    # Update sensor telemetry cache with incoming line
                    update_sensor_cache(data)
                    
                    # Check for thermal data capture command time and save it for the csv.
                    if THERMAL_CAPTURE_TIMESTAMP_FLAG in data:
                        ThermalCaptureTimestamp = datetime.now()
                        continue

                    # Check for CSV start marker
                    if THERMAL_CSV_START in data:
                        print(f"\n🎯 CSV export detected! Starting data capture...")
                        csv_capture_mode = True
                        csv_data = []
                        if THERMAL_CAPTURE_TIMESTAMP_FLAG not in data:
                            ThermalCaptureTimestamp = datetime.now()
                        continue
                    
                    # Check for CSV end marker
                    if THERMAL_CSV_END in data:
                        if csv_capture_mode:
                            print(f"\n🏁 CSV export complete! Processing data...")
                            csv_capture_mode = False
                            
                            # Save the captured data
                            metadata_lines = build_sensor_metadata_lines(ThermalCaptureTimestamp)
                            if metadata_lines:
                                csv_content = '\n'.join(metadata_lines + [''] + csv_data)
                            else:
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
                        clean_data = (
                            data.replace("Teensy: ", "")
                            .replace("SAT> ", "")
                            .rstrip('\n\r')
                        )
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
