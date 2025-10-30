"""
Thermal Data Viewer
EPSCOR C3M Payload - Ground Station Component

Displays thermal image data from CSV files captured by the ground station
Usage: python thermal_data_viewer.py [filename]
If no filename is provided, defaults to thermal_data_001.csv

@author EPSCOR C3M Team
@date 2025-10-14
@version 1.0.0
"""

__version__ = "1.0.0"
__build_date__ = "2025-10-14"

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import webbrowser
from io import StringIO
from matplotlib.offsetbox import TextArea, AnnotationBbox
from matplotlib.widgets import Button

def _parse_ddmm_mmmm(coord):
    """Return DMS text and decimal degrees from DDMM.MMMM coordinate strings."""
    coord = coord.strip()
    if not coord:
        return None

    direction = ""
    if coord[-1] in "NSEW":
        direction = coord[-1]
        coord = coord[:-1]

    try:
        numeric = float(coord)
    except ValueError:
        return None

    degrees = int(numeric // 100)
    minutes_full = numeric - (degrees * 100)
    minutes = int(minutes_full)
    seconds = (minutes_full - minutes) * 60

    decimal = degrees + minutes_full / 60
    if direction in ("S", "W"):
        decimal *= -1

    dms_text = f"{degrees}°{minutes}'{seconds:.2f}\"{direction}"
    return dms_text, decimal


def parse_gps_location_ddmm(raw_value):
    """Return DMS string and list of decimal degree floats from DDMM.MMMM pair."""
    parts = [part.strip() for part in raw_value.split(",")]
    dms_parts = []
    decimal_values = []

    for part in parts:
        parsed = _parse_ddmm_mmmm(part)
        if parsed is None:
            return None, None
        dms_text, decimal = parsed
        dms_parts.append(dms_text)
        decimal_values.append(decimal)

    return ", ".join(dms_parts), decimal_values


def format_gps_location_ddmm_to_dms(raw_value):
    """Convert comma-separated DDMM.MMMM location string to DMS for display."""
    dms_text, _ = parse_gps_location_ddmm(raw_value)
    return dms_text


def parse_decimal_coordinate_pair(raw_value):
    """Return list of decimal degree floats parsed from comma-separated string."""
    if not raw_value:
        return None

    decimal_values = []
    for part in raw_value.split(","):
        stripped = part.strip()
        if not stripped:
            continue
        try:
            decimal_values.append(float(stripped))
        except ValueError:
            return None

    if len(decimal_values) < 2:
        return None
    return decimal_values


def load_thermal_file(filename):
    """
    Load thermal CSV data and associated metadata comments.

    Returns:
        tuple[np.ndarray, dict]: thermal grid, metadata dict with keys
        'captured_at', 'gps', and 'imu'.
    """
    metadata = {
        "captured_at": None,
        "gps": {},
        "imu": {},
    }

    data_lines = []
    current_section = None

    with open(filename, "r") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                # Preserve blank line between metadata and data but do not append metadata blanks
                continue

            if line.startswith("#"):
                content = line.lstrip("#").strip()
                upper_content = content.upper()

                if upper_content.startswith("CAPTURED_AT,"):
                    _, value = content.split(",", 1)
                    metadata["captured_at"] = value.strip()
                elif upper_content.endswith("_DATA_START"):
                    current_section = content.split("_", 1)[0].lower()
                elif upper_content.endswith("_DATA_END"):
                    current_section = None
                elif "," in content and current_section in ("gps", "imu"):
                    key, value = content.split(",", 1)
                    # drop leading section prefix if present (e.g. GPS_latitude_deg)
                    key = key.strip()
                    prefix = f"{current_section.upper()}_"
                    if key.upper().startswith(prefix):
                        key = key[len(prefix):]
                    metadata[current_section][key] = value.strip()
                # Ignore other comment styles
                continue

            data_lines.append(raw_line)

    if not data_lines:
        raise ValueError("No thermal data rows found in file")

    data_buffer = StringIO("".join(data_lines))
    data = np.loadtxt(data_buffer, delimiter=",")
    return data, metadata

def main():
    # Get filename from command line argument or use default
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        print("Using filename: " + filename) 
    else:
        filename = 'thermal_data_001.csv'
        print("Using default filename: " + filename) 
    
    # Check if file exists
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found!")
        print("Usage: python thermal_data_viewer.py [filename]")
        return
    
    try:
        # Load the thermal data
        print(f"Loading thermal data from: {filename}")
        data, metadata = load_thermal_file(filename)
        
        # Display as thermal image
        fig, ax = plt.subplots(figsize=(12, 8))
        image = ax.imshow(data, cmap='hot', aspect='auto')
        fig.colorbar(image, ax=ax, label='Temperature (°C)')
        
        # Add some statistics
        min_temp = np.nanmin(data)
        max_temp = np.nanmax(data)
        mean_temp = np.nanmean(data)
        
        # Build title and subtitle
        main_title = f'Thermal Image from Flat Sat - {filename}'
        
        subtitle_parts = [f'Min: {min_temp:.1f}°C, Max: {max_temp:.1f}°C, Mean: {mean_temp:.1f}°C']
        
        if metadata.get("captured_at"):
            subtitle_parts.append(f'Captured: {metadata["captured_at"]}')

        gps_location_display = None
        gps_location_decimal = None
        maps_link = None

        gps_decimal_raw = metadata["gps"].get("gs_sat_decimal")
        gps_location_decimal = parse_decimal_coordinate_pair(gps_decimal_raw)

        gps_location_raw = (
            metadata["gps"].get("gs_sat_location")
            or metadata["gps"].get("gs_sat_location_ddmm_mmmm_format")
        )

        if gps_location_raw:
            ddmm_display, ddmm_decimal = parse_gps_location_ddmm(gps_location_raw)
            if ddmm_display:
                gps_location_display = ddmm_display
            if not gps_location_decimal and ddmm_decimal:
                gps_location_decimal = ddmm_decimal

        if not gps_location_display and gps_location_decimal:
            gps_location_display = (
                f"{gps_location_decimal[0]:.6f}°, {gps_location_decimal[1]:.6f}°"
            )

        if gps_location_display:
            subtitle_parts.append(f"GPS: {gps_location_display}")

        if gps_location_decimal and len(gps_location_decimal) >= 2:
            maps_link = (
                f"https://maps.google.com/maps?q="
                f"{gps_location_decimal[0]:.6f},{gps_location_decimal[1]:.6f}"
            )
        
        subtitle = '\n'.join(subtitle_parts)
        
        ax.set_title(main_title + '\n' + subtitle, fontsize=10, pad=10)
        ax.set_xlabel('Column')
        ax.set_ylabel('Row')
        
        if maps_link:
            fig.subplots_adjust(top=0.80, bottom=0.1)
            button_ax = fig.add_axes([0.35, 0.92, 0.20, 0.05])
            maps_button = Button(button_ax, 'Open in Google Maps', color='lightblue', hovercolor='skyblue')

            def on_button_click(_event):
                webbrowser.open(maps_link)

            maps_button.on_clicked(on_button_click)
        else:
            fig.tight_layout()
        
        plt.show()
        
        print(f"Thermal image displayed successfully!")
        print(f"Temperature range: {min_temp:.1f}°C to {max_temp:.1f}°C")
        print(f"Mean temperature: {mean_temp:.1f}°C")
        if metadata.get("captured_at"):
            print(f"Captured at (UTC): {metadata['captured_at']}")
        if gps_location_display:
            print(f"GPS location: {gps_location_display}")
            if maps_link:
                print(f"Google Maps: {maps_link}")
        elif gps_location_raw:
            print(f"GPS location unparsed: {gps_location_raw}")
        
        print("GS> ")
    except Exception as e:
        print(f"Error loading or displaying thermal data: {e}")
        print("Make sure the file contains valid CSV data with temperature values.")

if __name__ == "__main__":
    main()
