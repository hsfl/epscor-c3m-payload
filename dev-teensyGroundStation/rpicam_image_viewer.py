#!/usr/bin/env python3
"""
RPi Camera Image Viewer for Ground Station
EPSCOR C3M Payload - Ground Station Component

Opens a downlinked rpicam JPEG in the OS default image viewer. Meant to be
launched as its own process by ground_station_serial_cli_teensy.py
(run_image_viewer()) so it never blocks the CLI's serial-read thread.

@author EPSCOR C3M Team
"""

import argparse
import os
import sys

from PIL import Image


def view_image_file(filename):
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found!")
        print("Usage: python rpicam_image_viewer.py <filename>")
        return

    try:
        print(f"Loading image from: {filename}")
        img = Image.open(filename)
        img.show(title=filename)
        print(f"Image displayed successfully! ({img.width}x{img.height})")
    except Exception as e:
        print(f"Error loading or displaying image: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='RPi Camera Image Viewer - View a downlinked rpicam JPEG'
    )
    parser.add_argument('filename', help='JPG file to view')
    args = parser.parse_args()

    view_image_file(args.filename)


if __name__ == '__main__':
    main()
