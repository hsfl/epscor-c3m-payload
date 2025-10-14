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
        data = np.loadtxt(filename, delimiter=',')
        
        # Display as thermal image
        plt.figure(figsize=(12, 8))
        plt.imshow(data, cmap='hot', aspect='auto')
        plt.colorbar(label='Temperature (°C)')
        plt.title(f'Thermal Image from Flat Sat - {filename}')
        plt.xlabel('Column')
        plt.ylabel('Row')
        
        # Add some statistics
        min_temp = np.nanmin(data)
        max_temp = np.nanmax(data)
        mean_temp = np.nanmean(data)
        
        plt.figtext(0.02, 0.02, f'Min: {min_temp:.1f}°C, Max: {max_temp:.1f}°C, Mean: {mean_temp:.1f}°C', 
                   fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        print(f"Thermal image displayed successfully!")
        print(f"Temperature range: {min_temp:.1f}°C to {max_temp:.1f}°C")
        print(f"Mean temperature: {mean_temp:.1f}°C")
        
    except Exception as e:
        print(f"Error loading or displaying thermal data: {e}")
        print("Make sure the file contains valid CSV data with temperature values.")

if __name__ == "__main__":
    main()
