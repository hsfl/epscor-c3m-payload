# EPSCOR C3M CubeSat Payload System

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/yourusername/epscor-c3m-payload)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

**Version 1.0.0** - Production Release (October 14, 2025)

A complete thermal imaging payload system for CubeSat applications, featuring satellite-side thermal capture and ground station data reception via UHF radio link.

## Overview

The EPSCOR C3M system captures thermal images from space and transmits them to a ground station via 433MHz radio. The system consists of three main components:

1. **Satellite Payload** - Raspberry Pi + Teensy 4.1 + thermal camera for image capture and radio transmission
2. **Ground Station** - Teensy 4.1 receiver for downloading thermal data via radio
3. **(TODO) PDU Communication** - Power distribution unit protocol for satellite subsystem control

## Features

- **Thermal Imaging**: Captures 120×160 pixel thermal images using FLIR/Seek Thermal USB cameras
- **Frame Averaging**: 10-frame averaging for noise reduction
- **Robust Radio Link**: 433MHz UHF transmission using RFM23BP transceivers (1W output power)
- **Error Detection**: CRC16 validation on both per-packet and full-image levels
- **Packet Recovery**: Tracks missing/duplicate packets with automatic reporting
- **Real-time Visualization**: Automatic CSV export and matplotlib-based thermal visualization
- **Telemetry System**: 7× temperature sensors, 5× current sensors for system monitoring
- **Debug/Flight Modes**: Verbose logging for development, minimal logging for production

## System Architecture

### Data Flow

```
[Thermal Camera] → [Raspberry Pi] --UART--> [Satellite Teensy] --433MHz Radio--> [Ground Station Teensy] --USB--> [Python CLI]
```

### Communication Layers

**Satellite Side:**
1. Raspberry Pi captures thermal images from USB camera
2. Averages 10 frames for noise reduction
3. Sends data to Teensy via UART (115200 baud) with magic header `0xDE 0xAD 0xBE 0xEF`
4. Teensy packetizes thermal data (49 bytes max per radio packet)
5. Transmits via RFM23BP radio using RadioHead RH_RF22 library

**Ground Station Side:**
1. Receives radio packets via RFM23BP transceiver
2. Reassembles thermal image from packets (tracks duplicates and missing packets)
3. Validates data integrity using CRC16
4. Provides serial CLI for commands and data export
5. Auto-visualizes thermal data using Python matplotlib

## Hardware Requirements

### Satellite Payload
- **Microcontroller**: Teensy 4.1
- **SBC**: Raspberry Pi (3/4/Zero 2W)
- **Radio**: RFM23BP 433MHz transceiver (1W)
- **Camera**: USB thermal camera (FLIR/Seek Thermal compatible)
- **Sensors**:
  - 5× Adafruit INA219 current sensors (I2C addresses 0x40-0x44)
  - 7× TMP36 temperature sensors (analog)
- **Power**: 3.3V/5V/12V rails via PDU

### Ground Station
- **Microcontroller**: Teensy 4.1
- **Radio**: RFM23BP 433MHz transceiver (1W)
- **Interface**: USB connection to PC running Python CLI

### Pin Configuration

**Satellite Teensy:**
- Pin 36: RPI_ENABLE (Raspberry Pi power control)
- Pin 2: TRIGGER_PIN (signals RPi to capture image)
- Pin 13: LED_PIN (status indicator)
- Pins 7/8: UART RX/TX (Serial2)
- Pins 38/40: Radio CS/INT (SPI1 bus)

**Ground Station Teensy:**
- Pin 13: LED_PIN (status indicator)
- Pins 38/40: Radio CS/INT (SPI1 bus)

## Software Installation

### Prerequisites

- **Arduino IDE** or **PlatformIO** for Teensy development
- **Python 3.7+** for ground station CLI
- **Teensy 4.1 board support** in Arduino IDE

### Arduino/Teensy Libraries

Install via Arduino Library Manager:
- RadioHead (for RH_RF22 driver)
- Adafruit_INA219
- Wire (built-in)
- SPI (built-in)

### Python Dependencies

```bash
# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

**Requirements:**
- pyserial
- numpy
- matplotlib

## Quick Start

### 1. Upload Firmware to Teensys

**Satellite:**
```bash
# Open in Arduino IDE
# File: SatellitePayload/satellite_teensy/satellite_teensy.ino
# Board: Teensy 4.1
# USB Type: Serial
# Upload to satellite Teensy
```

**Ground Station:**
```bash
# Open in Arduino IDE
# File: dev-teensyGroundStation/ground_station_teensy/ground_station_teensy.ino
# Board: Teensy 4.1
# USB Type: Serial
# Upload to ground station Teensy
```

### 2. Run Raspberry Pi Thermal Capture

```bash
# On Raspberry Pi
cd SatellitePayload
python3 thermal_camera_controller.py
```

### 3. Run Ground Station CLI

```bash
cd dev-teensyGroundStation
python ground_station_serial_cli_teensy.py
```

### 4. Capture Thermal Data

In the ground station CLI:
```
> capture      # Start averaging frames for thermal data
> request      # Begin downlink of thermal data to ground station
> export       # Export received data as CSV (auto-visualizes)
> reset        # Software reset the flight satellite
```

## Configuration

### Radio Parameters

**Default Configuration (v1.0.0):**
- Frequency: 433.0 MHz
- Modem: GFSK_Rb38_4Fd19_6 (38.4 kbps data rate, 19.6 kHz deviation)
- TX Power: 30 dBm (1000 mW) - RFM23BP maximum
- Packet Size: 49 bytes (45 bytes data + 4 bytes overhead)

### UART Protocol

**Raspberry Pi → Satellite Teensy:**
- Baud Rate: 115200
- Frame Format:
  ```
  [0xDE 0xAD 0xBE 0xEF] [16-bit length] [payload data] [0xFF 0xFF]
  ```
- Timeouts: 15s header, 30s payload, 1s end marker

### Build Flags

Edit in `satellite_teensy.ino`:
```cpp
#define DEBUG   // Verbose logging for development
// #define FLIGHT  // Minimal logging for production (default)
```

## File Structure

```
epscor-c3m-payload/
├── SatellitePayload/
│   ├── satellite_teensy/
│   │   └── satellite_teensy.ino          # Main satellite firmware
│   └── thermal_camera_controller.py      # RPi thermal capture script
│
├── dev-teensyGroundStation/
│   ├── ground_station_teensy/
│   │   └── ground_station_teensy.ino     # Ground station firmware
│   ├── ground_station_serial_cli_teensy.py  # Python CLI
│   ├── thermal_data_viewer.py            # Thermal visualization tool
│   └── thermal_data_*.csv                # Captured thermal datasets
│
├── pdu_comm/
│   ├── pdu_protocol.h                    # PDU protocol definitions
│   └── pdu_comm.ino                      # PDU test/comm firmware
│
├── original_GroundStation/               # Legacy code (reference only)
├── CLAUDE.md                             # AI agent instructions
├── RH_RF_22_Documentation.md             # RadioHead RF22 driver docs
├── requirements.txt                      # Python dependencies
└── README.md                             # This file
```

## Usage Examples

### Basic Thermal Image Capture

1. Power up satellite payload (RPi + Teensy)
2. Start ground station CLI
3. Wait for satellite to boot and initialize
4. Send the 'capture' command, then the 'request' command
5. Export and visualize with `export` command

### Manual Thermal Data Visualization

```bash
python thermal_data_viewer.py thermal_data_001.csv
```

### Check Reception Statistics

```
> radio status
Total Packets Expected: 854
Packets Received: 850
Packets Missing: 4
CRC Errors: 0
Reception Rate: 99.5%
```

## Protocol Details

### Radio Packet Structure

**Header Packet:**
```
[Total Image Length: 4 bytes] [Total Packet Count: 2 bytes] [Header CRC: 2 bytes]
```

**Data Packets:**
```
[Packet Index: 2 bytes] [Image Data: 45 bytes] [Packet CRC: 2 bytes]
```

**End Packet:**
```
[End Marker] [Full Image CRC: 2 bytes]
```

**Serial Forwarding Packets:**
```
[Type: 0xAA] [Continuation Flag: 1 bit] [Serial Text Data]
```

### CRC Validation

- **Per-Packet CRC16**: Validates each radio packet individually
- **Full-Image CRC16**: Validates entire thermal image after reassembly
- **Mismatch Handling**: Reports corruption and allows retransmission

### Packet Tracking

Ground station maintains:
- `bool packetReceived[1200]` array for tracking received packets
- Duplicate detection (same packet index received multiple times)
- Missing packet range reporting
- CRC error counters

## Telemetry & Monitoring

### Temperature Sensors (TMP36)
- 7 sensors monitoring critical subsystems
- Analog readings on pins 14, 15, 41, 20, 21, 22, 23
- Temperature range: -40°C to +125°C

### Current Sensors (INA219)
- 5 sensors monitoring power rails
- I2C addresses: 0x40, 0x41, 0x42, 0x43, 0x44
- Voltage and current monitoring for 3.3V, 5V, 12V rails

### Serial Output Forwarding
- Satellite console output relayed to ground station
- Displayed with "SAT>" prefix in ground station CLI
- Useful for remote debugging during flight

## Troubleshooting

### Radio Communication Issues

**Problem**: No packets received at ground station

**Solutions**:
- Verify both radios are on same frequency (433.0 MHz)
- Check modem configuration matches on both ends
- Ensure antenna is properly connected
- Try `setGpioReversed(true)` if antenna switch wiring is reversed
- Check TX/RX LED indicators on both boards

**Problem**: High CRC error rate

**Solutions**:
- Reduce distance between satellite and ground station
- Check for RF interference
- Verify radio power levels
- Clear radio FIFOs via idle mode

### UART Communication Issues

**Problem**: Teensy not receiving data from Raspberry Pi

**Solutions**:
- Verify baud rate is 115200 on both ends
- Check UART wiring (TX ↔ RX must be crossed)
- Confirm magic header bytes are exact: `0xDE 0xAD 0xBE 0xEF`
- Monitor with oscilloscope to verify signal levels

### Missing Packets

**Problem**: Ground station reports missing packets

**Solutions**:
- Check radio link quality (RSSI values)
- Re-run transmission from satellite
- Enable packet retry logic (TODO: implement in firmware)
- Reduce packet transmission rate if congestion is suspected

### Thermal Data Quality

**Problem**: Noisy or poor quality thermal images

**Solutions**:
- Verify USB thermal camera connection to RPi
- Check camera driver installation
- Increase frame averaging count (currently 10 frames)
- Ensure thermal_queue is receiving frames from camera

## Development

### Debug Mode

Enable verbose logging in `satellite_teensy.ino`:
```cpp
#define DEBUG
```

This provides:
- Detailed packet transmission logs
- UART frame parsing details
- Sensor telemetry during transmission
- Error diagnostics
- Will severely slow down the downlink of thermal data

### Flight Mode

For production deployment:
```cpp
#define FLIGHT
```

This provides:
- Minimal logging for faster execution
- Telemetry skipped during payload transmission
- Optimized for reliability over verbosity

### Adding New Commands

Edit `ground_station_serial_cli_teensy.py`:
```python
def handle_custom_command(self, args):
    # Your command implementation
    pass

# Register in command_handlers dict
'custom': self.handle_custom_command
```

## (TODO) PDU Protocol

The Power Distribution Unit (PDU) controls satellite power subsystems:

### Available Commands
- `PING` - Health check
- `SET_SWITCH` - Control power switches
- `GET_SWITCH_STATUS` - Query switch states
- `SET_TORQUE` - Control reaction wheels
- `BURN_WIRE` - Deploy mechanisms

### Power Switches
- 3V3_SWITCH, 5V_SWITCH_1-4, 12V_SWITCH
- BATTERY_SWITCH
- RPI_SWITCH
- BURN_WIRE_1, BURN_WIRE_2

See [pdu_comm/pdu_protocol.h](pdu_comm/pdu_protocol.h) for complete protocol specification.

## Known Issues & TODOs

- [ ] Implement automatic packet retry logic on transmission failure
- [ ] Add RSSI reporting to ground station CLI
- [ ] Implement packet request/retransmission protocol for missing packets
- [ ] Add compression for thermal data to reduce transmission time
- [ ] Create web-based ground station interface

## Version History

### v1.0.0 (October 14, 2025) - Production Release

**Core Components:**
- Ground station firmware v1.0.0
- Ground station Python CLI v1.0.0
- Thermal data visualization tool v1.0.0
- Satellite firmware v1.0.0
- Raspberry Pi thermal capture script v1.0.0

**Key Features:**
- Stable UART communication (115200 baud)
- Reliable 433MHz radio transmission using RFM23BP
- Robust packet transmission with CRC16 validation
- Automatic CSV export and thermal visualization
- Comprehensive telemetry and sensor monitoring
- Debug and Flight modes for development/production

**Radio Configuration:**
- Frequency: 433 MHz
- Modem: GFSK_Rb38_4Fd19_6 (38.4 kbps, 19.6 kHz deviation)
- TX Power: 30dBm (1000mW)
- Packet size: 49 bytes (45 bytes data + 4 bytes overhead)

**System Specifications:**
- Thermal image size: 120×160 pixels (38,400 bytes)
- Frame averaging: 10 frames
- Packet retry: Up to 3 retries per packet (TODO)
- CRC validation: Per-packet and full-image CRC16

## Contributing

This is a research project for the EPSCOR C3M CubeSat mission. For contributions or questions, please contact the project maintainers.

### Git Workflow
- Main branch: `master` (production releases)
- Development branch: `refactor_satellite` (merged to master for v1.0.0)
- Create feature branches from `master` for new development
- Test thoroughly before merging to `master`

## License

[Specify your license here - e.g., MIT, Apache 2.0, GPL, etc.]

## Acknowledgments

- EPSCOR C3M CubeSat Team
- RadioHead Library by Mike McCauley
- Adafruit for INA219 sensor libraries

## Support

For issues, questions, or feature requests, please open an issue on the project repository or contact the development team.

---

**Project Status**: Production Ready (v1.0.0)
**Last Updated**: October 14, 2025
