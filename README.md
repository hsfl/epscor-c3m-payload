# EPSCOR C3M CubeSat Payload System

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/yourusername/epscor-c3m-payload)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

**Version 2.0.0** - Livestream Release (December 8, 2025)

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

The thermal data viewer allows you to visualize any saved thermal CSV file:

```bash
# Navigate to ground station directory
cd dev-teensyGroundStation

# View a specific thermal data file
python thermal_data_viewer.py thermal_data_001.csv

# Or use any other numbered CSV file
python thermal_data_viewer.py thermal_data_042.csv
```

**How it works:**
- The viewer reads CSV files containing 120×160 pixel thermal data
- Displays a color-mapped visualization using matplotlib
- Shows temperature distribution across the thermal image
- Files are auto-numbered incrementally (001, 002, 003, etc.)

**Note:** The `export` command in the ground station CLI automatically runs the viewer, but you can use this for reviewing previously captured data.

### Thermal Livestream Mode

**NEW in v1.1:** Real-time thermal video streaming from satellite to ground station!

The livestream feature enables continuous, low-latency thermal video transmission for real-time monitoring and situational awareness. This mode prioritizes speed over data archival - frames are optimized for fast transmission and immediate display.

#### How Livestream Works

**Architecture Overview:**
```
[Thermal Camera] → [RPi] → [Satellite Teensy] → [Radio] → [GS Teensy] → [Python CLI] → [Live Viewer]
     (USB)         (UART)    (Request/Response)  (433MHz)    (USB Serial)   (Queue)    (matplotlib)
```

**Data Flow:**

1. **Ground Station Initiates:** User types `stream start` in the ground station CLI
2. **Command Forwarding:** Ground station Teensy sends radio command `v1` to satellite
3. **Satellite Activation:** Satellite Teensy sends `STREAM_START\n` via UART to Raspberry Pi
4. **Request-Response Loop:**
   - Satellite Teensy requests frame: sends `FRAME\n` to RPi via UART
   - RPi captures latest thermal frame (160×120, 16-bit)
   - RPi downsamples to 80×60, 8-bit for faster transmission
   - RPi sends frame to Teensy via UART with magic header `0xCA 0xFE 0xBA 0xBE`
   - Teensy packetizes frame into ~107 radio packets (45 bytes data each)
   - Teensy transmits frame over 433MHz radio to ground station
   - Ground station reassembles packets into complete frame
   - Ground station forwards frame to Python CLI via USB serial
   - Python CLI queues frame for visualization
   - Matplotlib viewer displays frame in real-time (~5-10 FPS)
   - **Teensy requests next frame** (loop continues)

5. **Termination:** User types `stream stop` - command propagates back to stop the loop

#### Frame Optimization for Streaming

**Downsampling Strategy:**
- Original: 160×120 pixels @ 16-bit (38,400 bytes) → takes ~45 seconds to transmit
- Downsampled: 80×60 pixels @ 8-bit (4,800 bytes) → takes ~5.6 seconds to transmit
- Method: 2×2 block averaging (preserves thermal patterns, reduces noise)
- Scaling: 16-bit Kelvin×100 mapped to 8-bit (0°C=0, 100°C=255)

**Why Downsampling:**
- **8× faster transmission** (4,800 vs 38,400 bytes)
- **Better frame rate** (~5-10 FPS vs <1 FPS for full resolution)
- **Reduced radio congestion** (107 packets vs 854 packets per frame)
- **Sufficient resolution** for real-time monitoring and situational awareness

**Best-Effort Delivery:**
- No packet retry logic (speed over reliability)
- Ground station accepts partial frames (50%+ packets = display)
- Dropped packets appear as visual artifacts but don't stall the stream

#### Using Livestream Mode

**Start Livestream:**
```bash
cd dev-teensyGroundStation
python ground_station_serial_cli_teensy.py

GS> stream start
# Livestream begins, frames arrive continuously
# Frame rate: ~5-10 FPS depending on radio conditions
```

**View Livestream (Automatic):**
The Python CLI automatically detects stream mode and can launch the viewer:
```bash
# In a separate terminal while streaming is active:
python thermal_data_viewer.py --stream

# Or start CLI with auto-viewer:
python ground_station_serial_cli_teensy.py --auto-stream-viewer
```

**Stop Livestream:**
```
GS> stream stop
# Stream terminates gracefully
# Satellite returns to normal command mode
```

#### Livestream Protocol Details

**UART Frame Structure (RPi → Satellite Teensy):**
```
[Magic: 0xCA 0xFE 0xBA 0xBE] [Frame Seq: 1 byte] [Size: 2 bytes] [Data: 4800 bytes] [End: 0xFF 0xFF]
Total: 4809 bytes per frame
```

**Radio Packet Structure (Teensy → Ground Station):**

**Header Packet (6 bytes):**
```
[Type: 0xCC] [Frame Seq: 1 byte] [Frame Size: 2 bytes] [Total Packets: 2 bytes]
```

**Data Packets (48 bytes):**
```
[Type: 0xCC] [Frame Seq: 1 byte] [Packet Index: 1 byte] [Data: 45 bytes]
```

**Stream Statistics:**
- Frame size: 4,800 bytes (80×60 @ 8-bit)
- Packets per frame: 107 (4,800 ÷ 45 = 106.67, rounded up)
- Radio time per frame: ~5.6 seconds @ 125 kbps (GFSK_Rb125Fd125)
- Effective frame rate: ~10 FPS (ideal), ~5-8 FPS (typical with radio overhead)

#### Flow Control Mechanism

**Why Request-Response?**
The satellite Teensy uses explicit flow control to prevent serial buffer overflow:

1. **Problem:** If RPi continuously streams frames, UART buffer fills up → data loss
2. **Solution:** Teensy only requests next frame after transmitting current one
3. **Benefit:** Zero buffer overflow, predictable frame pacing

**Request-Response Sequence:**
```
Teensy: "FRAME\n"          → RPi receives request
RPi:    [Frame data 4809B] → Teensy receives frame
Teensy: [Transmit via radio ~5.6s]
Teensy: "FRAME\n"          → Request next frame
(cycle repeats)
```

#### Livestream Performance

**Factors Affecting Frame Rate:**
- Radio link quality (RSSI, packet loss)
- Modem configuration (faster = higher FPS, shorter range)
- Camera FFC events (auto-calibration pauses streaming briefly)
- Serial buffer processing speed

**Optimizations:**
- Latest frame buffer (no queue flushing - always fresh frame)
- Best-effort delivery (no retries during streaming)
- Efficient 8-bit encoding
- Request-response flow control prevents buffer overflow

**Typical Performance:**
```
Radio Config          Frame Rate    Transmission Time
GFSK_Rb125Fd125       ~8-10 FPS     ~5.6s per frame
GFSK_Rb57_6Fd28_8     ~4-6 FPS      ~12s per frame
GFSK_Rb38_4Fd19_6     ~2-4 FPS      ~18s per frame
```

#### Livestream vs. Single Capture

| Feature                | Livestream Mode       | Single Capture Mode      |
|------------------------|-----------------------|--------------------------|
| **Resolution**         | 80×60 @ 8-bit         | 160×120 @ 16-bit         |
| **Data Size**          | 4,800 bytes           | 38,400 bytes             |
| **Transmission Time**  | ~5.6s per frame       | ~45s per image           |
| **Frame Rate**         | ~5-10 FPS             | <1 FPS                   |
| **Frame Averaging**    | No (latest frame)     | Yes (10 frames avg)      |
| **Error Correction**   | Best-effort           | CRC16 + packet retry     |
| **Use Case**           | Real-time monitoring  | High-quality archival    |
| **Data Archival**      | No (display only)     | Yes (CSV export)         |

#### When to Use Livestream

**Best For:**
- Real-time situational awareness
- Quick target scanning
- Live thermal monitoring during operations
- Verifying camera is working and pointed correctly
- Demonstrating system capabilities

**Not Ideal For:**
- Scientific data collection (use single capture instead)
- Archival imagery (no save function - display only)
- Long-range operations (higher packet loss at distance)

#### Troubleshooting Livestream

**Issue:** Low frame rate or stuttering
- **Solution:** Check radio link quality, reduce distance, or switch to faster modem config

**Issue:** Visual artifacts (black patches in image)
- **Cause:** Dropped radio packets (normal for best-effort streaming)
- **Solution:** Acceptable if <20% packet loss; improve radio link if severe

**Issue:** Stream won't start
- **Check:**
  1. Satellite RPI status: should show `RPI STATUS: IDLE` before streaming
  2. Radio link active: try `ping` command first
  3. Camera operational: test with single `capture` command first

**Issue:** Stream won't stop
- **Solution:** Send `stream stop` multiple times (5× with delays) to ensure delivery

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
