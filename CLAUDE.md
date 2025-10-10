# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the EPSCOR C3M CubeSat payload system for thermal imaging and data transmission. The system consists of:

1. **Satellite Payload** - Teensy + Raspberry Pi + thermal camera that captures and transmits thermal images
2. **Ground Station** - Teensy-based receiver that downloads thermal data via radio
3. **PDU Communication** - Power distribution unit protocol for satellite subsystem control

## System Architecture

### Data Flow

```
[Thermal Camera] → [RPi] --UART--> [Satellite Teensy] --Radio--> [Ground Station Teensy] --USB--> [Python CLI]
```

**Satellite Side:**
- Raspberry Pi captures thermal images from USB camera (FLIR/Seek Thermal)
- Averages 10 frames for noise reduction
- Sends data to Teensy via UART (115200 baud) with magic header `0xDE 0xAD 0xBE 0xEF`
- Teensy packetizes thermal data (49 bytes max per radio packet)
- Transmits via RFM23BP radio (433MHz) using RadioHead RH_RF22 library

**Ground Station Side:**
- Receives radio packets via RFM23BP (same radio hardware)
- Reassembles thermal image from packets (tracks duplicates, missing packets)
- Provides serial CLI for commands and data export
- Python script auto-detects CSV export markers and visualizes thermal data

### Key Constants & Packet Structure

**Radio Configuration:**
- `RADIO_PACKET_MAX_SIZE = 49` bytes (reliable RF22 payload limit)
- Packet overhead: 2 bytes packet index + 2 bytes CRC16
- `PACKET_DATA_SIZE = 45` bytes of actual image data per packet
- Frequency: 433 MHz (configurable)
- Modem: GFSK modulation (various presets available)

**UART Protocol (RPi → Satellite Teensy):**
- Header: `[0xDE 0xAD 0xBE 0xEF] [16-bit length]`
- Payload: Raw thermal image data
- End marker: `[0xFF 0xFF]`
- Timeouts: 15s header, 30s payload, 1s end marker

**Radio Packets:**
- **Header packet:** Contains total image length, packet count, CRC
- **Data packets:** 2-byte index + 45 bytes data + 2-byte CRC16
- **End packet:** Signals transmission complete with final CRC
- **Serial packets:** Type `0xAA`, used for satellite console output forwarding

## Hardware Configuration

### Satellite Teensy (satellite_teensy.ino)
- Pin 36: RPI_ENABLE (power control for Raspberry Pi)
- Pin 2: TRIGGER_PIN (signals RPi to capture)
- Pin 13: LED_PIN
- UART: Serial2 on pins 7 (RX), 8 (TX)
- Radio: CS=38, INT=40, uses SPI1 bus (hardware_spi1)
- I2C sensors: 5× INA219 current sensors (0x40-0x44)
- Analog: 7× temperature sensors (TMP36) on pins 14,15,41,20,21,22,23

### Ground Station Teensy (ground_station_teensy.ino)
- Pin 13: LED_PIN
- Radio: CS=38, INT=40, uses SPI1 bus (hardware_spi1)
- Same radio configuration as satellite

### RadioHead Library Setup
- Uses `RH_RF22` driver for RFM22/23 transceivers
- Must use `RHHardwareSPI1.h` and `hardware_spi1` object for SPI1 bus
- See `RH_RF_22_Documentation.md` for register details and modem presets
- Antenna switch may need `setGpioReversed(true)` if wiring is reversed

## Development Commands

### Python Environment Setup
```bash
# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Ground Station CLI
```bash
cd dev-teensyGroundStation
python ground_station_serial_cli_teensy.py
```

**Available commands in Teensy CLI:**
- `receive` - Start listening for thermal data from satellite
- `export` - Export received thermal data as CSV (auto-visualizes)
- `auto` - Enable continuous reception mode
- `status` - Show reception statistics
- `clear` - Clear buffers and reset state
- `help` - Show all available commands

### Thermal Data Visualization
The Python CLI automatically detects CSV export markers (`=== START CSV ===` / `=== END CSV ===`) and:
1. Saves incrementing files: `thermal_data_001.csv`, `thermal_data_002.csv`, etc.
2. Auto-launches thermal viewer with matplotlib visualization

Manual viewer:
```bash
python thermal_data_viewer.py thermal_data_001.csv
```

### Arduino/Teensy Development
- Use Arduino IDE or PlatformIO
- Teensy 4.1 board selected
- Install libraries: RadioHead, Adafruit_INA219, Wire, SPI
- Upload via USB to Teensy
- Monitor: 115200 baud serial

**Build Flags:**
- `#define DEBUG` - Verbose logging for development (satellite_teensy.ino)
- `#define FLIGHT` - Minimal logging for production (default)

## File Organization

```
SatellitePayload/
├── satellite_teensy/
│   └── satellite_teensy.ino          # Main satellite firmware
└── thermal_camera_controller.py      # RPi thermal capture script

dev-teensyGroundStation/
├── ground_station_teensy/
│   └── ground_station_teensy.ino     # Main ground station firmware
├── ground_station_serial_cli_teensy.py  # Python CLI for ground station
├── thermal_data_viewer.py            # Matplotlib thermal visualization
└── thermal_data_*.csv                # Captured thermal datasets

pdu_comm/
├── pdu_protocol.h                    # PDU protocol definitions
└── pdu_comm.ino                      # PDU test/comm firmware

original_GroundStation/               # Legacy code (reference only)
```

## Important Implementation Details

### CRC Validation
Both satellite and ground station compute CRC16 over entire thermal image:
- Satellite computes CRC and sends in end packet
- Ground station recomputes from received data and validates
- Mismatch indicates corruption during transmission

### Packet Tracking
Ground station uses:
- `bool packetReceived[1200]` array to track which packets arrived
- Detects duplicates (radio retry logic may send same packet twice)
- Reports missing packet ranges at end of reception
- Counts CRC errors per packet: `crcErrorCount`

### Serial Output Forwarding
Satellite can forward its console output to ground station:
- Type `0xAA` packets contain serial text
- `SERIAL_CONTINUATION_FLAG` (0x80) indicates multi-chunk messages
- Ground station displays with "SAT>" prefix

### Timeouts & Retry Logic
- Satellite retries failed packet sends up to `MAX_SEND_PACKET_RETRIES = 3`
- Ground station has configurable reception timeout
- Auto-mode continuously listens for new thermal data

### Debug vs Flight Mode
- **DEBUG mode:** Satellite outputs verbose telemetry to Serial (USB)
- **FLIGHT mode:** Minimal logging, telemetry during payload send is skipped for speed
- Toggle via `#define DEBUG` / `#define FLIGHT` in satellite_teensy.ino

## PDU Protocol

Power Distribution Unit communication uses binary packet protocol:
- Commands: Ping, SetSwitch, GetSwitchStatus, SetTorque, etc.
- Switches: 3V3, 5V (multiple rails), 12V, Battery, RPI, burn wires, etc.
- Packet structure: `{type, switch_id, state, torque_value}`
- See `pdu_comm/pdu_protocol.h` for complete enum definitions

## Common Issues

### Radio Communication
- If TX stuck / RX deaf: Check if antenna switch needs `setGpioReversed(true)`
- CRC storms or timeouts: Clear FIFOs via idle mode + FFCLRTX/FFCLRRX
- Match modem preset, frequency, sync words, CRC polynomial on both ends

### UART Communication
- Ensure baud rates match (115200 for current setup)
- Check UART pin connections (TX↔RX crossed between devices)
- Magic headers must be exact byte sequence

### Missing Packets
- Radio environment quality affects packet loss
- Ground station reports missing packet ranges after reception
- Re-run transmission if critical packets are missing

### Thermal Data Quality
- RPi averages 10 frames to reduce noise
- Check USB camera connection and drivers
- Validate thermal_queue is receiving frames

## Git Workflow

Current branch: `refactor_satellite`
Main branch: `master`

Recent work focuses on:
- Radio GFSK configuration and power tuning (30dBm)
- Downlink progress visualization and timing
- Debug flag to skip telemetry during payload transmission
- Fixing data packet mask issues (thermal vs serial packet detection)

