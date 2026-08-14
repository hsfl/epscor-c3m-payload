/**
 * RPi UART Communication for EPSCOR C3M Satellite Payload
 *
 * Defines the UART framing protocol used between the Raspberry Pi and the
 * satellite Teensy, and provides the functions for receiving framed
 * messages (thermal images, STATUS text, livestream frames) from the Pi.
 *
 * Frame formats:
 *  - Thermal/status frame: [UART_MAGIC 4B][length 3B] [payload] [UART_END 2B]
 *  - Livestream frame:     [STREAM_MAGIC 4B][seq 1B][size 2B] [payload] [0xFF
 * 0xFF]
 *
 * This header depends on radioPrint()/radioPrintln(), defined by the main
 * sketch, to forward error/status text to the ground station.
 */
#pragma once

#include <Arduino.h>
#include <cassert>
#include <cstdint>

// Provided by satellite_teensy.ino for radio-forwarded logging
void radioPrint(const String &message);
void radioPrintln(const String &message = "");

// UART Packet Protocol Definitions
#define UART_BAUD                                                              \
  115200 // <-- set this to match the Pi; 921600 is fine on Teensy 4.1

struct UARTFrameHeader {
  uint8_t magic[4]; // 0xDEADBEEF for still images or 0xCAFEBABE for livestream
                    // frames
  uint8_t payload_id;
  uint8_t len[3]; // little-endian length of payload (max 16MB)
};

struct UARTFrame {
  UARTFrameHeader header;
  uint8_t *payload; // pointer to payload data
  uint8_t end[2];   // 0xFFFF
};

enum class PAYLOAD_ID : uint8_t {
  STATUS_MSG = 0xF0,
  LEPTON_CAMERA = 0x00,
  BOSON_CAMERA = 0x01,
  RPI_CAMERA = 0x02
};

// Magic numbers for still image and status message frames
const uint8_t UART_MAGIC[4] = {0xDE, 0xAD, 0xBE, 0xEF};
const uint8_t UART_END[2] = {0xFF, 0xFF};

const uint8_t UART_HEADER_SIZE = 8;
static_assert(sizeof(UARTFrameHeader) == UART_HEADER_SIZE,
              "UARTFrameHeader size mismatch");

const uint32_t UART_HEADER_TIMEOUT_MS =
    15000; // 15s to see header (Pi capture + prep time)
const uint32_t UART_PAYLOAD_TIMEOUT_MS = 30000; // 30s to receive payload
const uint32_t UART_END_TIMEOUT_MS = 1000;      // 1s to see end markers

// UART commands to the Pi. CAPTURE with no ID captures every connected
// camera; REQUEST needs a camera-ID argument appended by the caller (e.g.
// "REQUEST 0\n").
const char UART_CAPTURE_CMD[] = "CAPTURE\n";
const char UART_REQUEST_CMD[] = "REQUEST";

// Livestream protocol constants
const uint8_t STREAM_MAGIC[4] = {
    0xCA, 0xFE, 0xBA, 0xBE}; // Magic header for livestream frames from Pi
const uint16_t STREAM_FRAME_SIZE = 8000; // 100x80 8-bit = 8000 bytes per frame
const char STREAM_START_CMD[] =
    "STREAM_START\n"; // Command to RPi to start streaming
const char STREAM_STOP_CMD[] =
    "STREAM_STOP\n";                        // Command to RPi to stop streaming
const char FRAME_REQUEST_CMD[] = "FRAME\n"; // Command to RPi to send one frame

const uint32_t STREAM_HEADER_TIMEOUT_MS =
    1000; // 1s timeout for streaming header/magic
const uint32_t STREAM_DATA_TIMEOUT_MS =
    2000; // 2s for frame data (8000 bytes = ~700ms at 115200)

// Read exactly 'len' bytes from 'port' with a deadline
inline bool readExact(HardwareSerial &port, uint8_t *buf, size_t len,
                      uint32_t timeout_ms) {
  uint32_t start = millis();
  size_t got = 0;
  while (got < len) {
    if (millis() - start > timeout_ms)
      return false;
    int avail = port.available();
    if (avail > 0) {
      size_t toRead = (size_t)avail;
      size_t needed = len - got;
      if (toRead > needed)
        toRead = needed;

      size_t r = port.readBytes(buf + got, toRead);
      if (r > 0) {
        got += r;
      } else {
        delay(1);
      }
    } else {
      delay(1);
    }
  }
  return true;
}

// Validate end markers
inline bool endOK(const uint8_t *e) {
  return e[0] == UART_END[0] && e[1] == UART_END[1];
}

// Identify STATUS vs IMAGE by header's payload_id field
inline bool payloadIsStatus(uint8_t payload_id) {
  return payload_id == (uint8_t)PAYLOAD_ID::STATUS_MSG;
}

// Scan UART byte-by-byte looking for UART_MAGIC (0xDEADBEEF), discarding
// anything before it. Without this, a single stray byte ahead of a frame
// (e.g. a UART line-idle glitch when the Pi's TX first comes up) desyncs
// every frame after it forever, since a plain readExact() has no way to
// tell "8 arbitrary bytes" from "a real header" once it's off by one.
inline bool waitForMagic(HardwareSerial &port, uint32_t timeout_ms) {
  uint32_t start = millis();
  uint8_t matchIndex = 0;

  while (millis() - start < timeout_ms) {
    if (port.available()) {
      uint8_t b = port.read();
      if (b == UART_MAGIC[matchIndex]) {
        matchIndex++;
        if (matchIndex == 4)
          return true;
      } else {
        matchIndex = (b == UART_MAGIC[0]) ? 1 : 0;
      }
    } else {
      delay(1);
    }
  }
  return false;
}

// Receive ONE framed message from the Pi into 'dest' (up to destMax)
// Returns: true on success; writes outLen and sets isStatus accordingly.
// headerTimeoutMs bounds both the magic scan and the header read; callers
// that only want to opportunistically drain already-buffered bytes (e.g.
// an idle-loop poller) should pass a short value instead of the default
// 15s, so a run of non-magic bytes doesn't stall the caller's loop.
inline bool recvFramedFromPi(HardwareSerial &port, uint8_t *dest,
                             uint32_t destMax, uint32_t &outLen,
                             bool &isStatus,
                             uint32_t headerTimeoutMs = UART_HEADER_TIMEOUT_MS) {
  outLen = 0;
  isStatus = false;

  // 1) Header: 4 magic + 1 payload_id + 3 length. Scan for magic first so a
  // stray leading byte can't desync every frame after it.
  uint8_t header[UART_HEADER_SIZE];
  header[0] = UART_MAGIC[0];
  header[1] = UART_MAGIC[1];
  header[2] = UART_MAGIC[2];
  header[3] = UART_MAGIC[3];
  if (!waitForMagic(port, headerTimeoutMs)) {
    radioPrintln("ERROR: UART header timeout (no magic found)");
    return false;
  }
  if (!readExact(port, header + 4, UART_HEADER_SIZE - 4, headerTimeoutMs)) {
    radioPrintln("ERROR: UART header timeout");
    return false;
  }

  uint32_t len = (uint32_t)header[5] | ((uint32_t)header[6] << 8) |
                 ((uint32_t)header[7] << 16);
  if (len == 0) {
    radioPrintln("ERROR: Zero-length payload");
    return false;
  }

  if (len > destMax) {
    radioPrint("ERROR: Payload too large (");
    radioPrint(String(len));
    radioPrintln(" bytes) for buffer");

    // DEBUG: dump the raw header bytes that produced this length
    radioPrint("DEBUG: header bytes: ");
    for (int i = 0; i < UART_HEADER_SIZE; i++) {
      radioPrint("0x");
      radioPrint(String(header[i], HEX));
      radioPrint(" ");
    }
    radioPrintln();

    // Drain and discard payload + end markers to resync
    uint8_t dump[64];
    uint32_t remaining = (uint32_t)len + 2;
    uint32_t start = millis();
    while (remaining > 0 && (millis() - start) < UART_PAYLOAD_TIMEOUT_MS) {
      size_t toRead =
          (remaining < sizeof(dump)) ? (size_t)remaining : sizeof(dump);
      size_t r = port.readBytes(dump, toRead);
      if (r > 0) {
        // DEBUG: dump drained bytes as hex and printable ASCII
        radioPrint("DEBUG: drained ");
        radioPrint(String((unsigned)r));
        radioPrint(" bytes: ");
        for (size_t i = 0; i < r; i++) {
          if (dump[i] < 0x10)
            radioPrint("0");
          radioPrint(String(dump[i], HEX));
          radioPrint(" ");
        }
        radioPrint(" | ");
        for (size_t i = 0; i < r; i++) {
          char c = (char)dump[i];
          radioPrint(String((c >= 32 && c < 127) ? c : '.'));
        }
        radioPrintln();

        remaining -= (uint32_t)r;
      } else {
        delay(1);
      }
    }
    radioPrintln("ERROR: Discarded oversized payload; request retransmit.");
    return false;
  }

  // 2) Payload
  if (!readExact(port, dest, len, UART_PAYLOAD_TIMEOUT_MS)) {
    radioPrintln("ERROR: UART payload timeout");
    return false;
  }

  // 3) End markers
  uint8_t ender[2];
  if (!readExact(port, ender, 2, UART_END_TIMEOUT_MS)) {
    radioPrintln("ERROR: UART end-marker timeout");
    return false;
  }
  if (!endOK(ender)) {
    radioPrint("ERROR: Bad end markers: 0x");
    radioPrint(String(ender[0], HEX));
    radioPrint(" 0x");
    radioPrintln(String(ender[1], HEX));
    return false;
  }

  outLen = len;
  isStatus = payloadIsStatus(header[4]);
  return true;
}

// Check if the buffer starts with stream magic header
inline bool isStreamMagic(const uint8_t *buf) {
  return buf[0] == STREAM_MAGIC[0] && buf[1] == STREAM_MAGIC[1] &&
         buf[2] == STREAM_MAGIC[2] && buf[3] == STREAM_MAGIC[3];
}

// Scan UART byte-by-byte looking for STREAM_MAGIC header
// This allows recovery from partial data or misalignment
inline bool waitForStreamMagic(HardwareSerial &port, uint32_t timeout_ms) {
  uint32_t start = millis();
  uint8_t matchIndex = 0;

  while (millis() - start < timeout_ms) {
    if (port.available()) {
      uint8_t b = port.read();
      if (b == STREAM_MAGIC[matchIndex]) {
        matchIndex++;
        if (matchIndex == 4) {
          return true; // Found complete magic header
        }
      } else {
        // Mismatch - check if this byte could be start of new magic
        matchIndex = (b == STREAM_MAGIC[0]) ? 1 : 0;
      }
    } else {
      delay(1);
    }
  }
  return false; // Timeout
}

/**
 * Receive one stream frame from Pi via UART
 * Stream frame format: [STREAM_MAGIC 4B][Frame Seq 1B][Size 2B][Data
 * frameBufSize B][End 2B] Uses byte-by-byte scanning to find magic header
 * (handles misalignment)
 *
 * @param port Serial port connected to the Pi
 * @param frameBuf Destination buffer for the frame data (must be >=
 * frameBufSize)
 * @param frameBufSize Expected frame data size in bytes (e.g.
 * STREAM_FRAME_SIZE)
 * @param frameSeq Output parameter for frame sequence number
 * @return true if frame received successfully
 */
inline bool recvStreamFrameFromPi(HardwareSerial &port, uint8_t *frameBuf,
                                  uint16_t frameBufSize, uint8_t &frameSeq) {
  // Don't even try if no data available
  if (!port.available()) {
    return false;
  }

  // Scan for stream magic byte-by-byte
  if (!waitForStreamMagic(port, STREAM_HEADER_TIMEOUT_MS)) {
    return false; // Timeout or no magic found
  }

  // Magic found, now read rest of header: 1 seq + 2 size = 3 bytes
  uint8_t headerRest[3];
  if (!readExact(port, headerRest, 3, STREAM_HEADER_TIMEOUT_MS)) {
    radioPrintln("STREAM: Header rest timeout");
    return false;
  }

  frameSeq = headerRest[0];
  uint16_t frameSize = (uint16_t)headerRest[1] | ((uint16_t)headerRest[2] << 8);

#ifdef DEBUG
  radioPrint("STREAM: seq=");
  radioPrint(String(frameSeq));
  radioPrint(" size=");
  radioPrintln(String(frameSize));
#endif

  if (frameSize != frameBufSize) {
    radioPrint("STREAM: Bad frame size ");
    radioPrint(String(frameSize));
    radioPrint(" (expected ");
    radioPrint(String(frameBufSize));
    radioPrint(", got bytes 0x");
    radioPrint(String(headerRest[1], HEX));
    radioPrint(" 0x");
    radioPrint(String(headerRest[2], HEX));
    radioPrintln(")");
    // Drain remaining data to resync
    while (port.available())
      port.read();
    return false;
  }

  // Read frame data
  if (!readExact(port, frameBuf, frameBufSize, STREAM_DATA_TIMEOUT_MS)) {
    radioPrintln("STREAM: Frame data timeout");
    // Drain buffer to resync
    while (port.available())
      port.read();
    return false;
  }

  // Read end markers
  uint8_t ender[2];
  if (!readExact(port, ender, 2, 500)) {
    radioPrintln("STREAM: End marker timeout");
    // Drain buffer to resync
    while (port.available())
      port.read();
    return false;
  }

  if (ender[0] != 0xFF || ender[1] != 0xFF) {
    radioPrint("STREAM: Bad end markers 0x");
    radioPrint(String(ender[0], HEX));
    radioPrint(" 0x");
    radioPrintln(String(ender[1], HEX));
    // Drain buffer to resync for next frame
    while (port.available())
      port.read();
    return false;
  }

  return true;
}
