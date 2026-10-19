/**
 * Outreach Satellite Teensy - No-RF Edition for EPSCOR C3M Project
 *
 * A copy of SatellitePayload/satellite_teensy/satellite_teensy.ino for
 * outreach events, flashed onto the satellite Teensy and plugged straight
 * into the laptop over USB - no radio, no ground station Teensy.
 *
 * The RPi side is unchanged: the Teensy powers the Pi, waits for
 * "RPI STATUS: IDLE", sends CAPTURE / REQUEST <id> over UART and receives the
 * framed image exactly as the flight firmware does.
 *
 * The radio link is replaced by a loopback: the satellite still packetizes
 * the image into header / data / end packets with CRC16s, but
 * sendPacketReliable() hands each packet to a copy of the ground station's
 * receive path (processPacket() and friends from ground_station_teensy.ino)
 * instead of the RF23. That receive path reassembles and CRC-checks the
 * image and prints the same text the ground station Teensy prints - header
 * received, 25/50/75/100% progress, reception summary, and the
 * "--- EXPORTING ... ---" + "=== START CSV/JPG ===" export - so
 * ground_station_serial_cli_teensy.py and ground_station_gui_demo/gds_app.py
 * parse it with no changes.
 *
 * Commands are the ground station CLI's (capture, request <id>, export, ...),
 * since that is what the CLI / GUI buttons send. Satellite-side console
 * output keeps its "SAT> " prefix, as it would after radio forwarding.
 *
 * Not carried over from satellite_teensy.ino (radio-only): livestream mode,
 * missing-packet retry requests, RF23 FIFO dump, radio init/config.
 *
 * rpi_uart.hpp is a copy of SatellitePayload/satellite_teensy/rpi_uart.hpp -
 * Arduino can't include headers from outside the sketch folder. Re-copy it
 * if the UART framing changes there.
 *
 * Hardware Requirements:
 * - Teensy 4.1 microcontroller (satellite board)
 * - Raspberry Pi with thermal camera(s)
 * - UART connection between Teensy and RPI (Serial2, pins 7/8)
 *
 * @author EPSCOR C3M Team
 * @date 2026-10-19
 * @version 1.0.0
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include "rpi_uart.hpp"

// Packed struct attribute for ensuring no padding bytes
#define PACKED __attribute__((packed))

// Pin definitions for hardware control
const uint8_t RPI_ENABLE = 36; // Power control pin for Raspberry Pi
const uint8_t LED_PIN = 13;    // Solid while powered; blinks during capture / downlink

// Capture and downlink block for seconds inside the UART waits in
// rpi_uart.hpp, so the blink is driven by a hardware timer interrupt rather
// than from loop().
IntervalTimer ledBlinkTimer;
const uint32_t LED_BLINK_HALF_PERIOD_US = 150000; // 150 ms on / 150 ms off

void toggleLed()
{
  digitalToggleFast(LED_PIN);
}

void startLedBlink()
{
  ledBlinkTimer.begin(toggleLed, LED_BLINK_HALF_PERIOD_US);
}

void stopLedBlink()
{
  ledBlinkTimer.end();
  digitalWrite(LED_PIN, HIGH); // back to solid "powered on"
}

// Simple max for STATUS messages
const uint16_t MAX_STATUS_LEN = 256;

uint8_t piStatusBuf[MAX_STATUS_LEN];
bool piCaptureInProgress = false;
bool RPI_IDLE_READY = false;

// UART command to delete every queued capture on the Pi. Must match CMD_CLEAR
// in multicam_controller.py. Kept here rather than in rpi_uart.hpp so that
// file stays an exact copy of the flight one.
const char UART_CLEAR_CMD[] = "CLEAR\n";

// Camera names from the Pi's boot-time "CAM_READY:<name>,<name>" status
// ("" = not reported since this Teensy booted / Pi powered on).
String piConnectedCameras = "";

// Image data storage and tracking (satellite side)
uint32_t capturedImageLength = 0;
const uint32_t MAX_IMG = 165000; // Maximum image buffer size
uint8_t imgBuf[MAX_IMG];         // Image as received from the RPi

// Image as reassembled from packets (ground station side). In RAM2
// (DMAMEM) since RAM1 already holds imgBuf; zeroed on each header packet.
DMAMEM uint8_t rxImgBuf[MAX_IMG];

// Tracks when to prepend the SAT> prefix to satellite console output
bool satLineAtStart = true;

// Packet parameters - same as the flight radio link
const uint8_t RADIO_PACKET_MAX_SIZE = 49;
const uint8_t THERMAL_PACKET_OVERHEAD = 2 /*packet index*/ + 2 /*CRC16*/;
const uint8_t PACKET_DATA_SIZE = RADIO_PACKET_MAX_SIZE - THERMAL_PACKET_OVERHEAD; // 45 bytes of image data per packet
const uint16_t MAX_PACKETS = (MAX_IMG + PACKET_DATA_SIZE - 1) / PACKET_DATA_SIZE;

// Must match ground_station_serial_cli_teensy.py exactly.
const String THERMAL_CAPTURE_TIMESTAMP_FLAG = "--- UART THERMAL CAPTURE ---";
const String THERMAL_CSV_START = "=== START CSV ===";
const String THERMAL_CSV_END = "=== END CSV ===";
const String THERMAL_JPG_START = "=== START JPG ===";
const String THERMAL_JPG_END = "=== END JPG ===";

// === Packet Structure Definitions (identical to the radio link) ===

/** Header packet: total image size and packet count. 12 bytes. */
struct PACKED ThermalHeaderPacket
{
  uint8_t marker1;       // 0xFF
  uint8_t marker2;       // 0xFF
  uint32_t imageLength;  // Total image size in bytes
  uint16_t totalPackets; // Number of data packets to follow
  uint16_t magic[2];     // {0xDEAD, 0xBEEF}
};

/**
 * Data packet: 2-byte index + up to 45 data bytes + CRC16.
 * Note: For variable-length packets, CRC must be accessed via buffer offset, not struct member
 */
struct PACKED ThermalDataPacket
{
  uint16_t packetIndex;           // Packet sequence number
  uint8_t data[PACKET_DATA_SIZE]; // Image data chunk (45 bytes max)
  uint16_t crc16;                 // CRC16 of the data field only
};

/** End packet: packet count and whole-image CRC16. 6 bytes. */
struct PACKED ThermalEndPacket
{
  uint8_t marker1;      // 0xEE
  uint8_t marker2;      // 0xEE
  uint16_t packetCount; // Total packets sent
  uint16_t imageCrc;    // CRC16 of entire image
};

uint8_t txBuffer[RADIO_PACKET_MAX_SIZE];

// Ground station reception state (mirrors ground_station_teensy.ino)
bool headerReceived = false;
bool imageComplete = false;
bool crcVerified = false;
uint32_t expectedLength = 0;
uint16_t expectedPackets = 0;
uint16_t receivedPackets = 0;
uint16_t crcErrorCount = 0;
uint16_t expectedImageCrc = 0;
uint16_t lastComputedImageCrc = 0;
uint8_t lastReportedQuarter = 0;
bool packetReceived[MAX_PACKETS];
uint8_t lastRequestedCameraId = 0xFF; // Camera id from the most recent 'request <id>' (0xFF = unknown/legacy)
bool autoExportOnComplete = false;    // Set by 'request <id>', consumed by showReceptionSummary()
unsigned long downloadStartMs = 0;

// GPS/IMU hardware
#define GPSSerial Serial7
Adafruit_GPS GPS(&GPSSerial);
bool gpsConnected = false;

Adafruit_LIS3MDL lis = Adafruit_LIS3MDL();
Adafruit_LSM6DSOX lsm6dsox = Adafruit_LSM6DSOX();
bool imuConnected = false;

// Forward declarations
void initGPS(bool announce = true);
void initIMU(bool announce = true);
void serviceGPS();
void printGPSData();
void printIMUData();
void handleSensorCommand(uint8_t *buf, uint8_t len);
bool sendPacketReliable(uint8_t *data, uint8_t len);
void sendThermalData();
void processPacket(uint8_t *buf, uint8_t len);
void showReceptionSummary();
void exportThermalData();
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/**
 * Satellite console output. On the flight build this is chunked into 0xAA
 * radio packets and the ground station prints it with a "SAT> " prefix; here
 * it goes straight to USB serial with the same prefix.
 *
 * Named radioPrint/radioPrintln because rpi_uart.hpp calls them.
 */
void radioPrint(const String &message)
{
  String out;
  out.reserve(message.length() + 8);
  for (unsigned int i = 0; i < message.length(); i++)
  {
    char c = message.charAt(i);
    if (satLineAtStart)
    {
      out += "SAT> ";
      satLineAtStart = false;
    }
    out += c;
    if (c == '\n')
      satLineAtStart = true;
  }
  Serial.print(out);
}

void radioPrintln(const String &message)
{
  radioPrint(message + "\n");
}

void printHelp()
{
  Serial.println("Commands (outreach, no RF - same names as the ground station CLI):");
  Serial.println("  capture          - Trigger RPi capture (all cameras)");
  Serial.println("  request <id>     - Pull camera <id>'s latest image and export it (0=lepton, 1=boson, 2=rpicam)");
  Serial.println("  export           - Re-export the last received image");
  Serial.println("  clear_storage    - Delete ALL captured images stored on the RPi (asks to confirm)");
  Serial.println("  rpi <on|off|status> - RPi power control");
  Serial.println("  sensor <gps|imu|both|gps_init|imu_init> - GPS/IMU data");
  Serial.println("  ping             - Ping the satellite");
  Serial.println("  status           - Show RPi power, connected cameras, last image");
  Serial.println("  reset            - Software reset");
  Serial.println("  help             - Show this list");
}

/**
 * Setup function - initializes the outreach satellite payload
 * Configures USB serial, the RPi UART and power pin, and prints the commands.
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
    ;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Solid on = Teensy powered and running

  radioPrintln("=== Artemis Cubesat Teensy 4.1 Satellite (Outreach, no RF) ===");

  Wire.begin();

  // initGPS();

  // initIMU();

  // init the RPI for thermal images
  initRPI();

  printHelp();
  radioPrintln("Ready!");
}

static String twoDigit(uint8_t value)
{
  if (value < 10)
  {
    return "0" + String(value);
  }
  return String(value);
}

void initGPS(bool announce)
{
  if (announce)
  {
    radioPrintln("Initializing GPS...");
  }

  gpsConnected = false;

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);

  if (GPS.available() == 0)
  {
    if (announce)
    {
      radioPrintln("GPS module not detected");
    }
    gpsConnected = false;
  }
  else
  {
    if (announce)
    {
      radioPrintln("GPS module connected");
    }
    gpsConnected = true;
  }

  while (GPS.available())
  {
    GPS.read();
  }

  if (announce)
  {
    if (gpsConnected)
    {
      radioPrintln("GPS initialization complete");
    }
    else
    {
      radioPrintln("GPS initialization failed");
    }
  }
}

void initIMU(bool announce)
{
  if (announce)
  {
    radioPrintln("Initializing IMU...");
  }

  bool lisReady = lis.begin_I2C();
  bool lsmReady = lsm6dsox.begin_I2C();

  imuConnected = lisReady && lsmReady;

  if (!imuConnected)
  {
    if (announce)
    {
      if (!lisReady)
      {
        radioPrintln("LIS3MDL magnetometer not detected");
      }
      if (!lsmReady)
      {
        radioPrintln("LSM6DSOX accelerometer/gyro not detected");
      }
    }
    return;
  }

  if (announce)
  {
    radioPrintln("IMU connected");
  }
}

void serviceGPS()
{
  char c = GPS.read();
  if (c)
  {
    gpsConnected = true;
  }

  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
  }
}

void printGPSData()
{
  radioPrintln("--- GPS DATA ---");

  if (!gpsConnected)
  {
    radioPrintln("GPS not connected");
    return;
  }

  if (!GPS.fix)
  {
    radioPrintln("No GPS fix - searching for satellites...");
    radioPrint("Satellites visible: ");
    radioPrintln(String((int)GPS.satellites));
    return;
  }

  radioPrint("Time (UTC): ");
  radioPrintln(twoDigit(GPS.hour) + ":" + twoDigit(GPS.minute) + ":" + twoDigit(GPS.seconds));

  radioPrint("Date: ");
  radioPrintln(String(GPS.day) + "/" + String(GPS.month) + "/20" + String(GPS.year));

  radioPrint("Location (DDMM.MMMM format): ");
  radioPrint(String(GPS.latitude, 4));
  radioPrint(String(GPS.lat));
  radioPrint(", ");
  radioPrint(String(GPS.longitude, 4));
  radioPrintln(String(GPS.lon));

  float lat = GPS.latitude / 100.0;
  int lat_deg = (int)lat;
  float lat_min = (lat - lat_deg) * 100.0;
  float lat_decimal = lat_deg + (lat_min / 60.0);
  if (GPS.lat == 'S')
  {
    lat_decimal = -lat_decimal;
  }

  float lon = GPS.longitude / 100.0;
  int lon_deg = (int)lon;
  float lon_min = (lon - lon_deg) * 100.0;
  float lon_decimal = lon_deg + (lon_min / 60.0);
  if (GPS.lon == 'W')
  {
    lon_decimal = -lon_decimal;
  }

  radioPrint("Decimal: ");
  radioPrint(String(lat_decimal, 6));
  radioPrint(", ");
  radioPrintln(String(lon_decimal, 6));

  radioPrint("Altitude: ");
  radioPrint(String(GPS.altitude));
  radioPrintln(" m");

  radioPrint("Speed: ");
  radioPrint(String(GPS.speed));
  radioPrintln(" knots");

  radioPrint("Satellites: ");
  radioPrintln(String((int)GPS.satellites));

  radioPrint("Fix quality: ");
  radioPrintln(String((int)GPS.fixquality));
}

void printIMUData()
{
  radioPrintln("--- IMU DATA ---");

  if (!imuConnected)
  {
    radioPrintln("IMU not connected");
    return;
  }

  sensors_event_t accel, gyro, temp, mag;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  lis.getEvent(&mag);

  radioPrint("Acceleration (m/s^2): x = ");
  radioPrint(String(accel.acceleration.x, 2));
  radioPrint(", y = ");
  radioPrint(String(accel.acceleration.y, 2));
  radioPrint(", z = ");
  radioPrintln(String(accel.acceleration.z, 2));

  radioPrint("Gyro (dps): x = ");
  radioPrint(String(gyro.gyro.x, 2));
  radioPrint(", y = ");
  radioPrint(String(gyro.gyro.y, 2));
  radioPrint(", z = ");
  radioPrintln(String(gyro.gyro.z, 2));

  radioPrint("Temperature: ");
  radioPrint(String(temp.temperature, 1));
  radioPrintln(" C");

  radioPrint("Magnetic field (uT): x = ");
  radioPrint(String(mag.magnetic.x, 2));
  radioPrint(", y = ");
  radioPrint(String(mag.magnetic.y, 2));
  radioPrint(", z = ");
  radioPrintln(String(mag.magnetic.z, 2));
}

void handleSensorCommand(uint8_t *buf, uint8_t len)
{
  if (len < 2)
  {
    radioPrintln("Sensor commands: sg(si/sb) print data, sG reinit GPS, sI reinit IMU");
    return;
  }

  char sub = (char)buf[1];

  switch (sub)
  {
  case 'g':
    radioPrintln("========== GPS DATA ==========");
    printGPSData();
    radioPrintln("==============================");
    break;
  case 'i':
    radioPrintln("========== IMU DATA ==========");
    printIMUData();
    radioPrintln("==============================");
    break;
  case 'b':
  case 'B':
    radioPrintln("========== SENSOR DATA ==========");
    printGPSData();
    radioPrintln("");
    printIMUData();
    radioPrintln("=================================");
    break;
  case 'G':
    initGPS();
    break;
  case 'I':
    initIMU();
    break;
  default:
    radioPrintln("Unknown sensor subcommand. Use sg, si, sb, sG, or sI");
    break;
  }
}

void initRPI()
{
  // Initialize Raspberry Pi control pin and make sure its off.
  pinMode(RPI_ENABLE, OUTPUT);
  digitalWrite(RPI_ENABLE, LOW);

  // Initialize UART for RPi communication
  Serial2.begin(UART_BAUD); // High-speed UART for data transfer
  radioPrintln("RPI UART initialized at 115200 baud");

  digitalWrite(RPI_ENABLE, HIGH); // Turn Pi ON
  radioPrintln("RPI POWER: ON (Wait until 'RPI STATUS: IDLE' before thermal capture.)");
}

// === Command handling (ground station CLI command names) ===

void cmdCapture()
{
  // The ground station prints this before forwarding 'u'; the Python CLI
  // uses it to timestamp the capture's metadata.
  Serial.println(THERMAL_CAPTURE_TIMESTAMP_FLAG);
  startLedBlink();
  captureThermalImageUART();
  stopLedBlink();
}

void cmdRequest(const String &args)
{
  if (args.length() != 1 || args[0] < '0' || args[0] > '2')
  {
    Serial.println("Usage: request <camera_id>  (0=lepton, 1=boson, 2=rpicam)");
    return;
  }

  Serial.println("\n--- REQUESTING THERMAL DATA DOWNLINK ---");
  downloadStartMs = millis();
  lastRequestedCameraId = (uint8_t)(args[0] - '0');
  autoExportOnComplete = true;

  // Blinks through the UART transfer from the Pi, the packet loopback and
  // the CSV/JPG export - the whole downlink.
  startLedBlink();
  requestThermalImageFromPi(lastRequestedCameraId);
  stopLedBlink();

  // A successful downlink consumes the flag in showReceptionSummary(). Still
  // set means the Pi never handed over an image - clear it so a later bare
  // 'export' doesn't get mislabeled, and say so for the GUI log.
  if (autoExportOnComplete)
  {
    autoExportOnComplete = false;
    Serial.println("No image downlinked for this request");
  }
}

void cmdRPIControl(const String &args)
{
  if (args == "on")
  {
    digitalWrite(RPI_ENABLE, HIGH); // Turn Pi ON
    radioPrintln("RPI POWER: ON (Wait until 'RPI STATUS: IDLE' before thermal capture.)");
  }
  else if (args == "off")
  {
    digitalWrite(RPI_ENABLE, LOW); // Turn Pi OFF
    RPI_IDLE_READY = false;        // Reset idle state
    piConnectedCameras = "";
    radioPrintln("RPI POWER: OFF");
  }
  else if (args == "status")
  {
    int state = digitalRead(RPI_ENABLE);
    radioPrint("RPI POWER: ");
    radioPrint(state ? "ON" : "OFF");
    radioPrint(" | MODE: ");
    radioPrintln(RPI_IDLE_READY ? "IDLE" : "NOT READY");
  }
  else
  {
    Serial.println("Usage: rpi <on|off|status>");
  }
}

void cmdSensor(const String &args)
{
  uint8_t buf[2] = {'s', 0};
  if (args == "gps" || args == "g")
    buf[1] = 'g';
  else if (args == "imu" || args == "i")
    buf[1] = 'i';
  else if (args == "both" || args == "b")
    buf[1] = 'b';
  else if (args == "gps_init" || args == "init_gps" || args == "reinit_gps" || args == "ginit")
    buf[1] = 'G';
  else if (args == "imu_init" || args == "init_imu" || args == "reinit_imu" || args == "iinit")
    buf[1] = 'I';
  else
  {
    Serial.println("Usage: sensor <gps|imu|both|gps_init|imu_init>");
    return;
  }
  handleSensorCommand(buf, 2);
}

// Camera name -> request id. Must match CAMERA_IDS in multicam_controller.py.
int cameraIdForName(const String &name)
{
  if (name == "lepton")
    return 0;
  if (name == "boson")
    return 1;
  if (name == "rpicam")
    return 2;
  return -1;
}

// Prints piConnectedCameras as "lepton (request 0), rpicam (request 2)".
void printConnectedCameras()
{
  if (piConnectedCameras.length() == 0)
  {
    Serial.println("unknown - Pi hasn't reported CAM_READY since boot (power-cycle it: rpi off, rpi on)");
    return;
  }
  if (piConnectedCameras == "none")
  {
    Serial.println("none (Pi reported NO_CAMERA)");
    return;
  }

  int start = 0;
  bool first = true;
  while (start <= (int)piConnectedCameras.length())
  {
    int comma = piConnectedCameras.indexOf(',', start);
    if (comma == -1)
      comma = piConnectedCameras.length();
    String name = piConnectedCameras.substring(start, comma);
    name.trim();
    if (name.length() > 0)
    {
      if (!first)
        Serial.print(", ");
      Serial.print(name);
      int id = cameraIdForName(name);
      if (id >= 0)
      {
        Serial.print(" (request ");
        Serial.print(id);
        Serial.print(")");
      }
      first = false;
    }
    start = comma + 1;
  }
  Serial.println();
}

void cmdStatus()
{
  Serial.println("\n=== OUTREACH SATELLITE STATUS (no RF) ===");
  Serial.print("RPi power: ");
  Serial.println(digitalRead(RPI_ENABLE) ? "ON" : "OFF");
  Serial.print("RPi mode: ");
  Serial.println(RPI_IDLE_READY ? "IDLE" : "NOT READY");
  Serial.print("Connected cameras: ");
  printConnectedCameras();
  Serial.print("Last image from RPi: ");
  Serial.print(capturedImageLength);
  Serial.println(" bytes");
  Serial.print("Last downlinked image: ");
  if (headerReceived)
  {
    Serial.print(receivedPackets);
    Serial.print("/");
    Serial.print(expectedPackets);
    Serial.print(" packets, camera ");
    Serial.println(lastRequestedCameraId == 0xFF ? String("unknown") : String(lastRequestedCameraId));
  }
  else
  {
    Serial.println("none");
  }
  Serial.print("Uptime (s): ");
  Serial.println(millis() / 1000);
}

/**
 * Deletes every queued capture on the Pi (UART "CLEAR"). Blocks until the
 * Pi reports CLEAR_DONE/CLEAR_ERROR and IDLE.
 *
 * @return true if the Pi reported CLEAR_DONE
 */
bool clearPiStorage()
{
  if (!RPI_IDLE_READY)
  {
    radioPrintln("Raspberry Pi is not ready yet, retry once in RPI STATUS: IDLE");
    return false;
  }

  radioPrintln("--- RPI STORAGE CLEAR ---");

  piCaptureInProgress = true;

  // Clear any pending UART bytes before sending the command
  while (Serial2.available())
    Serial2.read();

  Serial2.print(UART_CLEAR_CMD);
  Serial2.flush();

  bool cleared = false;
  bool done = false;

  while (!done)
  {
    uint32_t rxLen = 0;
    bool isStatus = false;

    if (!recvFramedFromPi(Serial2, piStatusBuf, MAX_STATUS_LEN, rxLen, isStatus))
    {
      radioPrintln("ERROR: Failed to receive clear status (is the Pi running a multicam_controller.py with CLEAR?)");
      break;
    }

    if (!isStatus)
    {
      radioPrintln("ERROR: Unexpected non-status payload during clear");
      break;
    }

    String msg = handleStatusPayload(piStatusBuf, rxLen);
    if (msg.startsWith("CLEAR_DONE"))
      cleared = true;
    if (msg == "IDLE")
      done = true;
  }

  piCaptureInProgress = false;

  radioPrintln(cleared && done ? "--- RPI STORAGE CLEAR COMPLETE ---" : "--- RPI STORAGE CLEAR FAILED ---");
  return cleared && done;
}

/**
 * Runs the command lines that arrived over USB while clearPiStorage() was
 * blocking, except capture/request, which are rejected rather than queued -
 * they would otherwise run right after the clear against storage the
 * operator just wiped, instead of being re-issued deliberately.
 */
void handleCommandsReceivedWhileClearing()
{
  String line = "";
  while (Serial.available() || line.length() > 0)
  {
    char c = Serial.available() ? Serial.read() : '\n'; // flush a trailing partial line
    if (c != '\n' && c != '\r')
    {
      line += c;
      continue;
    }

    line.trim();
    if (line.length() == 0)
      continue;

    String cmd = line;
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex != -1)
      cmd = cmd.substring(0, spaceIndex);
    cmd.toLowerCase();

    if (cmd == "capture" || cmd == "request")
    {
      Serial.print("Ignored '");
      Serial.print(line);
      Serial.println("' - sent while RPi storage was being cleared. Send it again.");
    }
    else
    {
      handleCommand(line);
    }
    line = "";
  }
}

void cmdClearStorage(const String &args)
{
  if (args != "confirm")
  {
    Serial.println("⚠️ This permanently deletes ALL captured images stored on the RPi. This cannot be undone.");
    Serial.println("Send 'clear_storage confirm' to proceed.");
    return;
  }

  clearPiStorage();
  handleCommandsReceivedWhileClearing();
}

void handleCommand(String line)
{
  line.trim();
  if (line.length() == 0)
    return;

  int spaceIndex = line.indexOf(' ');
  String cmd = (spaceIndex == -1) ? line : line.substring(0, spaceIndex);
  String args = (spaceIndex == -1) ? String("") : line.substring(spaceIndex + 1);
  cmd.toLowerCase();
  args.trim();
  args.toLowerCase();

  if (cmd == "capture")
    cmdCapture();
  else if (cmd == "request")
    cmdRequest(args);
  else if (cmd == "export")
    exportThermalData();
  else if (cmd == "clear_storage")
    cmdClearStorage(args);
  else if (cmd == "rpi")
    cmdRPIControl(args);
  else if (cmd == "sensor")
    cmdSensor(args);
  else if (cmd == "ping")
    radioPrintln("pong from satellite");
  else if (cmd == "status")
    cmdStatus();
  else if (cmd == "help")
    printHelp();
  else if (cmd == "reset" || cmd == "sat_reset")
    SCB_AIRCR = 0x05FA0004; // Software reset for Teensy 4.1 (ARM Cortex-M7)
  else
  {
    Serial.print("Error: Unknown command '");
    Serial.print(cmd);
    Serial.println("'");
    Serial.println("Type 'help' for available commands");
  }
}

/**
 * Main loop - drains RPi STATUS messages and runs USB serial commands.
 */
void loop()
{
  static String serialLine = "";

  serviceGPS();

  pollPIUartStatus();

  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (serialLine.length() > 0)
      {
        handleCommand(serialLine);
        serialLine = "";
      }
    }
    else
    {
      serialLine += c;
      if (serialLine.length() > 64)
        serialLine = ""; // Garbage / no newline - drop it
    }
  }
}

// readExact, endOK, payloadIsStatus, recvFramedFromPi defined in rpi_uart.hpp

// Standard CRC-16/CCITT-FALSE for cross-checking packet integrity
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; ++i)
    {
      if (crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Pretty-print a STATUS payload (strip "STATUS:")
String handleStatusPayload(const uint8_t *payload, uint16_t len)
{
  const size_t L = 7;
  String msg;
  if (len > L)
  {
    // copy ASCII into a String safely
    for (uint16_t i = L; i < len; ++i)
    {
      char c = (char)payload[i];
      if (c == '\r' || c == '\n')
        continue;
      msg += c;
    }
  }
  if (msg.length() == 0)
    msg = "(empty)";
  radioPrint("RPI STATUS: ");
  radioPrintln(msg);

  if (msg == "IDLE")
  {
    RPI_IDLE_READY = true;
  }
  else if (msg == "BOOT")
  {
    piConnectedCameras = ""; // Pi (re)booting - camera list about to be re-sent
  }
  else if (msg.startsWith("CAM_READY:"))
  {
    piConnectedCameras = msg.substring(strlen("CAM_READY:"));
  }
  else if (msg == "NO_CAMERA")
  {
    piConnectedCameras = "none";
  }

  return msg;
}

// Drain any unsolicited framed UART messages (typically STATUS packets) while idle
void pollPIUartStatus()
{
  if (piCaptureInProgress)
    return;

  while (Serial2.available() >= UART_HEADER_SIZE)
  {
    uint32_t rxLen = 0;
    bool isStatus = false;

    if (!recvFramedFromPi(Serial2, piStatusBuf, MAX_STATUS_LEN, rxLen, isStatus, 200))
    {
      return; // recvFramedFromPi already reported the error
    }

    if (isStatus)
    {
      handleStatusPayload(piStatusBuf, rxLen);
    }
    else
    {
      radioPrint("RPI payload received while idle (");
      radioPrint(String(rxLen));
      radioPrintln(" bytes)");
    }
  }
}

/**
 * Requests a specific camera's most recent image from the Raspberry Pi via
 * UART, caches it into imgBuf, and downlinks it via sendThermalData().
 *
 * @param cameraId Camera id to request (0=lepton, 1=boson, 2=rpicam)
 */
void requestThermalImageFromPi(uint8_t cameraId)
{
  if (!RPI_IDLE_READY)
  {
    radioPrintln("Raspberry Pi is not ready yet, retry once in RPI STATUS: IDLE");
    return;
  }

  radioPrintln("--- UART REQUEST ---");
  radioPrint("Requesting image for camera ");
  radioPrintln(String(cameraId));

  piCaptureInProgress = true;

  // Clear any pending UART bytes before sending the command
  while (Serial2.available())
    Serial2.read();

  Serial2.print(UART_REQUEST_CMD);
  Serial2.print(' ');
  Serial2.print(cameraId);
  Serial2.print('\n');
  Serial2.flush();

  bool gotImage = false;
  bool requestFailed = false;
  bool done = false;

  while (!done)
  {
    uint32_t rxLen = 0;
    bool isStatus = false;

    if (!recvFramedFromPi(Serial2, imgBuf, MAX_IMG, rxLen, isStatus,
                          UART_HEADER_TIMEOUT_MS, piStatusBuf, MAX_STATUS_LEN))
    {
      radioPrintln("ERROR: Failed to receive requested image");
      piCaptureInProgress = false;
      return;
    }

    if (isStatus)
    {
      // Status frames go to piStatusBuf (not imgBuf) so the "IDLE" that
      // follows REQUEST_DONE can't clobber an already-received image.
      String msg = handleStatusPayload(piStatusBuf, rxLen);
      if (msg.startsWith("REQUEST_ERROR"))
        requestFailed = true;
      if (msg == "IDLE")
        done = true;
    }
    else
    {
      capturedImageLength = rxLen;
      gotImage = true;
    }
  }

  piCaptureInProgress = false;

  if (!gotImage || requestFailed)
  {
    radioPrintln("--- UART REQUEST FAILED ---");
    return;
  }

  radioPrint("Image received: ");
  radioPrint(String(capturedImageLength));
  radioPrintln(" bytes");
  radioPrintln("--- UART REQUEST COMPLETE ---");

  sendThermalData();
}

/**
 * Triggers a capture on the Raspberry Pi via UART. The Pi saves each capture
 * to its on-disk queue and only reports STATUS text; the image itself is
 * pulled later with requestThermalImageFromPi().
 *
 * Unlike satellite_teensy.ino (which returns at the first CAPTURE_DONE),
 * this waits for the Pi's IDLE, which it sends after every connected camera
 * has been captured. Otherwise "--- UART CAPTURE COMPLETE ---" fires while
 * the Pi is still capturing the other cameras, and a request sent then races
 * it. The GUI's sequential Request button relies on the full CAPTURE_DONE
 * list having arrived by the time capture reports complete.
 */
void captureThermalImageUART()
{
  if (!RPI_IDLE_READY)
  {
    radioPrintln("Raspberry Pi is not ready yet, retry once in RPI STATUS: IDLE");
    return;
  }

  radioPrintln("--- UART CAPTURE ---");
  radioPrintln("Triggering RPI capture...");

  piCaptureInProgress = true;

  // Clear any pending UART bytes before sending the command
  while (Serial2.available())
    Serial2.read();

  // Send UART capture command to Raspberry Pi
  Serial2.print(UART_CAPTURE_CMD);
  Serial2.flush(); // Ensure command is sent immediately

  radioPrintln("Waiting for capture status from RPI...");

  bool captureComplete = false;

  while (!captureComplete)
  {
    uint32_t rxLen = 0;
    bool isStatus = false;

    if (!recvFramedFromPi(Serial2, piStatusBuf, MAX_STATUS_LEN, rxLen, isStatus))
    {
      radioPrintln("ERROR: Failed to receive capture status");
      piCaptureInProgress = false;
      return;
    }

    if (!isStatus)
    {
      radioPrintln("ERROR: Unexpected non-status payload during capture");
      piCaptureInProgress = false;
      return;
    }

    String msg = handleStatusPayload(piStatusBuf, rxLen);
    if (msg == "IDLE")
    {
      captureComplete = true;
    }
  }

  piCaptureInProgress = false;

  radioPrintln("--- UART CAPTURE COMPLETE ---");

  // Send GPS/IMU data so the ground station stores it with the capture.
  // Skipped when not connected - the outreach bench usually has neither, and
  // "GPS not connected" after every visitor's capture is just noise. The
  // 'sensor' command still reports them unconditionally.
//   if (gpsConnected)
//     printGPSData();
//   if (imuConnected)
//     printIMUData();
}

/**
 * "Transmits" a packet. On the flight build this is an RF23 send with
 * retries; here the packet is handed straight to the ground station receive
 * path below, so the packet framing and CRCs are still exercised end to end.
 *
 * @return true if packet was delivered, false if its length is invalid
 */
bool sendPacketReliable(uint8_t *data, uint8_t len)
{
  if (len == 0 || len > RADIO_PACKET_MAX_SIZE)
  {
    radioPrint("Invalid packet length: ");
    radioPrintln(String(len));
    return false;
  }

  processPacket(data, len);
  return true;
}

/**
 * Packetizes the image in imgBuf into header, data and end packets - same
 * framing as the flight radio downlink - and sends them via
 * sendPacketReliable().
 */
void sendThermalData()
{
  if (capturedImageLength == 0)
  {
    radioPrintln("No image captured yet!");
    return;
  }

  const uint16_t totalPackets = (capturedImageLength + PACKET_DATA_SIZE - 1) / PACKET_DATA_SIZE;
  if (totalPackets == 0)
  {
    radioPrintln("No packets to transmit");
    return;
  }

  const uint16_t imageCrc = crc16_ccitt(imgBuf, capturedImageLength);

  // Build and send header packet
  ThermalHeaderPacket headerPacket;
  headerPacket.marker1 = 0xFF;
  headerPacket.marker2 = 0xFF;
  headerPacket.imageLength = capturedImageLength;
  headerPacket.totalPackets = totalPackets;
  headerPacket.magic[0] = 0xDEAD;
  headerPacket.magic[1] = 0xBEEF;

  if (!sendPacketReliable((uint8_t *)&headerPacket, sizeof(headerPacket)))
  {
    radioPrintln("❌ Failed to send header!");
    return;
  }

  uint32_t bytesSent = 0;
  uint16_t packetNum = 0;
  uint16_t successCount = 0;
  unsigned long startTime = millis();

  while (bytesSent < capturedImageLength)
  {
    uint32_t remaining = capturedImageLength - bytesSent;
    uint16_t chunkSize = (remaining < (uint16_t)PACKET_DATA_SIZE) ? remaining : (uint16_t)PACKET_DATA_SIZE;

    // Build data packet
    ThermalDataPacket *dataPacket = (ThermalDataPacket *)txBuffer;
    dataPacket->packetIndex = packetNum;
    memcpy(dataPacket->data, &imgBuf[bytesSent], chunkSize);

    // CRC goes right after the actual data, not at the struct's fixed offset
    uint16_t packetCrc = crc16_ccitt(dataPacket->data, chunkSize);
    uint16_t crcOffset = 2 + chunkSize;
    memcpy(&txBuffer[crcOffset], &packetCrc, sizeof(uint16_t));

    // Packet structure: [2 bytes packetIndex][variable data][2 bytes CRC]
    uint8_t frameLen = 2 + chunkSize + 2;

    if (sendPacketReliable(txBuffer, frameLen))
    {
      successCount++;
    }

    bytesSent += chunkSize;
    packetNum++;
  }

  // Build and send end packet
  ThermalEndPacket endPacket;
  endPacket.marker1 = 0xEE;
  endPacket.marker2 = 0xEE;
  endPacket.packetCount = packetNum;
  endPacket.imageCrc = imageCrc;

  if (!sendPacketReliable((uint8_t *)&endPacket, sizeof(endPacket)))
  {
    radioPrintln("❌ Failed to send end data packet!");
    return;
  }

  float duration = (millis() - startTime) / 1000.0f;

  radioPrintln("=== SAT TRANSMISSION COMPLETE ===");
  radioPrint("SAT Packets sent ok: ");
  radioPrint(String(successCount));
  radioPrint("/");
  radioPrintln(String(totalPackets));
  radioPrint("Elapsed (s): ");
  radioPrintln(String(duration, 2));
  radioPrint("Image CRC16: 0x");
  radioPrintln(String(imageCrc, HEX));
}

// ============================================================================
// GROUND STATION RECEIVE PATH (from ground_station_teensy.ino)
// Prints exactly what the ground station Teensy prints, so the Python CLI /
// web GUI parse it unchanged. No retry logic - the loopback can't drop
// packets, so anything missing is reported and left for manual inspection.
// ============================================================================

void handleThermalHeaderPacket(uint8_t *buf)
{
  ThermalHeaderPacket *headerPacket = (ThermalHeaderPacket *)buf;

  expectedLength = headerPacket->imageLength;
  expectedPackets = headerPacket->totalPackets;

  Serial.println("\n📦 THERMAL IMAGE HEADER RECEIVED!");
  Serial.print("Expected size: ");
  Serial.print(expectedLength);
  Serial.println(" bytes");
  Serial.print("Expected packets: ");
  Serial.println(expectedPackets);

  if (expectedLength > MAX_IMG)
  {
    Serial.println("✗ Header length exceeds local buffer; aborting reception");
    headerReceived = false;
    return;
  }

  // Validate magic bytes (0xDEAD 0xBEEF)
  if (headerPacket->magic[0] == 0xDEAD && headerPacket->magic[1] == 0xBEEF)
  {
    Serial.println("✓ Header valid - receiving thermal data...");
    headerReceived = true;
    imageComplete = false;
    receivedPackets = 0;
    expectedImageCrc = 0;
    lastComputedImageCrc = 0;
    crcErrorCount = 0;
    crcVerified = false;
    lastReportedQuarter = 0;
    memset(packetReceived, false, sizeof(packetReceived));
    memset(rxImgBuf, 0, expectedLength);
  }
  else
  {
    Serial.println("✗ Invalid header!");
  }
}

void handleThermalDataPacket(uint8_t *buf, uint8_t len)
{
  if (len <= THERMAL_PACKET_OVERHEAD)
  {
    Serial.println("ERROR: data packet len less than thermal packet overhead?!");
    return;
  }

  ThermalDataPacket *dataPacket = (ThermalDataPacket *)buf;
  uint8_t dataLen = len - THERMAL_PACKET_OVERHEAD;
  uint16_t packetNum = dataPacket->packetIndex;

  // Verify CRC
  uint16_t crcOffset = 2 + dataLen;
  uint16_t packetCrc;
  memcpy(&packetCrc, &buf[crcOffset], sizeof(uint16_t));
  uint16_t computedCrc = crc16_ccitt(dataPacket->data, dataLen);

  if (computedCrc != packetCrc)
  {
    crcErrorCount++;
    if (crcErrorCount <= 10)
    {
      Serial.print("CRC mismatch on packet ");
      Serial.print(packetNum);
      Serial.print(" (expected 0x");
      Serial.print(packetCrc, HEX);
      Serial.print(", computed 0x");
      Serial.print(computedCrc, HEX);
      Serial.println(")");
      if (crcErrorCount == 10)
      {
        Serial.println("Further CRC mismatch logs suppressed");
      }
    }
    return; // Discard corrupt packet
  }

  if (packetNum >= MAX_PACKETS || packetReceived[packetNum])
    return;

  uint32_t bufferPos = (uint32_t)packetNum * PACKET_DATA_SIZE;
  if (bufferPos + dataLen > expectedLength)
    return; // Buffer overflow protection

  memcpy(&rxImgBuf[bufferPos], dataPacket->data, dataLen);
  packetReceived[packetNum] = true;
  receivedPackets++;

  // Progress indication - every 25%. The web GUI's progress bar parses these.
  float progress = (float)receivedPackets / expectedPackets * 100;
  uint8_t currentQuarter = progress / 25;
  if (currentQuarter > lastReportedQuarter && currentQuarter <= 4)
  {
    lastReportedQuarter = currentQuarter;
    Serial.print(currentQuarter * 25);
    Serial.println("%");
  }
}

void handleThermalEndPacket(uint8_t *buf)
{
  if (imageComplete)
    return;

  ThermalEndPacket *endPacket = (ThermalEndPacket *)buf;
  uint16_t pktCount = endPacket->packetCount;
  expectedImageCrc = endPacket->imageCrc;

  Serial.println("\n🏁 END PACKET RECEIVED!");
  Serial.print("Transmitter sent ");
  Serial.print(pktCount);
  Serial.println(" packets");

  if (expectedLength > 0)
  {
    lastComputedImageCrc = crc16_ccitt(rxImgBuf, expectedLength);
    crcVerified = (expectedImageCrc == lastComputedImageCrc);
    Serial.print("Image CRC16 expected 0x");
    Serial.print(expectedImageCrc, HEX);
    Serial.print(", computed 0x");
    Serial.print(lastComputedImageCrc, HEX);
    Serial.println(crcVerified ? " (match)" : " (MISMATCH)");
  }

  if (pktCount != expectedPackets)
  {
    Serial.print("⚠️ End packet reports ");
    Serial.print(pktCount);
    Serial.print(" packets but header expected ");
    Serial.println(expectedPackets);
  }

  imageComplete = (receivedPackets >= expectedPackets);
  showReceptionSummary();
}

void processPacket(uint8_t *buf, uint8_t len)
{
  // Header packet (12 bytes with magic bytes)
  if (len == 12 && buf[0] == 0xFF && buf[1] == 0xFF)
  {
    handleThermalHeaderPacket(buf);
  }
  // End packet (6 bytes with magic bytes)
  else if (len == 6 && buf[0] == 0xEE && buf[1] == 0xEE)
  {
    handleThermalEndPacket(buf);
  }
  // Data packet (requires valid header and incomplete image)
  else if (headerReceived && !imageComplete && len >= 3)
  {
    handleThermalDataPacket(buf, len);
  }
  else
  {
    Serial.print("Unknown packet (header=");
    Serial.print(headerReceived);
    Serial.print(" complete=");
    Serial.print(imageComplete);
    Serial.print(" len=");
    Serial.print(len);
    Serial.println(")");
  }
}

void showReceptionSummary()
{
  Serial.println("\n=== RECEPTION SUMMARY ===");
  Serial.print("Received ");
  Serial.print(receivedPackets);
  Serial.print(" of ");
  Serial.print(expectedPackets);
  Serial.print(" packets (");
  float pct = (expectedPackets > 0) ? ((float)receivedPackets / expectedPackets * 100.0f) : 0.0f;
  Serial.print(pct, 1);
  Serial.println("%)");

  if (crcErrorCount > 0)
  {
    Serial.print("Packets discarded (CRC): ");
    Serial.println(crcErrorCount);
  }

  if (expectedPackets > 0 && receivedPackets < expectedPackets)
  {
    Serial.print("Missing packets: ");
    Serial.println(expectedPackets - receivedPackets);
  }

  if (receivedPackets == expectedPackets)
  {
    Serial.println("\n✅ PERFECT RECEPTION!");
  }
  else
  {
    Serial.println("\n❌ Incomplete reception");
  }

  // Auto-export when this reception was triggered by 'request <id>'.
  if (autoExportOnComplete)
  {
    autoExportOnComplete = false;
    if (headerReceived)
    {
      exportThermalData();
    }
  }

  Serial.println("\n**** Thermal Data Download Statistics ****");
  Serial.print("Total Duration (request → completion): ");
  Serial.print((millis() - downloadStartMs) / 1000.0f, 2);
  Serial.println(" s");
}

// Standard base64 (RFC 4648) encoder, line-wrapped at 76 chars so the
// Python CLI's line-based reader can capture it like CSV rows.
void printBase64(const uint8_t *data, uint32_t len)
{
  static const char b64Table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint8_t lineChars = 0;

  for (uint32_t i = 0; i < len; i += 3)
  {
    uint32_t remaining = len - i;
    uint8_t b0 = data[i];
    uint8_t b1 = (remaining > 1) ? data[i + 1] : 0;
    uint8_t b2 = (remaining > 2) ? data[i + 2] : 0;

    char out[4];
    out[0] = b64Table[b0 >> 2];
    out[1] = b64Table[((b0 & 0x03) << 4) | (b1 >> 4)];
    out[2] = (remaining > 1) ? b64Table[((b1 & 0x0F) << 2) | (b2 >> 6)] : '=';
    out[3] = (remaining > 2) ? b64Table[b2 & 0x3F] : '=';

    Serial.write(out, 4);
    lineChars += 4;
    if (lineChars >= 76)
    {
      Serial.println();
      lineChars = 0;
    }
  }

  if (lineChars > 0)
    Serial.println();
}

// Exports the most recently downlinked image in the correct format for
// lastRequestedCameraId: CSV thermal grid for lepton (0) / boson (1),
// base64-encoded JPEG for rpicam (2). Same output as the ground station.
void exportThermalData()
{
  if (!headerReceived)
  {
    Serial.println("No complete thermal image to export");
    return;
  }

  if (!crcVerified)
  {
    Serial.println("⚠️ Warning: image CRC mismatch – export may contain corrupt data");
  }

  if (lastRequestedCameraId == 2)
  {
    // rpicam: raw JPEG bytes, no pixel-grid interpretation
    Serial.println("\n--- EXPORTING RPICAM IMAGE ---");
    Serial.println(THERMAL_JPG_START);
    printBase64(rxImgBuf, expectedLength);
    Serial.println(THERMAL_JPG_END);
    return;
  }

  // Lepton (0) and boson (1): 16-bit thermal grid, printed as CSV.
  // Default to the legacy boson dimensions (320x256).
  uint16_t width = 320;
  uint16_t height = 256;
  uint32_t expectedBytes = 163840;
  if (lastRequestedCameraId == 0)
  {
    width = 160;
    height = 120;
    expectedBytes = 38400;
  }

  if (expectedLength != expectedBytes)
  {
    Serial.println("⚠️ Warning: image size doesn't match expected camera resolution – export may be misaligned");
  }

  // Camera-source tag the Python CLI reads to name the output file.
  if (lastRequestedCameraId == 0)
  {
    Serial.println("\n--- EXPORTING LEPTON THERMAL DATA ---");
  }
  else if (lastRequestedCameraId == 1)
  {
    Serial.println("\n--- EXPORTING BOSON THERMAL DATA ---");
  }
  else
  {
    Serial.println("\n--- EXPORTING THERMAL DATA ---");
  }
  Serial.println("Copying data below to a 'thermal_image.csv'");
  Serial.println(THERMAL_CSV_START);

  // One raw 16-bit little-endian value per pixel, comma-separated.
  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col++)
    {
      uint32_t idx = (row * width + col) * 2; // 2 bytes per pixel
      if (idx < MAX_IMG - 1)
      {
        uint16_t pixel = rxImgBuf[idx] | (rxImgBuf[idx + 1] << 8);
        Serial.print(pixel);
      }
      else
      {
        Serial.print("NaN"); // Buffer overflow protection
      }
      if (col < width - 1)
        Serial.print(",");
    }
    Serial.println();
  }

  Serial.println(THERMAL_CSV_END);
}
