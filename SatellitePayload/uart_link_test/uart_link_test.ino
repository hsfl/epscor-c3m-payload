/**
 * RPi <-> Satellite Teensy UART Link Test Harness
 *
 * Standalone Teensy sketch (no radio) for bench-testing the UART link
 * between the multicam RPi and the satellite Teensy: CAPTURE/REQUEST
 * commands, framing, and error paths, against real hardware.
 *
 * Wiring: same Serial2 (pins 7 RX / 8 TX) connection used by
 * satellite_teensy.ino. Interact over USB Serial at 115200 baud.
 *
 * Commands (type into the USB serial monitor):
 *   capture           - capture from every connected camera
 *   capture <id>      - capture from one camera (0=lepton, 1=boson, 2=rpicam)
 *   request <id>      - fetch the most recently captured image for camera <id>
 *   cameras           - show which cameras the RPi reported as connected at boot
 *   auto              - run the full automated test sequence below
 *   help              - show this list
 *
 * "auto" runs: CAPTURE (all) -> REQUEST 0/1/2 -> REQUEST <invalid id> ->
 * CAPTURE <invalid id>, checking that each behaves as designed (a real
 * image frame comes back for a connected+captured camera, and a clean
 * *_ERROR:UNKNOWN_ID status comes back for an invalid one), and prints a
 * pass/fail/skip summary.
 *
 * CAPTURE/REQUEST here mirror captureThermalImageUART()/
 * requestThermalImageFromPi() in satellite_teensy.ino: same RPI_IDLE_READY
 * gate, same piCaptureInProgress bookkeeping, same command framing and
 * STATUS-text handling (handleStatusPayload()). The one deliberate
 * difference is that this harness blocks until the RPi reports IDLE again
 * before returning, since it's a synchronous CLI with nothing else to do
 * meanwhile - satellite_teensy.ino instead returns after the first terminal
 * status and drains the rest later via pollPIUartStatus() from its main loop.
 */
#include "rpi_uart.hpp"

// Declared before any function definitions: the Arduino build inserts
// auto-generated function prototypes right after the #includes, so any
// struct used in a function signature must already be visible there.
struct FrameResult
{
  bool isStatus;
  String statusMsg; // valid only if isStatus
};

// rpi_uart.hpp expects these from the main sketch to forward log text over
// radio; this harness has no radio, so route them to USB Serial instead.
void radioPrint(const String &message) { Serial.print(message); }
void radioPrintln(const String &message) { Serial.println(message); }

// Matches MAX_IMG in satellite_teensy.ino - largest single frame we can hold.
const uint32_t MAX_FRAME = 165000;
uint8_t frameBuf[MAX_FRAME];

// Power control pin for Raspberry Pi, same as satellite_teensy.ino.
const uint8_t RPI_ENABLE = 36;

// Same globals satellite_teensy.ino uses to gate CAPTURE/REQUEST and to
// avoid the passive status poll racing an in-flight command.
bool RPI_IDLE_READY = false;
bool piCaptureInProgress = false;

// Camera names the RPi reported connected in its boot-time "CAM_READY:..."
// status (see multicam_controller.py's detect_connected_cameras()). There is
// no on-demand UART query for this - it's only ever sent once at boot - so
// we just remember the last one we saw.
String connectedCameras = "(unknown - waiting for RPi boot status)";

// Standard CRC-16/CCITT-FALSE, same as satellite_teensy.ino's crc16_ccitt -
// duplicated here since this is a standalone sketch (no shared .cpp/.o).
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; ++i)
    {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// Pretty-print a STATUS payload (strip "STATUS:") and update RPI_IDLE_READY /
// connectedCameras. Same logic as satellite_teensy.ino's handleStatusPayload.
String handleStatusPayload(const uint8_t *payload, uint16_t len)
{
  const size_t L = 7;
  String msg;
  if (len > L)
  {
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

  Serial.print("  [STATUS] ");
  Serial.println(msg);

  if (msg == "IDLE")
  {
    RPI_IDLE_READY = true;
  }

  if (msg.startsWith("CAM_READY:"))
  {
    connectedCameras = msg.substring(10);
  }

  return msg;
}

// Read and print one framed message from the Pi. Returns false on a hard
// read error (recvFramedFromPi already logged the reason via radioPrintln).
bool receiveAndReport(FrameResult &out)
{
  uint32_t rxLen = 0;
  bool isStatus = false;
  if (!recvFramedFromPi(Serial2, frameBuf, MAX_FRAME, rxLen, isStatus, 200))
  {
    return false;
  }

  out.isStatus = isStatus;

  if (isStatus)
  {
    out.statusMsg = handleStatusPayload(frameBuf, rxLen);
  }
  else
  {
    uint16_t crc = crc16_ccitt(frameBuf, rxLen);
    Serial.print("  [DATA] ");
    Serial.print(rxLen);
    Serial.print(" bytes, CRC16=0x");
    Serial.println(crc, HEX);
  }

  return true;
}

// Drain any unsolicited framed UART messages (typically STATUS packets)
// while idle. Same purpose as satellite_teensy.ino's pollPIUartStatus().
void pollPIUartStatus()
{
  if (piCaptureInProgress)
    return;

  while (Serial2.available() >= UART_HEADER_SIZE)
  {
    FrameResult r;
    if (!receiveAndReport(r))
      return; // error already logged
  }
}

/**
 * Trigger a capture on the RPi, same command framing and gating as
 * captureThermalImageUART() in satellite_teensy.ino: bails out if the Pi
 * hasn't reported IDLE yet, sends "CAPTURE\n" (all cameras) or
 * "CAPTURE <id>\n" (one camera), and watches for the terminal per-camera
 * status (CAPTURE_DONE/CAPTURE_ERROR/NO_FRAME). Unlike the flight code, this
 * keeps draining status frames after that until IDLE reappears, since this
 * harness has nothing else to do while a command is in flight.
 *
 * @param cameraId  camera to capture, or -1 for every connected camera
 * @param lastMsg   set to the last non-IDLE terminal status text seen
 * @return true if every terminal status seen was a *_DONE/NO_FRAME, not *_ERROR
 */
bool doCapture(int cameraId, uint32_t timeout_ms, String &lastMsg)
{
  lastMsg = "";

  if (!RPI_IDLE_READY)
  {
    Serial.println("RPi is not ready yet, retry once it reports IDLE");
    return false;
  }

  Serial.println("--- UART CAPTURE ---");
  Serial.println("Triggering RPi capture...");

  piCaptureInProgress = true;

  while (Serial2.available())
    Serial2.read();

  if (cameraId < 0)
  {
    Serial2.print(UART_CAPTURE_CMD); // "CAPTURE\n" - every connected camera
  }
  else
  {
    Serial2.print("CAPTURE");
    Serial2.print(' ');
    Serial2.print(cameraId);
    Serial2.print('\n');
  }
  Serial2.flush();

  Serial.println("Waiting for capture status from RPi...");

  bool sawTerminal = false;
  bool sawError = false;
  bool idleSeen = false;
  uint32_t start = millis();

  while (!idleSeen)
  {
    if (millis() - start > timeout_ms)
    {
      Serial.println("  [TIMEOUT] no IDLE status within window");
      piCaptureInProgress = false;
      return false;
    }

    if (Serial2.available() < UART_HEADER_SIZE)
    {
      delay(1);
      continue;
    }

    uint32_t rxLen = 0;
    bool isStatus = false;
    if (!recvFramedFromPi(Serial2, frameBuf, MAX_FRAME, rxLen, isStatus))
    {
      Serial.println("ERROR: Failed to receive capture status");
      piCaptureInProgress = false;
      return false;
    }

    if (!isStatus)
    {
      Serial.println("ERROR: Unexpected non-status payload during capture");
      piCaptureInProgress = false;
      return false;
    }

    String msg = handleStatusPayload(frameBuf, rxLen);
    if (msg == "IDLE")
    {
      idleSeen = true;
    }
    else if (msg.startsWith("CAPTURE_DONE") || msg.startsWith("CAPTURE_ERROR") || msg.startsWith("NO_FRAME"))
    {
      sawTerminal = true;
      lastMsg = msg;
      if (msg.indexOf("ERROR") >= 0)
        sawError = true;
    }
  }

  piCaptureInProgress = false;
  Serial.println("--- UART CAPTURE COMPLETE ---");

  return sawTerminal && !sawError;
}

/**
 * Request a specific camera's most recently captured image from the RPi.
 * Same command framing and gating as requestThermalImageFromPi() in
 * satellite_teensy.ino: bails out if the Pi hasn't reported IDLE yet, sends
 * "REQUEST <id>\n", and loops until IDLE reappears, tracking whether an
 * image frame actually came back and whether the Pi reported REQUEST_ERROR.
 *
 * @param cameraId    camera id to request (0=lepton, 1=boson, 2=rpicam)
 * @param dataFrames  set to the number of image frames received (0 or 1)
 * @param lastMsg     set to the last non-IDLE status text seen
 * @return true if exactly one image frame came back with no REQUEST_ERROR
 */
bool doRequest(uint8_t cameraId, uint32_t timeout_ms, uint32_t &dataFrames, String &lastMsg)
{
  dataFrames = 0;
  lastMsg = "";

  if (!RPI_IDLE_READY)
  {
    Serial.println("RPi is not ready yet, retry once it reports IDLE");
    return false;
  }

  Serial.println("--- UART REQUEST ---");
  Serial.print("Requesting image for camera ");
  Serial.println(cameraId);

  piCaptureInProgress = true;

  while (Serial2.available())
    Serial2.read();

  Serial2.print(UART_REQUEST_CMD); // "REQUEST"
  Serial2.print(' ');
  Serial2.print(cameraId);
  Serial2.print('\n');
  Serial2.flush();

  bool gotImage = false;
  bool requestFailed = false;
  bool done = false;
  uint32_t start = millis();

  while (!done)
  {
    if (millis() - start > timeout_ms)
    {
      Serial.println("  [TIMEOUT] no IDLE status within window");
      piCaptureInProgress = false;
      return false;
    }

    if (Serial2.available() < UART_HEADER_SIZE)
    {
      delay(1);
      continue;
    }

    uint32_t rxLen = 0;
    bool isStatus = false;
    if (!recvFramedFromPi(Serial2, frameBuf, MAX_FRAME, rxLen, isStatus))
    {
      Serial.println("ERROR: Failed to receive requested image");
      piCaptureInProgress = false;
      return false;
    }

    if (isStatus)
    {
      String msg = handleStatusPayload(frameBuf, rxLen);
      if (msg.startsWith("REQUEST_ERROR"))
      {
        requestFailed = true;
        lastMsg = msg;
      }
      if (msg == "IDLE")
        done = true;
    }
    else
    {
      uint16_t crc = crc16_ccitt(frameBuf, rxLen);
      Serial.print("  [DATA] ");
      Serial.print(rxLen);
      Serial.print(" bytes, CRC16=0x");
      Serial.println(crc, HEX);
      dataFrames++;
      gotImage = true;
    }
  }

  piCaptureInProgress = false;

  if (!gotImage || requestFailed)
  {
    Serial.println("--- UART REQUEST FAILED ---");
    return false;
  }

  Serial.println("--- UART REQUEST COMPLETE ---");
  return true;
}

void printHelp()
{
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  capture           - capture from all connected cameras");
  Serial.println("  capture <id>      - capture from one camera (0=lepton,1=boson,2=rpicam)");
  Serial.println("  request <id>      - fetch most recent capture for camera <id>");
  Serial.println("  cameras           - show cameras the RPi reported connected at boot");
  Serial.println("  auto              - run automated capture/request/error-path test suite");
  Serial.println("  help              - show this message");
  Serial.println();
}

void printCameras()
{
  Serial.print("Connected cameras (per RPi boot status): ");
  Serial.println(connectedCameras);
}

void runAutoTest()
{
  Serial.println();
  Serial.println("=== AUTO TEST: RPi <-> Teensy UART Link ===");

  printCameras();

  int passed = 0;
  int total = 0;
  uint32_t dataFrames;
  String lastMsg;

  // 1) Capture from every connected camera
  total++;
  Serial.println("\n[1] CAPTURE (all cameras)");
  bool captureOk = doCapture(-1, 30000, lastMsg);
  Serial.println(captureOk ? "  PASS" : "  FAIL");
  if (captureOk)
    passed++;

  // 2) Request each known camera ID's most recent capture
  const char *names[3] = {"lepton (id 0)", "boson (id 1)", "rpicam (id 2)"};
  for (uint8_t id = 0; id < 3; id++)
  {
    total++;
    Serial.print("\n[REQUEST ");
    Serial.print(id);
    Serial.print("] ");
    Serial.println(names[id]);

    bool ok = doRequest(id, 15000, dataFrames, lastMsg);

    if (!ok && lastMsg.indexOf("NO_CAPTURE") >= 0)
    {
      Serial.println("  SKIP (no capture queued for this camera - not connected?)");
      total--; // don't penalize a camera that simply isn't present on this bench
    }
    else if (!ok)
    {
      Serial.println("  FAIL (" + lastMsg + ")");
    }
    else
    {
      Serial.println("  PASS");
      passed++;
    }
  }

  // 3) Error paths: an ID that can't map to any camera should be rejected cleanly
  total++;
  Serial.println("\n[ERROR CHECK] REQUEST invalid id 99");
  bool ok = doRequest(99, 10000, dataFrames, lastMsg);
  bool errPass = !ok && lastMsg.indexOf("UNKNOWN_ID") >= 0;
  Serial.println(errPass ? "  PASS (rejected as expected)" : "  FAIL");
  if (errPass)
    passed++;

  total++;
  Serial.println("\n[ERROR CHECK] CAPTURE invalid id 99");
  ok = doCapture(99, 10000, lastMsg);
  errPass = !ok && lastMsg.indexOf("UNKNOWN_ID") >= 0;
  Serial.println(errPass ? "  PASS (rejected as expected)" : "  FAIL");
  if (errPass)
    passed++;

  Serial.println();
  Serial.print("=== RESULT: ");
  Serial.print(passed);
  Serial.print("/");
  Serial.print(total);
  Serial.println(" checks passed ===");
}

// Pull the optional trailing integer id off a command line, e.g. "capture 1"
// -> 1. Returns -1 if no id was given.
int parseOptionalId(const String &line)
{
  int sp = line.indexOf(' ');
  if (sp < 0)
    return -1;
  String rest = line.substring(sp + 1);
  rest.trim();
  if (rest.length() == 0)
    return -1;
  return rest.toInt();
}

void handleCommand(const String &line)
{
  String upper = line;
  upper.toUpperCase();

  String lastMsg;

  if (upper == "HELP" || upper == "?")
  {
    printHelp();
  }
  else if (upper == "AUTO")
  {
    runAutoTest();
  }
  else if (upper == "CAMERAS")
  {
    printCameras();
  }
  else if (upper.startsWith("CAPTURE"))
  {
    doCapture(parseOptionalId(line), 30000, lastMsg);
  }
  else if (upper.startsWith("REQUEST"))
  {
    int id = parseOptionalId(line);
    if (id < 0)
    {
      Serial.println("REQUEST needs a camera id, e.g. 'request 0'");
    }
    else
    {
      uint32_t dataFrames;
      doRequest((uint8_t)id, 15000, dataFrames, lastMsg);
    }
  }
  else
  {
    Serial.println("Unknown command. Type 'help'.");
  }
}

// Same sequence as satellite_teensy.ino's initRPI(): hold the Pi off while
// the UART is brought up, then power it on.
void initRPI()
{
  pinMode(RPI_ENABLE, OUTPUT);
  digitalWrite(RPI_ENABLE, LOW);

  Serial2.begin(UART_BAUD);
  Serial.println("RPI UART initialized at 115200 baud");

  digitalWrite(RPI_ENABLE, HIGH); // Turn Pi ON
  Serial.println("RPI POWER: ON (Wait until 'RPI STATUS: IDLE' before thermal capture.)");
}

void setup()
{
  Serial.begin(115200);
  uint32_t bootStart = millis();
  while (!Serial && millis() - bootStart < 3000)
  {
  }

  initRPI();

  Serial.println("=== RPi <-> Satellite Teensy UART Link Test Harness ===");
  Serial.println("Listening for RPi boot status on Serial2...");
  printHelp();
}

void loop()
{
  // Passively drain and print any unsolicited status frames (BOOT,
  // CAM_READY, IDLE, etc.) so boot-time status is visible even before the
  // first command is typed.
  pollPIUartStatus();

  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
      handleCommand(line);
  }
}
