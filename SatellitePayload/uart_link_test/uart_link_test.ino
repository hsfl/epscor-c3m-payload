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
 *   auto              - run the full automated test sequence below
 *   help              - show this list
 *
 * "auto" runs: CAPTURE (all) -> REQUEST 0/1/2 -> REQUEST <invalid id> ->
 * CAPTURE <invalid id>, checking that each behaves as designed (a real
 * image frame comes back for a connected+captured camera, and a clean
 * *_ERROR:UNKNOWN_ID status comes back for an invalid one), and prints a
 * pass/fail/skip summary.
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

bool rpiIdle = false;

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

// Read and print one framed message from the Pi. Returns false on a hard
// read error (recvFramedFromPi already logged the reason via radioPrintln).
bool receiveAndReport(FrameResult &out)
{
  uint32_t rxLen = 0;
  bool isStatus = false;
  if (!recvFramedFromPi(Serial2, frameBuf, MAX_FRAME, rxLen, isStatus))
  {
    return false;
  }

  out.isStatus = isStatus;

  if (isStatus)
  {
    // Strip the "STATUS:" prefix, same as satellite_teensy.ino's handleStatusPayload
    const size_t L = 7;
    String msg;
    for (uint32_t i = L; i < rxLen; ++i)
    {
      char c = (char)frameBuf[i];
      if (c == '\r' || c == '\n')
        continue;
      msg += c;
    }
    if (msg.length() == 0)
      msg = "(empty)";
    out.statusMsg = msg;

    Serial.print("  [STATUS] ");
    Serial.println(msg);

    if (msg == "IDLE")
      rpiIdle = true;
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

/**
 * Send an ASCII command line to the Pi, then read and print frames until an
 * IDLE status arrives (the Pi's standard "command complete, ready for next"
 * signal - see multicam_controller.py) or timeout_ms elapses.
 *
 * Reports back:
 *   dataFrames - count of non-status (image) frames received
 *   sawError   - true if any STATUS text contained "ERROR" or ended in
 *                "_FAIL" (e.g. CAPTURE_ERROR, REQUEST_FAIL)
 *   lastMsg    - the last STATUS text seen (useful for inspecting *why*)
 * Returns false on timeout or a frame read error.
 */
bool sendCommandAndWaitIdle(const String &cmd, uint32_t timeout_ms,
                             uint32_t &dataFrames, bool &sawError, String &lastMsg)
{
  dataFrames = 0;
  sawError = false;
  lastMsg = "";
  rpiIdle = false;

  while (Serial2.available())
    Serial2.read();

  Serial.print("> ");
  Serial.print(cmd);
  Serial2.print(cmd);
  Serial2.flush();

  uint32_t start = millis();
  while (!rpiIdle)
  {
    if (millis() - start > timeout_ms)
    {
      Serial.println("  [TIMEOUT] no IDLE status within window");
      return false;
    }

    if (Serial2.available() < UART_HEADER_SIZE)
    {
      delay(1);
      continue;
    }

    FrameResult r;
    if (!receiveAndReport(r))
    {
      Serial.println("  [ERROR] frame read failed");
      return false;
    }

    if (r.isStatus)
    {
      if (r.statusMsg != "IDLE")
        lastMsg = r.statusMsg;
      if (r.statusMsg.indexOf("ERROR") >= 0 || r.statusMsg.indexOf("FAIL") >= 0)
        sawError = true;
    }
    else
    {
      dataFrames++;
    }
  }

  return true;
}

void printHelp()
{
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  capture           - capture from all connected cameras");
  Serial.println("  capture <id>      - capture from one camera (0=lepton,1=boson,2=rpicam)");
  Serial.println("  request <id>      - fetch most recent capture for camera <id>");
  Serial.println("  auto              - run automated capture/request/error-path test suite");
  Serial.println("  help              - show this message");
  Serial.println();
}

void runAutoTest()
{
  Serial.println();
  Serial.println("=== AUTO TEST: RPi <-> Teensy UART Link ===");

  int passed = 0;
  int total = 0;
  uint32_t dataFrames;
  bool sawError;
  String lastMsg;

  // 1) Capture from every connected camera
  total++;
  Serial.println("\n[1] CAPTURE (all cameras)");
  bool ok = sendCommandAndWaitIdle("CAPTURE\n", 30000, dataFrames, sawError, lastMsg);
  bool captureOk = ok && !sawError;
  Serial.println(captureOk ? "  PASS" : "  FAIL");
  if (captureOk)
    passed++;

  // 2) Request each known camera ID's most recent capture
  const char *names[3] = {"lepton (id 0)", "boson (id 1)", "rpicam (id 2)"};
  for (int id = 0; id < 3; id++)
  {
    total++;
    Serial.print("\n[REQUEST ");
    Serial.print(id);
    Serial.print("] ");
    Serial.println(names[id]);

    String cmd = "REQUEST " + String(id) + "\n";
    ok = sendCommandAndWaitIdle(cmd, 15000, dataFrames, sawError, lastMsg);

    if (!ok)
    {
      Serial.println("  FAIL (timeout/read error)");
    }
    else if (sawError && lastMsg.indexOf("NO_CAPTURE") >= 0)
    {
      Serial.println("  SKIP (no capture queued for this camera - not connected?)");
      total--; // don't penalize a camera that simply isn't present on this bench
    }
    else if (sawError)
    {
      Serial.println("  FAIL (" + lastMsg + ")");
    }
    else if (dataFrames != 1)
    {
      Serial.println("  FAIL (expected 1 image frame, got " + String(dataFrames) + ")");
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
  ok = sendCommandAndWaitIdle("REQUEST 99\n", 10000, dataFrames, sawError, lastMsg);
  bool errPass = ok && sawError && lastMsg.indexOf("UNKNOWN_ID") >= 0;
  Serial.println(errPass ? "  PASS (rejected as expected)" : "  FAIL");
  if (errPass)
    passed++;

  total++;
  Serial.println("\n[ERROR CHECK] CAPTURE invalid id 99");
  ok = sendCommandAndWaitIdle("CAPTURE 99\n", 10000, dataFrames, sawError, lastMsg);
  errPass = ok && sawError && lastMsg.indexOf("UNKNOWN_ID") >= 0;
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

void handleCommand(const String &line)
{
  String upper = line;
  upper.toUpperCase();

  uint32_t dataFrames;
  bool sawError;
  String lastMsg;

  if (upper == "HELP" || upper == "?")
  {
    printHelp();
  }
  else if (upper == "AUTO")
  {
    runAutoTest();
  }
  else if (upper.startsWith("CAPTURE"))
  {
    sendCommandAndWaitIdle(line + "\n", 30000, dataFrames, sawError, lastMsg);
  }
  else if (upper.startsWith("REQUEST"))
  {
    sendCommandAndWaitIdle(line + "\n", 15000, dataFrames, sawError, lastMsg);
  }
  else
  {
    Serial.println("Unknown command. Type 'help'.");
  }
}

void setup()
{
  Serial.begin(115200);
  uint32_t bootStart = millis();
  while (!Serial && millis() - bootStart < 3000)
  {
  }

  Serial2.begin(UART_BAUD);

  Serial.println("=== RPi <-> Satellite Teensy UART Link Test Harness ===");
  Serial.println("Listening for RPi boot status on Serial2...");
  printHelp();
}

void loop()
{
  // Passively drain and print any unsolicited status frames (BOOT,
  // CAM_READY, IDLE, etc.) so boot-time status is visible even before the
  // first command is typed.
  if (Serial2.available() >= UART_HEADER_SIZE)
  {
    FrameResult r;
    receiveAndReport(r); // errors are already logged by recvFramedFromPi
  }

  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
      handleCommand(line);
  }
}
