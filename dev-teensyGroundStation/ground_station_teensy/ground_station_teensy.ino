/*
 * Ground Station Command Interpreter
 * A comprehensive command interpreter system for Arduino with serial communication
 *
 * Features:
 * - Serial command parsing
 * - Command history
 * - Help system
 * - Error handling
 * - Extensible command structure
 * - Version management
 *
 * Version: 1.0.0
 * Build Date: 2024-01-15
 * Author: Artemis CubeSat Team
 */

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF22.h>
#include <RHHardwareSPI1.h>

// Version information
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_BUILD "2025-09-10"
#define VERSION_STRING "1.0.0"
#define BUILD_INFO "Arduino Teensy 4.1 Ground Station Command Interpreter"

// Command structure definition
struct Command
{
    const char *name;
    const char *description;
    void (*function)(const char *args);
};

// Radio configuration pins and object
const int RADIO_CS = 38;  // Chip select pin for RF22 module
const int RADIO_INT = 40; // Interrupt pin for RF22 module
RH_RF22 rf23(RADIO_CS, RADIO_INT, hardware_spi1);

// Image reception buffer and tracking variables
const uint32_t MAX_IMG = 40000; // Maximum image buffer size (40KB)
uint8_t imgBuffer[MAX_IMG];     // Buffer to store received thermal image data
uint16_t expectedLength = 0;    // Expected total image size from header
uint16_t expectedPackets = 0;   // Total number of packets expected
uint16_t receivedPackets = 0;   // Number of packets successfully received
bool headerReceived = false;    // Flag indicating header packet was received
bool imageComplete = false;     // Flag indicating all data was received

// Packet tracking for duplicate detection and missing packet identification
bool packetReceived[1200]; // Array to track which packets have been received
uint16_t maxPacketNum = 0; // Highest packet number received

// Communication statistics
unsigned long packetsReceived = 0; // Total packets received (including duplicates)
unsigned long lastPacketTime = 0;  // Timestamp of last packet reception
int lastRSSI = 0;                  // Signal strength of last received packet

// Auto mode flag for continuous reception
bool autoMode = false;

// Packet size configuration
const uint8_t PACKET_DATA_SIZE = 45; // Data payload size per packet

// Serial message radio reception parameters
const uint8_t SERIAL_MSG_TYPE = 0xAA; // Message type identifier for serial output

// Global variables
String inputBuffer = "";
String commandHistory[10];
int historyIndex = 0;
bool commandComplete = false;
bool serialConnected = false;
bool interruptRequested = false;
const int RASPBERRY_PI_GPIO_PIN = 36;

// Forward declarations
void parseCommand(const String &input);
void executeCommand(const char *cmd, const char *args);
void printPrompt();
void addToHistory(const String &command);
void showHistory();
void clearHistory();
void checkForInterrupt();
void resetInterrupt();
bool isInterruptRequested();

// Radio function declarations
void initRadio();
void clearReception();
void handlePacket();
void processPacket(uint8_t *buf, uint8_t len);
void handleHeaderPacket(uint8_t *buf);
void handleEndPacket(uint8_t *buf);
void handleDataPacket(uint8_t *buf, uint8_t len);
void handleSerialMessage(uint8_t *buf, uint8_t len);
void runAutoMode();
void showReceptionSummary();
void exportThermalData();
void forwardToSatellite(char cmd);

// Command function prototypes
void cmdHelp(const char *args);
void cmdVersion(const char *args);
void cmdStatus(const char *args);
void cmdGSPing(const char *args);
void cmdEcho(const char *args);
void cmdLed(const char *args);
void cmdAnalog(const char *args);
void cmdDigital(const char *args);
void cmdTime(const char *args);
void cmdReset(const char *args);
void cmdHistory(const char *args);
void cmdClear(const char *args);
void cmdRPIControl(const char *args);
void cmdTestInterrupt(const char *args);
void cmdRadio(const char *args);
void cmdAutoMode(const char *args);
void cmdExport(const char *args);
void cmdCapture(const char *args);
void cmdRequest(const char *args);
void cmdRadioStatus(const char *args);

// Command table - easily extensible
const Command commands[] = {
    {"help", "Show available commands", cmdHelp},
    {"version", "Show GS version information", cmdVersion},
    {"status", "Show system status", cmdStatus},
    {"ping", "Test GS communication", cmdGSPing},
    {"echo", "Echo back the arguments", cmdEcho},
    {"led", "Control LED (on/off/toggle)", cmdLed},
    {"analog", "Read analog pin (analog <pin>)", cmdAnalog},
    {"digital", "Read/write digital pin (digital <pin> [value])", cmdDigital},
    {"time", "Show uptime", cmdTime},
    {"reset", "Reset the system", cmdReset},
    {"history", "Show command history", cmdHistory},
    {"clear", "Clear screen", cmdClear},
    {"rpi", "Control Raspberry Pi (rpi <on|off|status>)", cmdRPIControl},
    {"testint", "Test interrupt functionality (testint <seconds>)", cmdTestInterrupt},
    {"radio", "Radio control (radio <init|status|tx|rx>)", cmdRadio},
    {"auto", "Start auto reception mode", cmdAutoMode},
    {"export", "Export thermal data as CSV", cmdExport},
    {"capture", "Command satellite to capture thermal data", cmdCapture},
    {"request", "Request thermal data downlink from satellite", cmdRequest},
    {"rstatus", "Show radio reception status", cmdRadioStatus}};

const int numCommands = sizeof(commands) / sizeof(commands[0]);

void setup()
{

    // Initialize serial communication
    Serial.begin(115200);

    // Wait for serial port to connect (optional for some boards)
    while (!Serial)
    {
        delay(10);
    }

    // Mark serial as connected and clear any existing buffer
    serialConnected = true;
    inputBuffer = "";

    // Initialize built-in LED to off.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Configure the Raspberry Pi control pin
    pinMode(RASPBERRY_PI_GPIO_PIN, OUTPUT);
    // Set Raspberry Pi to off state initially
    digitalWrite(RASPBERRY_PI_GPIO_PIN, LOW);

    // Initialize radio module
    initRadio();
    
    // Initialize reception state
    clearReception();

    // Clear command history
    clearHistory();

    // Welcome message with version info
    Serial.println("\n================================================");
    Serial.println("    Artemis Ground Station Command Interpreter");
    Serial.println("================================================");
    Serial.print("GS Version: ");
    Serial.println(VERSION_STRING);
    Serial.print("Build Date: ");
    Serial.println(VERSION_BUILD);
    Serial.print("Build Info: ");
    Serial.println(BUILD_INFO);
    Serial.println("================================================");
    Serial.println("Type 'help' for available commands");
    Serial.println("Type 'version' for detailed version info");
    Serial.println("Type 'clear' to clear the screen");
    Serial.println("Type 'auto' to start thermal image reception");
    Serial.println("Type 'Q' during command execution to interrupt");
    Serial.println("================================================\n");

    printPrompt();
}

void loop()
{
    // Check for serial connection state changes
    bool currentlyConnected = Serial;

    // Handle serial reconnection
    if (!serialConnected && currentlyConnected)
    {
        Serial.println("\nSerial connection restored.");
        printPrompt();
    }
    // Handle serial disconnection
    else if (serialConnected && !currentlyConnected)
    {
        // Serial disconnected - no action needed
    }

    serialConnected = currentlyConnected;

    // Check for interrupt character (Q) at any time
    checkForInterrupt();

    // Check for incoming radio packets (always listen for serial messages)
    if (rf23.available())
    {
        handlePacket();
    }

    // Read entire line when available
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove whitespace and newlines

        if (input.length() > 0)
        {
            // Reset interrupt flag before processing new command
            resetInterrupt();
            parseCommand(input);
            addToHistory(input);
            printPrompt();
        }
    }
}

void parseCommand(const String &input)
{
    // Trim whitespace
    String trimmed = input;
    trimmed.trim();

    if (trimmed.length() == 0)
    {
        return;
    }

    // Find space to separate command and arguments
    int spaceIndex = trimmed.indexOf(' ');
    String command;
    String args;

    if (spaceIndex == -1)
    {
        command = trimmed;
        args = "";
    }
    else
    {
        command = trimmed.substring(0, spaceIndex);
        args = trimmed.substring(spaceIndex + 1);
    }

    // Convert to lowercase for case-insensitive matching
    command.toLowerCase();

    // Execute the command
    executeCommand(command.c_str(), args.c_str());
}

void executeCommand(const char *cmd, const char *args)
{
    // Search for command in command table
    for (int i = 0; i < numCommands; i++)
    {
        if (strcmp(cmd, commands[i].name) == 0)
        {
            commands[i].function(args);
            return;
        }
    }

    // Command not found
    Serial.print("Error: Unknown command '");
    Serial.print(cmd);
    Serial.println("'");
    Serial.println("Type 'help' for available commands");
}

void printPrompt()
{
    Serial.print("GS> ");
}

void addToHistory(const String &command)
{
    // Shift history array
    for (int i = 9; i > 0; i--)
    {
        commandHistory[i] = commandHistory[i - 1];
    }
    commandHistory[0] = command;

    if (historyIndex < 10)
    {
        historyIndex++;
    }
}

void clearHistory()
{
    for (int i = 0; i < 10; i++)
    {
        commandHistory[i] = "";
    }
    historyIndex = 0;
}

void checkForInterrupt()
{
    // Check for interrupt character (Q) without blocking
    if (Serial.available())
    {
        char c = Serial.peek(); // Look at next character without consuming it
        if (c == 'Q' || c == 'q')
        {
            // Consume the interrupt character
            Serial.read();
            interruptRequested = true;
            Serial.println("\n*** INTERRUPT REQUESTED ***");
            Serial.println("Returning to command prompt...");
            printPrompt();
        }
    }
}

// Function to check for interrupt during command execution
bool isInterruptRequested()
{
    // Check for Q key during command execution
    if (Serial.available())
    {
        char c = Serial.peek();
        if (c == 'Q' || c == 'q')
        {
            Serial.read(); // Consume the Q
            interruptRequested = true;
            Serial.println("\n*** INTERRUPT REQUESTED ***");
            return true;
        }
    }
    return interruptRequested;
}

void resetInterrupt()
{
    interruptRequested = false;
}

// Radio function implementations
void initRadio()
{
  Serial.println("Initializing radio...");

  // Configure RX/TX control pins
  pinMode(30, OUTPUT);    // RX_ON pin
  pinMode(31, OUTPUT);    // TX_ON pin
  digitalWrite(30, HIGH); // RX_ON = HIGH for receive mode
  digitalWrite(31, LOW);  // TX_ON = LOW for receive mode
  delay(100);

  // Configure SPI1 interface for radio communication
  SPI1.setMISO(39); // Master In, Slave Out
  SPI1.setMOSI(26); // Master Out, Slave In
  SPI1.setSCK(27);  // Serial Clock

  // Initialize RF22 radio module
  if (!rf23.init())
  {
    Serial.println("Radio init failed!");
    while (1)
      ; // Halt if radio initialization fails
  }

  // Configure radio parameters
  rf23.setFrequency(433.0);                     // Set frequency to 433MHz
  rf23.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45); // GFSK modulation, 9.6kbps
  rf23.setTxPower(RH_RF22_TXPOW_20DBM);         // Set transmit power to 20dBm
  rf23.setModeIdle();                           // Set radio to idle mode
  delay(100);
  Serial.println("Radio ready");
}

void clearReception()
{
  headerReceived = false;
  imageComplete = false;
  expectedLength = 0;
  expectedPackets = 0;
  receivedPackets = 0;
  maxPacketNum = 0;
  memset(packetReceived, false, sizeof(packetReceived)); // Clear packet tracking array
}

void handlePacket()
{
  uint8_t buf[64]; // Buffer for incoming packet data
  uint8_t len = sizeof(buf);

  if (rf23.recv(buf, &len))
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate packet reception
    lastPacketTime = millis();
    lastRSSI = rf23.lastRssi(); // Store signal strength
    packetsReceived++;
    processPacket(buf, len); // Process the received packet
    digitalWrite(LED_BUILTIN, LOW);   // Turn off LED
  }
}

void processPacket(uint8_t *buf, uint8_t len)
{
  // Check for serial message packet (MSG_TYPE + LENGTH + MESSAGE_DATA)
  if (len >= 2 && buf[0] == SERIAL_MSG_TYPE)
  {
    handleSerialMessage(buf, len);
    return;
  }

  // Check for header packet (10 bytes with magic bytes)
  if (len == 10 && buf[0] == 0xFF && buf[1] == 0xFF)
  {
    handleHeaderPacket(buf);
    return;
  }

  // Check for end packet (6 bytes with magic bytes)
  if (len == 6 && buf[0] == 0xEE && buf[1] == 0xEE)
  {
    handleEndPacket(buf);
    return;
  }

  // Check for data packet (requires valid header and incomplete image)
  if (len >= 3 && headerReceived && !imageComplete)
  {
    handleDataPacket(buf, len);
    return;
  }
}

void handleHeaderPacket(uint8_t *buf)
{
  // Extract image size and packet count (little-endian format)
  expectedLength = buf[2] | (buf[3] << 8);
  expectedPackets = buf[4] | (buf[5] << 8);

  Serial.println("\n📦 THERMAL IMAGE HEADER RECEIVED!");
  Serial.print("Expected size: ");
  Serial.print(expectedLength);
  Serial.println(" bytes");
  Serial.print("Expected packets: ");
  Serial.println(expectedPackets);

  // Validate magic bytes (0xDE 0xAD 0xBE 0xEF)
  if (buf[6] == 0xDE && buf[7] == 0xAD && buf[8] == 0xBE && buf[9] == 0xEF)
  {
    Serial.println("✓ Header valid - receiving thermal data...");
    headerReceived = true;
    imageComplete = false;
    receivedPackets = 0;
    maxPacketNum = 0;
    memset(packetReceived, false, sizeof(packetReceived)); // Reset packet tracking
  }
  else
  {
    Serial.println("✗ Invalid header!");
  }
}

void handleEndPacket(uint8_t *buf)
{
  uint16_t finalCount = buf[2] | (buf[3] << 8); // Extract final packet count
  Serial.println("\n🏁 END PACKET RECEIVED!");
  Serial.print("Transmitter sent ");
  Serial.print(finalCount);
  Serial.println(" packets");

  imageComplete = true;
  autoMode = false; // Exit auto mode
  showReceptionSummary();
}

void handleDataPacket(uint8_t *buf, uint8_t len)
{
  if (len < 3)
    return; // Minimum packet size check

  // Extract packet number and data length
  uint16_t packetNum = buf[0] | (buf[1] << 8); // Little-endian packet number
  uint8_t dataLen = len - 2;                   // Data length excludes packet number bytes

  if (packetNum >= 1200)
    return; // Bounds check for packet number

  // Process packet only if not already received (duplicate detection)
  if (!packetReceived[packetNum])
  {
    uint32_t bufferPos = (uint32_t)packetNum * PACKET_DATA_SIZE; // Calculate buffer position
    if (bufferPos + dataLen <= MAX_IMG)
    {                                                  // Buffer overflow protection
      memcpy(&imgBuffer[bufferPos], &buf[2], dataLen); // Copy data to buffer
      packetReceived[packetNum] = true;                // Mark packet as received
      receivedPackets++;

      if (packetNum > maxPacketNum)
      {
        maxPacketNum = packetNum; // Track highest packet number
      }

      // Progress indication in auto mode
      if (autoMode && receivedPackets % 10 == 0)
      {
        Serial.print(".");
        if (receivedPackets % 100 == 0)
        {
          float progress = (float)receivedPackets / expectedPackets * 100;
          Serial.print(" ");
          Serial.print(progress, 0);
          Serial.println("%");
        }
      }
    }
  }
}

void handleSerialMessage(uint8_t *buf, uint8_t len)
{
  if (len < 2)
    return; // Minimum packet size check

  uint8_t msgLen = buf[1]; // Message length from second byte
  
  if (msgLen == 0 || len < msgLen + 2)
    return; // Invalid message length or packet too short

  // Extract message data and print to serial
  String message = "";
  for (uint8_t i = 0; i < msgLen; i++)
  {
    message += (char)buf[2 + i];
  }
  
  // Print the message to serial (this is the satellite's serial output)
  Serial.print(message);
}

void runAutoMode()
{
  Serial.println("\n=== AUTO MODE - THERMAL IMAGE RECEPTION ===");
  Serial.println("Clearing buffer and waiting for transmission...");
  Serial.println("Start transmission from flat sat now!");
  Serial.println("Press any key to abort\n");

  clearReception(); // Reset reception state
  autoMode = true;  // Enable auto mode
  rf23.setModeRx(); // Ensure radio is in receive mode

  unsigned long lastActivity = millis();
  while (!Serial.available() && !imageComplete)
  {
    if (rf23.available())
    {
      handlePacket();
      lastActivity = millis(); // Update activity timestamp
    }

    // Timeout if no activity for 30 seconds after header received
    if (headerReceived && (millis() - lastActivity > 30000))
    {
      Serial.println("\n\nTimeout - no packets for 30 seconds");
      break;
    }
    delay(1);
  }

  // Clear any key press from input buffer
  while (Serial.available())
    Serial.read();
  autoMode = false; // Exit auto mode if end packet not received

  // Handle different completion scenarios
  if (!imageComplete && !headerReceived)
  {
    Serial.println("\nNo transmission detected");
  }
  else if (!imageComplete)
  {
    Serial.println("\nTransmission interrupted");
    showReceptionSummary();
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
  Serial.print((float)receivedPackets / expectedPackets * 100, 1);
  Serial.println("%)");

  // Quality assessment based on reception rate
  if (receivedPackets == expectedPackets)
  {
    Serial.println("\n✅ PERFECT RECEPTION!");
  }
  else if (receivedPackets > expectedPackets * 0.95)
  {
    Serial.println("\n✅ Excellent reception!");
  }
  else if (receivedPackets > expectedPackets * 0.80)
  {
    Serial.println("\n⚠️ Good reception");
  }
  else
  {
    Serial.println("\n❌ Poor reception");
  }

  // Indicate if thermal image is ready for export
  if (expectedLength == 38400)
  {
    Serial.println("\nThermal image ready! Type 'export' to export as CSV");
  }
}

void exportThermalData()
{
  if (!headerReceived || expectedLength != 38400)
  {
    Serial.println("No complete thermal image to export");
    return;
  }

  Serial.println("\n--- EXPORTING THERMAL DATA ---");
  Serial.println("Copy data below to 'thermal_image.csv'");
  Serial.println("=== START CSV ===");

  // Export thermal data as CSV (120x160 pixel grid)
  for (int row = 0; row < 120; row++)
  {
    for (int col = 0; col < 160; col++)
    {
      int idx = (row * 160 + col) * 2; // 2 bytes per pixel
      if (idx < MAX_IMG - 1)
      {
        // Convert raw 16-bit value to temperature in Celsius
        uint16_t pixel = imgBuffer[idx] | (imgBuffer[idx + 1] << 8);
        if (pixel >= 27315 && pixel <= 37315)
        {                                        // Valid temperature range (0-100°C)
          float tempC = (pixel - 27315) / 100.0; // Convert from Kelvin*100 to Celsius
          Serial.print(tempC, 2);
        }
        else
        {
          Serial.print("NaN"); // Invalid temperature value
        }
      }
      else
      {
        Serial.print("NaN"); // Buffer overflow protection
      }
      if (col < 159)
        Serial.print(","); // CSV separator
    }
    Serial.println(); // New line for each row
  }

  Serial.println("=== END CSV ===");
  Serial.println("\nVisualize with Python:");
  Serial.println(" import numpy as np");
  Serial.println(" import matplotlib.pyplot as plt");
  Serial.println(" data = np.loadtxt('thermal_image.csv', delimiter=',')");
  Serial.println(" plt.imshow(data, cmap='hot')");
  Serial.println(" plt.colorbar(label='Temperature (°C)')");
  Serial.println(" plt.show()");
}

void forwardToSatellite(char cmd)
{
  if (cmd == 'u')
  {
    Serial.println("\n--- UART THERMAL CAPTURE ---");
  }
  else if (cmd == 'r')
  {
    Serial.println("\n--- REQUESTING THERMAL DATA DOWNLINK ---");
  }
  else
  {
    return; // Ignore unknown commands
  }

  Serial.println("Forwarding command to satellite...");
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED during transmission

  // Ensure radio is in transmit mode
  rf23.setModeTx();

  // Send the command
  if (!rf23.send((uint8_t *)&cmd, 1))
  { // a command should be a single byte
    Serial.print("Failed to queue command for transmission: ");
    Serial.println(cmd);
  }
  else
  {
    if (!rf23.waitPacketSent(500))
    {
      Serial.print("Failed to send command: ");
      Serial.println(cmd);
    }
    else
    {
      Serial.print("Command sent successfully: ");
      Serial.println(cmd);
    }
  }

  digitalWrite(LED_BUILTIN, LOW); // Turn off LED
  rf23.setModeIdle();    // Return to idle mode
  return;
}

// Command implementations
void cmdHelp(const char *args)
{
    Serial.println("\nAvailable Commands:");
    Serial.println("==================");

    for (int i = 0; i < numCommands; i++)
    {
        Serial.print("  ");
        Serial.print(commands[i].name);
        Serial.print(" - ");
        Serial.println(commands[i].description);
    }

    Serial.println("\nUsage: command [arguments]");
    Serial.println("Example: led on, analog 0, digital 13 1");
    Serial.println("\nInterrupt: Press 'Q' during command execution to interrupt and return to prompt");
}

void cmdVersion(const char *args)
{
    Serial.println("\n=== GS Version Information ===");
    Serial.print("Software: ");
    Serial.println(BUILD_INFO);
    Serial.print("GS Version: ");
    Serial.print(VERSION_MAJOR);
    Serial.print(".");
    Serial.print(VERSION_MINOR);
    Serial.print(".");
    Serial.println(VERSION_PATCH);
    Serial.print("Build Date: ");
    Serial.println(VERSION_BUILD);
    Serial.print("GS Arduino Version: ");
    Serial.println(ARDUINO);
    Serial.print("Board: Teensy 4.1");
    Serial.println(F_CPU);
    Serial.println("========================");
}

void cmdStatus(const char *args)
{
    Serial.println("\n=== GS System Status ===");
    cmdTime(args);

    Serial.print("GS LED Status: ");
    Serial.println(digitalRead(LED_BUILTIN) ? "ON" : "OFF");

    Serial.print("Commands in history: ");
    Serial.println(historyIndex);
}

void cmdGSPing(const char *args)
{
    Serial.println("GS Teensy 4.1: PONG");
}

void cmdEcho(const char *args)
{
    if (strlen(args) > 0)
    {
        Serial.print("Echo: ");
        Serial.println(args);
    }
    else
    {
        Serial.println("Usage: echo <message>");
    }
}

void cmdLed(const char *args)
{
    String argStr = String(args);
    argStr.toLowerCase();

    if (argStr == "on")
    {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("LED turned ON");
    }
    else if (argStr == "off")
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("LED turned OFF");
    }
    else if (argStr == "toggle")
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.print("LED toggled to ");
        Serial.println(digitalRead(LED_BUILTIN) ? "ON" : "OFF");
    }
    else
    {
        Serial.println("Usage: led [on|off|toggle]");
    }
}

void cmdAnalog(const char *args)
{
    if (strlen(args) == 0)
    {
        Serial.println("Usage: analog <pin>");
        return;
    }

    int pin = atoi(args);
    if (pin < 0 || pin > 15)
    { // Adjust based on your board
        Serial.println("Error: Invalid pin number (0-15)");
        return;
    }

    int value = analogRead(pin);
    float voltage = (value * 5.0) / 1024.0; // Assuming 5V reference

    Serial.print("Analog pin ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" (");
    Serial.print(voltage, 2);
    Serial.println("V)");
}

void cmdDigital(const char *args)
{
    String argStr = String(args);
    int spaceIndex = argStr.indexOf(' ');

    if (spaceIndex == -1)
    {
        Serial.println("Usage: digital <pin> [value]");
        return;
    }

    int pin = argStr.substring(0, spaceIndex).toInt();
    String valueStr = argStr.substring(spaceIndex + 1);

    if (pin < 0 || pin > 13)
    { // Adjust based on your board
        Serial.println("Error: Invalid pin number (0-13)");
        return;
    }

    if (valueStr.length() == 0)
    {
        // Read mode
        pinMode(pin, INPUT);
        int value = digitalRead(pin);
        Serial.print("Digital pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(value ? "HIGH" : "LOW");
    }
    else
    {
        // Write mode
        pinMode(pin, OUTPUT);
        int value = valueStr.toInt();
        digitalWrite(pin, value);
        Serial.print("Digital pin ");
        Serial.print(pin);
        Serial.print(" set to ");
        Serial.println(value ? "HIGH" : "LOW");
    }
}

void cmdTime(const char *args)
{
    unsigned long uptime = millis();
    unsigned long seconds = uptime / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;

    seconds = seconds % 60;
    minutes = minutes % 60;

    Serial.print("GS Uptime: ");
    if (hours > 0)
    {
        Serial.print(hours);
        Serial.print("h ");
    }
    if (minutes > 0)
    {
        Serial.print(minutes);
        Serial.print("m ");
    }
    Serial.print(seconds);
    Serial.println("s");
}

void cmdReset(const char *args)
{
    Serial.println("Resetting GS system... manually reconnect to serial after 20 seconds.");
    Serial.println("Press 'Q' to cancel reset within 1 second...");

    // Check for interrupt during the 1-second delay
    unsigned long startTime = millis();
    while (millis() - startTime < 1000)
    {
        if (isInterruptRequested())
        {
            Serial.println("\nReset cancelled by user!");
            return;
        }
        delay(50);
    }

    // Software reset for Teensy 4.1 (ARM Cortex-M7)
    SCB_AIRCR = 0x05FA0004;
}

void cmdHistory(const char *args)
{
    Serial.println("\nGS Command History:");
    Serial.println("================");

    for (int i = 0; i < historyIndex && i < 10; i++)
    {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(commandHistory[i]);
    }

    if (historyIndex == 0)
    {
        Serial.println("No commands in history");
    }
}

void cmdClear(const char *args)
{
    // Clear screen by printing newlines
    for (int i = 0; i < 50; i++)
    {
        Serial.println();
    }

    // Re-print welcome message with version
    Serial.println("================================================");
    Serial.println("    Artemis Ground Station Command Interpreter");
    Serial.println("================================================");
    Serial.print("GS Version: ");
    Serial.println(VERSION_STRING);
    Serial.print("Build Date: ");
    Serial.println(VERSION_BUILD);
    Serial.println("================================================\n");
}

void cmdRPIControl(const char *args)
{
    String argStr = String(args);
    argStr.toLowerCase();

    if (argStr == "on")
    {
        digitalWrite(RASPBERRY_PI_GPIO_PIN, HIGH);
        Serial.println("RPI turned ON");
    }
    else if (argStr == "off")
    {
        digitalWrite(RASPBERRY_PI_GPIO_PIN, LOW);
        Serial.println("RPI turned OFF");
    }
    else if (argStr == "status")
    {
        Serial.print("RPI Status: ");
        Serial.println(digitalRead(RASPBERRY_PI_GPIO_PIN));
    }
    else
    {
        Serial.println("Usage: rpi [on|off|status]");
    }
}

void cmdTestInterrupt(const char *args)
{
    if (strlen(args) == 0)
    {
        Serial.println("Usage: testint <seconds>");
        Serial.println("Example: testint 10 (runs for 10 seconds, press Q to interrupt)");
        return;
    }

    int duration = atoi(args);
    if (duration <= 0 || duration > 60)
    {
        Serial.println("Error: Duration must be between 1 and 60 seconds");
        return;
    }

    Serial.print("Starting test for ");
    Serial.print(duration);
    Serial.println(" seconds...");
    Serial.println("Press 'Q' at any time to interrupt and return to prompt");

    unsigned long startTime = millis();
    unsigned long endTime = startTime + (duration * 1000);

    while (millis() < endTime)
    {
        // Check for interrupt every 100ms
        if (isInterruptRequested())
        {
            Serial.println("\nTest interrupted by user!");
            return;
        }

        // Show progress every second
        unsigned long elapsed = (millis() - startTime) / 1000;
        static unsigned long lastPrint = 0;
        if (elapsed != lastPrint)
        {
            Serial.print("Test running... ");
            Serial.print(elapsed);
            Serial.print("/");
            Serial.print(duration);
            Serial.println(" seconds");
            lastPrint = elapsed;
        }

        delay(100); // Small delay to allow interrupt checking
    }

    Serial.println("Test completed successfully!");
}

// Radio command implementations
void cmdRadio(const char *args)
{
    String argStr = String(args);
    argStr.toLowerCase();

    if (argStr == "init")
    {
        initRadio();
    }
    else if (argStr == "status")
    {
        Serial.println("\n=== RADIO STATUS ===");
        Serial.print("Frequency: 433.0 MHz");
        Serial.println();
        Serial.print("Last RSSI: ");
        Serial.println(lastRSSI);
        Serial.print("Packets received: ");
        Serial.println(packetsReceived);
        Serial.print("Auto mode: ");
        Serial.println(autoMode ? "ON" : "OFF");
        Serial.print("Header received: ");
        Serial.println(headerReceived ? "YES" : "NO");
        Serial.print("Image complete: ");
        Serial.println(imageComplete ? "YES" : "NO");
        if (headerReceived)
        {
            Serial.print("Expected packets: ");
            Serial.println(expectedPackets);
            Serial.print("Received packets: ");
            Serial.println(receivedPackets);
        }
    }
    else if (argStr == "tx")
    {
        rf23.setModeTx();
        Serial.println("Radio set to transmit mode");
    }
    else if (argStr == "rx")
    {
        rf23.setModeRx();
        Serial.println("Radio set to receive mode");
    }
    else
    {
        Serial.println("Usage: radio [init|status|tx|rx]");
    }
}

void cmdAutoMode(const char *args)
{
    runAutoMode();
}

void cmdExport(const char *args)
{
    exportThermalData();
}

void cmdCapture(const char *args)
{
    forwardToSatellite('u');
}

void cmdRequest(const char *args)
{
    forwardToSatellite('r');
}

void cmdRadioStatus(const char *args)
{
    showReceptionSummary();
}
