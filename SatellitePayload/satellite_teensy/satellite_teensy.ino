/**
 * Satellite Payload Transmitter for EPSCOR C3M Project
 *
 * This Arduino sketch runs on a Teensy microcontroller in the satellite payload
 * to capture thermal images from a Raspberry Pi via UART and transmit them
 * to the ground station via radio communication.
 *
 * Key Features:
 * - Receives thermal image data from RPi via UART at 115200 baud
 * - Transmits image data via RF22 radio module in packetized format
 * - Handles reliable packet transmission with retry logic
 * - Provides transmission statistics and quality assessment
 * - Supports both capture and transmission operations
 *
 * Hardware Requirements:
 * - Teensy microcontroller
 * - RF22 radio module (433MHz)
 * - Raspberry Pi with thermal camera
 * - UART connection between Teensy and RPi
 *
 * Communication Protocol:
 * - UART: Receives thermal data with header/end markers
 * - Radio: Transmits packetized data with header/data/end packets
 *
 * @author EPSCOR C3M Team
 * @date 8/25/2025
 * @version 1.0
 */

// Simplified Transmitter with UART (Essential Functions Only)
// UART TRANSMITTER CODE - satellite teensy (8/25/2025)
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF22.h>
#include <RHHardwareSPI1.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// Pin definitions for hardware control
const uint8_t RPI_ENABLE = 36; // Power control pin for Raspberry Pi
const uint8_t TRIGGER_PIN = 2; // Trigger pin to signal RPi for capture
const uint8_t LED_PIN = 13;

// UART pins: Serial2 uses pins 7 (RX) and 8 (TX) automatically

// Radio configuration pins and object
const int RADIO_CS = 38;  // Chip select pin for RF22 module
const int RADIO_INT = 40; // Interrupt pin for RF22 module
RH_RF22 rf23(RADIO_CS, RADIO_INT, hardware_spi1);

// Image data storage and tracking
uint16_t capturedImageLength = 0; // Length of captured thermal image

// Buffer for thermal image storage
const uint32_t MAX_IMG = 40000; // Maximum image buffer size (40KB)
uint8_t imgBuf[MAX_IMG];        // Buffer to store thermal image data

// Serial output redirection
String serialBuffer = "";       // Buffer to accumulate serial output
bool radioReady = false;        // Flag to indicate if radio is ready for transmission

// Radio transmission parameters
const uint8_t PACKET_DATA_SIZE = 45; // Data payload size per packet
const uint16_t PACKET_DELAY_MS = 20; // Delay between packet transmissions

// Serial message radio transmission parameters
const uint8_t SERIAL_MSG_TYPE = 0xAA; // Message type identifier for serial output
const uint8_t MAX_SERIAL_MSG_LEN = 60; // Maximum serial message length per packet

// Define the analog input pins for the temperature sensors and their labels (pulled from Artemis Manual AIN# should be 7 pins)
const int temperatureSensorPins[] = {14, 15, 41, 20, 21, 22, 23};
const char *tempereatureSensorLabels[] = {"OBC", "PDU", "Battery Board", "Solar Panel 1", "Solar Panel 2", "Solar Panel 3", "Solar Panel 4"};
const int numTemperatureSensors = 7;

// TMP36 temperature sensor constants
const float MV_PER_DEGREE_F = 1.0;             // 1 mV/°F
const float OFFSET_F = 58.0;                   // 58 mV (58°F) offset in the output voltage
const float MV_PER_ADC_UNIT = 3300.0 / 1024.0; // Assuming 3.3V reference voltage and 10-bit ADC resolution

// Define the I2C address and labels for each current sensor
#define INA219_ADDR_SOLAR1 0x40  // Solar Panel 1
#define INA219_ADDR_SOLAR2 0x41  // Solar Panel 2
#define INA219_ADDR_SOLAR3 0x42  // Solar Panel 3
#define INA219_ADDR_SOLAR4 0x43  // Solar Panel 4
#define INA219_ADDR_BATTERY 0x44 // Battery Board

// Create an array of current sensors and their labels
Adafruit_INA219 currentSensors[] = {
    Adafruit_INA219(INA219_ADDR_SOLAR1),
    Adafruit_INA219(INA219_ADDR_SOLAR2),
    Adafruit_INA219(INA219_ADDR_SOLAR3),
    Adafruit_INA219(INA219_ADDR_SOLAR4),
    Adafruit_INA219(INA219_ADDR_BATTERY)};
const char *currentSensorsLabels[] = {"Solar Panel 1", "Solar Panel 2", "Solar Panel 3", "Solar Panel 4", "Battery Board"};
const int numCurrentSensors = 5;

/**
 * Setup function - initializes the satellite payload transmitter
 *
 * Configures serial communication, GPIO pins, radio module, and UART
 * interface. Powers up the Raspberry Pi and displays available commands.
 */
void setup()
{
  /* Don't need unless debugging.
   Serial.begin(115200);
    while (!Serial && millis() < 5000); */
  
  // Initialize radio first so we can send setup messages
  initRadio();
  
  // Send startup message via radio instead of serial
  radioPrintln("\n=== Artemis Cubesat Teensy 4.1 Flat Sat Transmitter ===");

  initTemperatureSensors();

  initCurrentSensors();

  //TODO: Implement initPDU

  // init the RPI for thermal images
  initRPI();

  radioPrintln("\nWaiting 5 seconds for RPi boot...");
  delay(5000);

  // Perform initial temperature sensor check
  checkTemperatureSensors();

  // Perform initial current sensor check
  checkCurrentSensors();

  //TODO: Implement checkPingPDU to ensure a pong is receivied from PDU

  radioPrintln("\nCommands:");
  radioPrintln(" 'u' - UART thermal capture (fast!)");
  radioPrintln(" 'r' - Send captured image via radio");
  radioPrintln(" 't' - Check temperature sensors");
  radioPrintln(" 'c' - Check current sensors");
  radioPrintln("\nReady!");
}

void initTemperatureSensors()
{
  radioPrintln("Initializing temperature sensors...");

  // Initialize all temperature sensor pins as INPUT
  for (int i = 0; i < numTemperatureSensors; i++)
  {
    pinMode(temperatureSensorPins[i], INPUT);
    radioPrint("  Pin ");
    radioPrint(String(temperatureSensorPins[i]));
    radioPrint(" configured for ");
    radioPrintln(tempereatureSensorLabels[i]);
  }

  radioPrintln("Temperature sensors initialized successfully");
}

/**
 * Reads temperature from a TMP36 sensor and converts to Celsius
 *
 * @param sensorPin The analog pin connected to the TMP36 sensor
 * @return Temperature in Celsius
 */
float readTemperatureCelsius(int sensorPin)
{
  int adcValue = analogRead(sensorPin);
  float voltage = adcValue * MV_PER_ADC_UNIT;

  // Convert voltage to temperature in Fahrenheit and then subtract the offset
  float temperatureF = (voltage - OFFSET_F) / MV_PER_DEGREE_F;

  // Convert temperature to Celsius
  float temperatureC = (temperatureF - 32) * 5 / 9;

  return temperatureC;
}

/**
 * Checks if temperature sensor values are within valid range
 *
 * @return true if all sensors are reading valid temperatures, false otherwise
 */
bool checkTemperatureSensors()
{
  radioPrintln("\n--- TEMPERATURE SENSOR CHECK ---");

  bool allValid = true;
  int validSensors = 0;

  for (int i = 0; i < numTemperatureSensors; i++)
  {
    float temperatureC = readTemperatureCelsius(temperatureSensorPins[i]);

    // Check if temperature is within reasonable range (-40°C to +125°C for TMP36)
    bool isValid = (temperatureC >= -40.0 && temperatureC <= 125.0);

    radioPrint(tempereatureSensorLabels[i]);
    radioPrint(" (Teensy Pin ");
    radioPrint(String(temperatureSensorPins[i]));
    radioPrint("): ");
    radioPrint(String(temperatureC, 2));
    radioPrint("°C ");

    if (isValid)
    {
      radioPrintln("Y");
      validSensors++;
    }
    else
    {
      radioPrintln("N");
      allValid = false;
    }
  }

  radioPrint("Valid sensors: ");
  radioPrint(String(validSensors));
  radioPrint("/");
  radioPrintln(String(numTemperatureSensors));

  if (allValid)
  {
    radioPrintln("✓ All temperature sensors operational");
  }
  else
  {
    radioPrintln("⚠️ Some temperature sensors showing invalid readings");
  }

  return allValid;
}

/**
 * Gets temperature data from all sensors for telemetry transmission
 *
 * @param temperatures Array to store temperature readings (must be at least numTemperatureSensors)
 * @return Number of valid temperature readings
 */
int getValidTemperatureData(float temperatures[])
{
  int validCount = 0;

  for (int i = 0; i < numTemperatureSensors; i++)
  {
    float tempC = readTemperatureCelsius(temperatureSensorPins[i]);
    temperatures[i] = tempC;

    // Count valid readings
    if (tempC >= -40.0 && tempC <= 125.0)
    {
      validCount++;
    }
  }

  return validCount;
}

/**
 * Initializes all INA219 current sensors and checks their connectivity
 *
 * Configures I2C communication and calibrates each sensor for 16V/400mA range.
 * Reports which sensors are connected and operational.
 */
void initCurrentSensors()
{
  radioPrintln("Initializing current sensors...");

  // Initialize I2C communication
  Wire2.begin();

  int connectedSensors = 0;

  // Initialize and configure each INA219 sensor
  for (int i = 0; i < numCurrentSensors; i++)
  {
    bool sensorConnected = currentSensors[i].begin(&Wire2);

    if (sensorConnected)
    {
      radioPrint("  ");
      radioPrint(currentSensorsLabels[i]);
      radioPrintln(" - Connected");
      connectedSensors++;

      // Set calibration for 16V and 400mA range
      currentSensors[i].setCalibration_16V_400mA();
    }
    else
    {
      radioPrint("  ");
      radioPrint(currentSensorsLabels[i]);
      radioPrintln(" - Not detected");
    }
  }

  radioPrint("Current sensors initialized: ");
  radioPrint(String(connectedSensors));
  radioPrint("/");
  radioPrintln(String(numCurrentSensors));

  if (connectedSensors == numCurrentSensors)
  {
    radioPrintln("✓ All current sensors operational");
  }
  else if (connectedSensors > 0)
  {
    radioPrintln("⚠️ Some current sensors not detected");
  }
  else
  {
    radioPrintln("❌ No current sensors detected");
  }
}

/**
 * Checks if current sensor values are within valid range
 *
 * @return true if all sensors are reading valid current/voltage, false otherwise
 */
bool checkCurrentSensors()
{
  radioPrintln("\n--- CURRENT SENSOR CHECK ---");

  bool allValid = true;
  int validSensors = 0;

  for (int i = 0; i < numCurrentSensors; i++)
  {
    // Read current and voltage from each sensor
    float current_mA = currentSensors[i].getCurrent_mA();
    float bus_voltage_V = currentSensors[i].getBusVoltage_V();

    // Check if readings are within reasonable ranges
    // Current: -400mA to +400mA (INA219 range)
    // Voltage: 0V to 16V (INA219 range)
    bool currentValid = (current_mA >= -400.0 && current_mA <= 400.0);
    bool voltageValid = (bus_voltage_V >= 0.0 && bus_voltage_V <= 16.0);
    bool isValid = currentValid && voltageValid;

    radioPrint(currentSensorsLabels[i]);
    radioPrint(": ");
    radioPrint(String(current_mA, 2));
    radioPrint(" mA, ");
    radioPrint(String(bus_voltage_V, 2));
    radioPrint(" V ");

    if (isValid)
    {
      radioPrintln("Y");
      validSensors++;
    }
    else
    {
      radioPrintln("N");
      allValid = false;

      if (!currentValid)
      {
        radioPrint("  Current out of range: ");
        radioPrintln(String(current_mA, 2));
      }
      if (!voltageValid)
      {
        radioPrint("  Voltage out of range: ");
        radioPrintln(String(bus_voltage_V, 2));
      }
    }
  }

  radioPrint("Valid sensors: ");
  radioPrint(String(validSensors));
  radioPrint("/");
  radioPrintln(String(numCurrentSensors));

  if (allValid)
  {
    radioPrintln("✓ All current sensors operational");
  }
  else
  {
    radioPrintln("⚠️ Some current sensors showing invalid readings");
  }

  return allValid;
}

/**
 * Gets current and voltage data from all sensors for telemetry transmission
 *
 * @param currents Array to store current readings (must be at least numCurrentSensors)
 * @param voltages Array to store voltage readings (must be at least numCurrentSensors)
 * @return Number of valid sensor readings
 */
int getValidCurrentData(float currents[], float voltages[])
{
  int validCount = 0;

  for (int i = 0; i < numCurrentSensors; i++)
  {
    float current_mA = currentSensors[i].getCurrent_mA();
    float bus_voltage_V = currentSensors[i].getBusVoltage_V();

    currents[i] = current_mA;
    voltages[i] = bus_voltage_V;

    // Count valid readings
    bool currentValid = (current_mA >= -400.0 && current_mA <= 400.0);
    bool voltageValid = (bus_voltage_V >= 0.0 && bus_voltage_V <= 16.0);

    if (currentValid && voltageValid)
    {
      validCount++;
    }
  }

  return validCount;
}

void initRPI()
{
  // Initialize Raspberry Pi control pin and make sure its off.
  pinMode(RPI_ENABLE, OUTPUT);
  digitalWrite(RPI_ENABLE, LOW);

  // Configure trigger pin for RPI communication and leave HIGH (if LOW it will trigger thermal data capture script)
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);

  // Initialize UART for RPi communication
  Serial2.begin(115200); // High-speed UART for data transfer
  radioPrintln("RPI UART initialized at 115200 baud");
}
/**
 * Initializes the RF22 radio module for transmit mode
 *
 * Configures SPI1 interface, sets radio frequency to 433MHz, configures
 * modem settings for GFSK modulation, and sets the radio to idle mode
 * ready for transmission. Also configures RX/TX control pins.
 */
void initRadio()
{
  radioPrintln("Initializing radio");

  // Configure RX/TX control pins
  pinMode(30, OUTPUT);    // RX_ON pin
  pinMode(31, OUTPUT);    // TX_ON pin
  digitalWrite(30, LOW);  // RX_ON = LOW for transmit mode
  digitalWrite(31, HIGH); // TX_ON = HIGH for transmit mode
  delay(100);

  // Configure SPI1 interface for radio communication
  SPI1.setMISO(39); // Master In, Slave Out
  SPI1.setMOSI(26); // Master Out, Slave In
  SPI1.setSCK(27);  // Serial Clock

  // LED for status of active use of radio (sending/receiving packets)
  pinMode(LED_PIN, OUTPUT);

  // Initialize RF22 radio module
  if (!rf23.init())
  {
    radioPrintln("Radio init failed!");
    radioReady = false;
    return; // init failed dont setup the rest.
  }

  // Configure radio parameters
  rf23.setFrequency(433.0);                     // Set frequency to 433MHz
  rf23.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45); // GFSK modulation, 9.6kbps
  rf23.setTxPower(RH_RF22_TXPOW_20DBM);         // Set transmit power to 20dBm
  rf23.setModeIdle();                           // Set radio to idle mode
  delay(100);
  radioReady = true;
  radioPrintln("Radio ready");
}

/**
 * Main loop - handles user commands and system operations
 *
 * Continuously monitors serial interface for user commands and executes
 * the corresponding operations (capture or transmit).
 */
void loop()
{
  // Handle commands from ground station via radio
  listenForCommands();

  // Also allow manual commands via serial for testing/debug
  if (Serial.available())
  {
    char cmd = Serial.read();
    while (Serial.available())
      Serial.read(); // Clear input buffer

    radioPrint("\nCommand: ");
    radioPrintln(String(cmd));

    switch (cmd)
    {
    case 'u':
    case 'U':
      captureThermalImageUART();
      break;
    case 'r':
    case 'R':
      sendViaRadio();
      break;
    case 't':
    case 'T':
      checkTemperatureSensors();
      break;
    case 'c':
    case 'C':
      checkCurrentSensors();
      break;
    default:
      radioPrintln("Unknown command. Use 'u' for capture, 'r' for transmit, 't' for temperature check, 'c' for current check");
    }
  }
}

/**
 * Captures thermal image data from Raspberry Pi via UART
 *
 * Triggers the RPi to capture thermal data, receives the data via UART,
 * validates the transmission, and stores the image in the buffer.
 * Provides real-time progress updates and data quality assessment.
 */
void captureThermalImageUART()
{
  radioPrintln("\n--- UART THERMAL CAPTURE ---");
  radioPrintln("Triggering RPi capture...");

  // Clear any pending UART data
  while (Serial2.available())
  {
    Serial2.read();
  }

  // Send trigger signal to RPi (falling edge)
  digitalWrite(TRIGGER_PIN, LOW);
  delay(10);
  digitalWrite(TRIGGER_PIN, HIGH);

  radioPrintln("Waiting for RPi processing...");
  delay(10000); // Wait for RPi to capture and process //TODO get alert from RPI instead of delay.

  radioPrintln("\n--- RECEIVING DATA VIA UART ---");

  // Wait for data to start arriving with timeout
  unsigned long timeout = millis() + 20000; // 20 second timeout
  while (!Serial2.available() && millis() < timeout)
  {
    delay(10);
  }

  if (!Serial2.available())
  {
    radioPrintln("ERROR: No UART data received within timeout!");
    return;
  }

  radioPrintln("UART data detected, receiving...");

  // Read header (magic bytes + length)
  uint8_t header[6];
  size_t headerRead = Serial2.readBytes(header, 6);

  if (headerRead != 6)
  {
    radioPrintln("ERROR: Incomplete header received!");
    return;
  }

  // Validate magic bytes (0xDE 0xAD 0xBE 0xEF)
  if (header[0] != 0xDE || header[1] != 0xAD || header[2] != 0xBE || header[3] != 0xEF)
  {
    radioPrint("ERROR: Invalid magic bytes: ");
    for (int i = 0; i < 4; i++)
    {
      radioPrint("0x");
      radioPrint(String(header[i], HEX));
      radioPrint(" ");
    }
    radioPrintln();
    return;
  }

  radioPrintln("Valid header received!");

  // Extract data length (little-endian format)
  uint16_t dataLen = header[4] | (header[5] << 8);

  radioPrint("Expected data length: ");
  radioPrint(String(dataLen));
  radioPrintln(" bytes");

  if (dataLen == 0 || dataLen > MAX_IMG)
  {
    radioPrintln("ERROR: Invalid data length!");
    return;
  }

  // Receive image data with progress tracking
  radioPrintln("Receiving thermal data via UART...");
  unsigned long startTime = millis();
  uint16_t totalReceived = 0;

  while (totalReceived < dataLen)
  {
    // Check for timeout (30 seconds total)
    if (millis() - startTime > 30000)
    {
      radioPrintln("ERROR: UART receive timeout!");
      break;
    }

    // Read available data
    int available = Serial2.available();
    if (available > 0)
    {
      // Don't read more than we need
      int toRead = min(available, (int)(dataLen - totalReceived));
      size_t actualRead = Serial2.readBytes(&imgBuf[totalReceived], toRead);
      totalReceived += actualRead;

      // Progress update every 2KB
      if (totalReceived % 2048 == 0 || totalReceived == dataLen)
      {
        float progress = (float)totalReceived / dataLen * 100;
        float elapsed = (millis() - startTime) / 1000.0;
        float rate = totalReceived / elapsed;

        radioPrint("Progress: ");
        radioPrint(String(totalReceived));
        radioPrint("/");
        radioPrint(String(dataLen));
        radioPrint(" (");
        radioPrint(String(progress, 1));
        radioPrint("%) - ");
        radioPrint(String(rate, 0));
        radioPrintln(" bytes/sec");
      }
    }
    else
    {
      delay(1); // Small delay if no data available
    }
  }

  // Read end markers
  uint8_t endMarkers[2];
  Serial2.readBytes(endMarkers, 2);

  float totalTime = (millis() - startTime) / 1000.0;

  radioPrintln("\n--- UART RECEPTION COMPLETE ---");
  radioPrint("Received ");
  radioPrint(String(totalReceived));
  radioPrint(" bytes in ");
  radioPrint(String(totalTime, 2));
  radioPrintln(" seconds");

  radioPrint("Average rate: ");
  radioPrint(String(totalReceived / totalTime, 0));
  radioPrintln(" bytes/sec");

  capturedImageLength = totalReceived;

  // Validate thermal data quality
  if (totalReceived >= 38400)
  { // Expected thermal image size
    int validPixels = 0;
    for (int i = 0; i < totalReceived - 1; i += 2)
    {
      uint16_t pixel = imgBuf[i] | (imgBuf[i + 1] << 8);
      float tempC = (pixel - 27315) / 100.0;
      if (tempC >= 0 && tempC <= 60)
        validPixels++; // Reasonable temperature range
    }

    float validPct = (float)validPixels * 100.0 / (totalReceived / 2);
    radioPrint("Data quality: ");
    radioPrint(String(validPct, 1));
    radioPrintln("% valid temperature pixels");

    if (validPct > 80)
    {
      radioPrintln("✓ Excellent data quality! Ready for radio transmission - press 'r'");
    }
    else if (validPct > 50)
    {
      radioPrintln("⚠️ Moderate data quality - may still be usable");
    }
    else
    {
      radioPrintln("❌ Poor data quality detected");
    }
  }
  else
  {
    radioPrintln("⚠️ Received data size doesn't match expected thermal image size");
  }
}

/**
 * Sends a packet reliably with retry logic
 *
 * Attempts to send a packet up to MAX_RETRIES times, with proper
 * radio state management and error handling. Provides visual feedback
 * via LED during transmission.
 *
 * @param data Pointer to packet data
 * @param len Length of packet data
 * @return true if packet was sent successfully, false otherwise
 */
bool sendPacketReliable(uint8_t *data, uint8_t len)
{
  const int MAX_RETRIES = 3;
  for (int retry = 0; retry < MAX_RETRIES; retry++)
  {
    rf23.setModeIdle(); // Set radio to idle mode
    delay(5);

    // Clear interrupt flags
    rf23.spiRead(RH_RF22_REG_03_INTERRUPT_STATUS1);
    rf23.spiRead(RH_RF22_REG_04_INTERRUPT_STATUS2);

    digitalWrite(LED_PIN, HIGH); // Turn on LED during transmission

    if (rf23.send(data, len))
    {
      if (rf23.waitPacketSent(500))
      {
        digitalWrite(LED_PIN, LOW); // Turn off LED
        return true;
      }
    }

    digitalWrite(LED_PIN, LOW); // Turn off LED
    if (retry < MAX_RETRIES - 1)
    {
      delay(50); // Delay before retry
    }
  }
  return false;
}

/**
 * Transmits captured thermal image data via radio
 *
 * Sends the captured image data in packetized format with header,
 * data packets, and end packet. Provides transmission statistics
 * and quality assessment. Requires captured image data to be present.
 */
void sendViaRadio()
{
  if (capturedImageLength == 0)
  {
    radioPrintln("No image captured yet!");
    return;
  }

  radioPrintln("\n--- SENDING THERMAL IMAGE VIA RADIO ---");
  radioPrint("Image size: ");
  radioPrint(String(capturedImageLength));
  radioPrintln(" bytes");

  // Calculate total packets needed
  uint16_t totalPackets = (capturedImageLength + PACKET_DATA_SIZE - 1) / PACKET_DATA_SIZE;
  radioPrint("Total packets: ");
  radioPrintln(String(totalPackets));

  // Send header packet with image metadata
  radioPrintln("Sending header...");
  uint8_t header[10];
  header[0] = 0xFF; // TODO: tf does this mean
  header[1] = 0xFF;
  header[2] = capturedImageLength & 0xFF; // Image length (little-endian)
  header[3] = (capturedImageLength >> 8) & 0xFF;
  header[4] = totalPackets & 0xFF; // Total packets (little-endian)
  header[5] = (totalPackets >> 8) & 0xFF;
  header[6] = 0xDE; // Magic bytes for validation
  header[7] = 0xAD;
  header[8] = 0xBE;
  header[9] = 0xEF;

  // Send header with retry logic
  bool headerSent = false;
  for (int retry = 0; retry < 5 && !headerSent; retry++)
  {
    if (sendPacketReliable(header, 10))
    {
      headerSent = true;
      radioPrintln("✓ Header sent");
    }
    else
    {
      radioPrint("Header retry ");
      radioPrintln(String(retry + 1));
      delay(200);
    }
  }

  if (!headerSent)
  {
    radioPrintln("❌ Failed to send header!");
    return;
  }

  delay(1000); // Wait for receiver to prepare

  // Send data packets
  radioPrintln("Sending data packets...");
  uint16_t bytesSent = 0;
  uint16_t packetNum = 0;
  uint16_t successCount = 0;
  uint16_t failCount = 0;
  unsigned long startTime = millis();

  while (bytesSent < capturedImageLength)
  {
    uint16_t chunkSize = min((int)PACKET_DATA_SIZE, capturedImageLength - bytesSent);
    uint8_t packet[PACKET_DATA_SIZE + 2];

    // Packet number (little-endian)
    packet[0] = packetNum & 0xFF;
    packet[1] = (packetNum >> 8) & 0xFF;

    // Copy image data to packet
    memcpy(&packet[2], &imgBuf[bytesSent], chunkSize);

    // Progress update every 50 packets
    if (packetNum % 50 == 0)
    {
      float progress = (float)bytesSent / capturedImageLength * 100;
      radioPrint("Progress: ");
      radioPrint(String(progress, 1));
      radioPrintln("%");
    }

    // Send packet with retry logic
    if (sendPacketReliable(packet, chunkSize + 2))
    {
      successCount++;
    }
    else
    {
      failCount++;
    }

    bytesSent += chunkSize;
    packetNum++;
    delay(PACKET_DELAY_MS); // Delay between packets
  }

  // Send end packet to signal completion
  radioPrintln("Sending end packet...");
  uint8_t endPkt[6];
  endPkt[0] = 0xEE; // End packet magic bytes
  endPkt[1] = 0xEE;
  endPkt[2] = packetNum & 0xFF; // Final packet count (little-endian)
  endPkt[3] = (packetNum >> 8) & 0xFF;
  endPkt[4] = 0xFF; // Padding
  endPkt[5] = 0xFF;

  // Send end packet multiple times for reliability
  for (int i = 0; i < 3; i++)
  {
    sendPacketReliable(endPkt, 6);
    delay(200);
  }

  // Display transmission summary
  float duration = (millis() - startTime) / 1000.0;
  radioPrintln("\n=== TRANSMISSION COMPLETE ===");
  radioPrint("Packets sent: ");
  radioPrint(String(successCount));
  radioPrint("/");
  radioPrint(String(totalPackets));
  radioPrint(" (");
  radioPrint(String((float)successCount / totalPackets * 100, 1));
  radioPrintln("% success rate)");

  radioPrint("Duration: ");
  radioPrint(String(duration, 1));
  radioPrintln(" seconds");

  radioPrint("Data rate: ");
  radioPrint(String(capturedImageLength / duration, 0));
  radioPrintln(" bytes/sec");

  // Quality assessment based on success rate
  if (successCount == totalPackets)
  {
    radioPrintln("\n✅ Perfect transmission!");
  }
  else if (successCount > totalPackets * 0.95)
  {
    radioPrintln("\n✅ Excellent transmission!");
  }
  else
  {
    radioPrintln("\n⚠️ Some packets lost");
  }
}

/**
 * Sends a serial message via radio to ground station
 *
 * @param message The message string to send
 */
void radioPrint(const String &message)
{
  if (!radioReady)
    return;

  // Add message to buffer
  serialBuffer += message;
  
  // Send buffer when it gets long enough or contains newlines
  if (serialBuffer.length() >= MAX_SERIAL_MSG_LEN || serialBuffer.indexOf('\n') != -1)
  {
    sendSerialBuffer();
  }
}

/**
 * Sends a serial message with newline via radio to ground station
 *
 * @param message The message string to send (optional)
 */
void radioPrintln(const String &message = "")
{
  radioPrint(message + "\n");
  sendSerialBuffer(); // Force send after newline
}

/**
 * Sends accumulated serial buffer via radio
 */
void sendSerialBuffer()
{
  if (!radioReady || serialBuffer.length() == 0)
    return;

  // Create packet: [MSG_TYPE][LENGTH][MESSAGE_DATA]
  uint8_t packet[MAX_SERIAL_MSG_LEN + 2];
  packet[0] = SERIAL_MSG_TYPE;
  
  // Split long messages into chunks
  while (serialBuffer.length() > 0)
  {
    uint8_t chunkSize = min(serialBuffer.length(), (unsigned int)MAX_SERIAL_MSG_LEN);
    packet[1] = chunkSize;
    
    // Copy message data
    for (uint8_t i = 0; i < chunkSize; i++)
    {
      packet[2 + i] = serialBuffer.charAt(i);
    }
    
    // Send packet
    if (sendPacketReliable(packet, chunkSize + 2))
    {
      // Remove sent chunk from buffer
      serialBuffer = serialBuffer.substring(chunkSize);
    }
    else
    {
      // Failed to send, keep buffer for retry
      break;
    }
    
    delay(10); // Small delay between chunks
  }
}

/**
 * Listens for commands from ground station via radio and handles them
 *
 */
void listenForCommands()
{
  uint8_t buf[1];
  uint8_t len = sizeof(buf);

  while (rf23.available())
  {
    if (rf23.recv(buf, &len))
    {
      // Process received command
      char cmd = (char)buf[0];

      if (cmd.toLowerCase() == 'u')
      {
        captureThermalImageUART();
      }
      else if (cmd.toLowerCase() == 'r')
      {
        sendViaRadio();
      }
      else
      {
        radioPrintln("Unknown command received via radio");
        return;
      }
    }
  }
}