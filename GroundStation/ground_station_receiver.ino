/**
 * Ground Station Receiver for EPSCOR C3M Payload
 * 
 * This Arduino sketch runs on a Teensy microcontroller to receive thermal image data
 * from the satellite payload via radio communication. It handles packetized data
 * transmission with error checking and provides data export capabilities.
 * 
 * Key Features:
 * - Receives thermal image data via RF22 radio module
 * - Handles packetized transmission with header/end packets
 * - Tracks packet reception and provides statistics
 * - Exports thermal data as CSV for analysis
 * - Auto mode for continuous reception
 * 
 * Hardware Requirements:
 * - Teensy microcontroller
 * - RF22 radio module (433MHz)
 * - SPI1 interface for radio communication
 * 
 * @author EPSCOR C3M Team
 * @date 8/25/2025
 * @version 1.0
 */

//Receiver (simplified) - ground station teensy 8/25/2025
// Simplified Receiver v1 (Essential Functions Only)
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF22.h>
#include <RHHardwareSPI1.h>

// Radio configuration pins and object
const int RADIO_CS = 38;    // Chip select pin for RF22 module
const int RADIO_INT = 40;   // Interrupt pin for RF22 module
RH_RF22 rf23(RADIO_CS, RADIO_INT, hardware_spi1);

// Image reception buffer and tracking variables
const uint32_t MAX_IMG = 40000;  // Maximum image buffer size (40KB)
uint8_t imgBuffer[MAX_IMG];      // Buffer to store received thermal image data
uint16_t expectedLength = 0;     // Expected total image size from header
uint16_t expectedPackets = 0;    // Total number of packets expected
uint16_t receivedPackets = 0;    // Number of packets successfully received
bool headerReceived = false;     // Flag indicating header packet was received
bool imageComplete = false;      // Flag indicating all data was received

// Packet tracking for duplicate detection and missing packet identification
bool packetReceived[1200];       // Array to track which packets have been received
uint16_t maxPacketNum = 0;       // Highest packet number received

// Communication statistics
unsigned long packetsReceived = 0;  // Total packets received (including duplicates)
unsigned long lastPacketTime = 0;   // Timestamp of last packet reception
int lastRSSI = 0;                  // Signal strength of last received packet

// Auto mode flag for continuous reception
bool autoMode = false;

// Packet size configuration
const uint8_t PACKET_DATA_SIZE = 45;  // Data payload size per packet

/**
 * Setup function - initializes the ground station receiver
 * 
 * Configures serial communication, GPIO pins, radio module, and displays
 * available commands to the user.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n=== Ground Station Receiver ===");
  
  pinMode(13, OUTPUT); // LED for status indication
  initRadioForReceive();
  
  Serial.println("\nCommands:");
  Serial.println(" 'a' - Auto mode (start this first!)");
  Serial.println(" 'e' - Export as CSV");
  Serial.println("\nType 'a' to start auto mode before transmitting!");
}

/**
 * Initializes the RF22 radio module for receive mode
 * 
 * Configures SPI1 interface, sets radio frequency to 433MHz, configures
 * modem settings for GFSK modulation, and sets the radio to receive mode.
 * Also configures RX/TX control pins for proper receive operation.
 */
void initRadioForReceive() {
  Serial.println("Initializing radio for receive...");
  
  // Configure RX/TX control pins BEFORE radio initialization
  pinMode(30, OUTPUT);  // RX_ON pin
  pinMode(31, OUTPUT);  // TX_ON pin
  digitalWrite(30, HIGH); // RX_ON = HIGH for receive mode
  digitalWrite(31, LOW);  // TX_ON = LOW for receive mode
  delay(100);
  
  // Configure SPI1 interface for radio communication
  SPI1.setMISO(39);  // Master In, Slave Out
  SPI1.setMOSI(26);  // Master Out, Slave In
  SPI1.setSCK(27);   // Serial Clock
  
  // Initialize RF22 radio module
  if (!rf23.init()) {
    Serial.println("Radio init failed!");
    while(1);  // Halt if radio initialization fails
  }
  
  // Configure radio parameters
  rf23.setFrequency(433.0);  // Set frequency to 433MHz
  rf23.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45);  // GFSK modulation, 9.6kbps
  rf23.setModeRx();  // Set radio to receive mode
  Serial.println("Radio ready for receive");
  
  clearReception();  // Initialize reception state
}

/**
 * Clears all reception state variables
 * 
 * Resets packet tracking, counters, and flags to prepare for a new
 * image transmission. Called at startup and before each new reception.
 */
void clearReception() {
  headerReceived = false;
  imageComplete = false;
  expectedLength = 0;
  expectedPackets = 0;
  receivedPackets = 0;
  maxPacketNum = 0;
  memset(packetReceived, false, sizeof(packetReceived));  // Clear packet tracking array
}

/**
 * Main loop - handles packet reception and user commands
 * 
 * Continuously checks for incoming radio packets and processes user
 * commands from serial interface. In auto mode, focuses on packet
 * reception; otherwise, handles both packets and commands.
 */
void loop() {
  // Check for incoming packets when not in auto mode
  if (!autoMode && rf23.available()) {
    handlePacket();
  }
  
  // Process user commands from serial interface
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    switch (cmd) {
      case 'a':
      case 'A':
        runAutoMode();  // Start automatic reception mode
        break;
      case 'e':
      case 'E':
        exportThermalData();  // Export received data as CSV
        break;
      case 'u': // command for captureThermalImageUART() on satellite
      case 'U':
      case 'r': // command for sendViaRadio() on satellite
      case 'R':
        forwardToSatellite(cmd);
        break;
      default:
        if (!autoMode) {
          Serial.println("Unknown command. Use 'a' for auto mode, 'e' for export, 'u' to trigger image capture, 'r' to downlink data");
        }
    }
  }
}

/**
 * Handles incoming radio packets
 * 
 * Receives data from the radio module, updates statistics, and processes
 * the packet content. Provides visual feedback via LED and tracks
 * signal strength (RSSI).
 */
void handlePacket() {
  uint8_t buf[64];  // Buffer for incoming packet data
  uint8_t len = sizeof(buf);
  
  if (rf23.recv(buf, &len)) {
    digitalWrite(13, HIGH);  // Turn on LED to indicate packet reception
    lastPacketTime = millis();
    lastRSSI = rf23.lastRssi();  // Store signal strength
    packetsReceived++;
    processPacket(buf, len);  // Process the received packet
    digitalWrite(13, LOW);   // Turn off LED
  }
}

/**
 * Processes received packet based on its type and content
 * 
 * Analyzes packet length and header bytes to determine packet type:
 * - Header packets (10 bytes, starts with 0xFF 0xFF)
 * - End packets (6 bytes, starts with 0xEE 0xEE)
 * - Data packets (variable length, contains image data)
 * 
 * @param buf Pointer to packet data buffer
 * @param len Length of received packet
 */
void processPacket(uint8_t* buf, uint8_t len) {
  // Check for header packet (10 bytes with magic bytes)
  if (len == 10 && buf[0] == 0xFF && buf[1] == 0xFF) {
    handleHeaderPacket(buf);
    return;
  }
  
  // Check for end packet (6 bytes with magic bytes)
  if (len == 6 && buf[0] == 0xEE && buf[1] == 0xEE) {
    handleEndPacket(buf);
    return;
  }
  
  // Check for data packet (requires valid header and incomplete image)
  if (len >= 3 && headerReceived && !imageComplete) {
    handleDataPacket(buf, len);
    return;
  }
}

/**
 * Processes header packet containing image metadata
 * 
 * Extracts image size, packet count, and validates magic bytes.
 * Initializes reception state for the upcoming data transmission.
 * 
 * @param buf Pointer to header packet data
 */
void handleHeaderPacket(uint8_t* buf) {
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
  if (buf[6] == 0xDE && buf[7] == 0xAD && buf[8] == 0xBE && buf[9] == 0xEF) {
    Serial.println("✓ Header valid - receiving thermal data...");
    headerReceived = true;
    imageComplete = false;
    receivedPackets = 0;
    maxPacketNum = 0;
    memset(packetReceived, false, sizeof(packetReceived));  // Reset packet tracking
  } else {
    Serial.println("✗ Invalid header!");
  }
}

/**
 * Processes end packet indicating transmission completion
 * 
 * Extracts final packet count from transmitter and marks image
 * reception as complete. Exits auto mode and displays reception summary.
 * 
 * @param buf Pointer to end packet data
 */
void handleEndPacket(uint8_t* buf) {
  uint16_t finalCount = buf[2] | (buf[3] << 8);  // Extract final packet count
  Serial.println("\n🏁 END PACKET RECEIVED!");
  Serial.print("Transmitter sent ");
  Serial.print(finalCount);
  Serial.println(" packets");
  
  imageComplete = true;
  autoMode = false;  // Exit auto mode
  showReceptionSummary();
}

/**
 * Processes data packet containing image data
 * 
 * Extracts packet number and image data, stores it in the buffer,
 * and tracks reception progress. Handles duplicate packets and
 * provides progress updates in auto mode.
 * 
 * @param buf Pointer to data packet buffer
 * @param len Length of data packet
 */
void handleDataPacket(uint8_t* buf, uint8_t len) {
  if (len < 3) return;  // Minimum packet size check
  
  // Extract packet number and data length
  uint16_t packetNum = buf[0] | (buf[1] << 8);  // Little-endian packet number
  uint8_t dataLen = len - 2;  // Data length excludes packet number bytes
  
  if (packetNum >= 1200) return;  // Bounds check for packet number
  
  // Process packet only if not already received (duplicate detection)
  if (!packetReceived[packetNum]) {
    uint32_t bufferPos = (uint32_t)packetNum * PACKET_DATA_SIZE;  // Calculate buffer position
    if (bufferPos + dataLen <= MAX_IMG) {  // Buffer overflow protection
      memcpy(&imgBuffer[bufferPos], &buf[2], dataLen);  // Copy data to buffer
      packetReceived[packetNum] = true;  // Mark packet as received
      receivedPackets++;
      
      if (packetNum > maxPacketNum) {
        maxPacketNum = packetNum;  // Track highest packet number
      }
      
      // Progress indication in auto mode
      if (autoMode && receivedPackets % 10 == 0) {
        Serial.print(".");
        if (receivedPackets % 100 == 0) {
          float progress = (float)receivedPackets / expectedPackets * 100;
          Serial.print(" ");
          Serial.print(progress, 0);
          Serial.println("%");
        }
      }
    }
  }
}

/**
 * Runs automatic reception mode for continuous image capture
 * 
 * Clears reception state, enters auto mode, and continuously monitors
 * for incoming packets until image is complete or timeout occurs.
 * Provides real-time progress updates and handles user interruption.
 */
void runAutoMode() {
  Serial.println("\n=== AUTO MODE - THERMAL IMAGE RECEPTION ===");
  Serial.println("Clearing buffer and waiting for transmission...");
  Serial.println("Start transmission from flat sat now!");
  Serial.println("Press any key to abort\n");
  
  clearReception();  // Reset reception state
  autoMode = true;   // Enable auto mode
  rf23.setModeRx();  // Ensure radio is in receive mode
  
  unsigned long lastActivity = millis();
  while (!Serial.available() && !imageComplete) {
    if (rf23.available()) {
      handlePacket();
      lastActivity = millis();  // Update activity timestamp
    }
    
    // Timeout if no activity for 30 seconds after header received
    if (headerReceived && (millis() - lastActivity > 30000)) {
      Serial.println("\n\nTimeout - no packets for 30 seconds");
      break;
    }
    delay(1);
  }
  
  // Clear any key press from input buffer
  while (Serial.available()) Serial.read();
  autoMode = false;  // Exit auto mode if end packet not received
  
  // Handle different completion scenarios
  if (!imageComplete && !headerReceived) {
    Serial.println("\nNo transmission detected");
  } else if (!imageComplete) {
    Serial.println("\nTransmission interrupted");
    showReceptionSummary();
  }
}

/**
 * Displays reception statistics and quality assessment
 * 
 * Shows packet reception rates, success percentages, and provides
 * quality assessment based on reception completeness. Indicates
 * when thermal image is ready for export.
 */
void showReceptionSummary() {
  Serial.println("\n=== RECEPTION SUMMARY ===");
  Serial.print("Received ");
  Serial.print(receivedPackets);
  Serial.print(" of ");
  Serial.print(expectedPackets);
  Serial.print(" packets (");
  Serial.print((float)receivedPackets / expectedPackets * 100, 1);
  Serial.println("%)");
  
  // Quality assessment based on reception rate
  if (receivedPackets == expectedPackets) {
    Serial.println("\n✅ PERFECT RECEPTION!");
  } else if (receivedPackets > expectedPackets * 0.95) {
    Serial.println("\n✅ Excellent reception!");
  } else if (receivedPackets > expectedPackets * 0.80) {
    Serial.println("\n⚠️ Good reception");
  } else {
    Serial.println("\n❌ Poor reception");
  }
  
  // Indicate if thermal image is ready for export
  if (expectedLength == 38400) {
    Serial.println("\nThermal image ready! Press 'e' to export as CSV");
  }
}

/**
 * Exports received thermal image data as CSV format
 * 
 * Converts raw thermal data to temperature values in Celsius and
 * outputs as comma-separated values. Provides visualization
 * instructions for Python analysis. Only works with complete
 * thermal images (38400 bytes expected).
 */
void exportThermalData() {
  if (!headerReceived || expectedLength != 38400) {
    Serial.println("No complete thermal image to export");
    return;
  }
  
  Serial.println("\n--- EXPORTING THERMAL DATA ---");
  Serial.println("Copy data below to 'thermal_image.csv'");
  Serial.println("=== START CSV ===");
  
  // Export thermal data as CSV (120x160 pixel grid)
  for (int row = 0; row < 120; row++) {
    for (int col = 0; col < 160; col++) {
      int idx = (row * 160 + col) * 2;  // 2 bytes per pixel
      if (idx < MAX_IMG - 1) {
        // Convert raw 16-bit value to temperature in Celsius
        uint16_t pixel = imgBuffer[idx] | (imgBuffer[idx+1] << 8);
        if (pixel >= 27315 && pixel <= 37315) {  // Valid temperature range (0-100°C)
          float tempC = (pixel - 27315) / 100.0;  // Convert from Kelvin*100 to Celsius
          Serial.print(tempC, 2);
        } else {
          Serial.print("NaN");  // Invalid temperature value
        }
      } else {
        Serial.print("NaN");  // Buffer overflow protection
      }
      if (col < 159) Serial.print(",");  // CSV separator
    }
    Serial.println();  // New line for each row
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

/**
 * Forwards commands to the satellite via radio
 * Since commands are small, we can send them without packetization
 *
 * @param cmd Character command to forward ('u' for capture, 'r' for receive)
 */
void forwardToSatellite(char cmd) {
  if (cmd.toLowerCase() == 'u') {
    Serial.println("\n--- UART THERMAL CAPTURE ---");
  } else if (cmd.toLowerCase() == 'r') {
    Serial.println("\n--- REQUESTING THERMAL DATA DOWNLINK ---");
  } else {
    return; // Ignore unknown commands
  }

  Serial.println("Forwarding command to satellite...");

  // Set radio to transmit mode
  rf23.setModeTx();
  digitalWrite(31, HIGH); // Turn on LED during transmission
  delay(10); // Allow time for TX_ON to stabilize   

  // Send the command
  if (!rf23.send((uint8_t *) &cmd, 1)) { // a command should be a single byte
    Serial.print("Failed to queue command for transmission: ");
    Serial.println(cmd);
  }
  else {
    if (!rf23.waitPacketSent(500)) {
      Serial.print("Failed to send command: ");
      Serial.println(cmd);
    } else {
      Serial.print("Command sent successfully: ");
      Serial.println(cmd);
    }
  }

  digitalWrite(31, LOW); // Turn off LED
  rf23.setModeRx();  // Return to receive mode
  return;

}