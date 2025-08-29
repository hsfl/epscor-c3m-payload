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
//UART TRANSMITTER CODE - satellite teensy (8/25/2025)
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF22.h>
#include <RHHardwareSPI1.h>

// Pin definitions for hardware control
const uint8_t RPI_ENABLE = 36;    // Power control pin for Raspberry Pi
const uint8_t TRIGGER_PIN = 2;    // Trigger pin to signal RPi for capture
// UART pins: Serial2 uses pins 7 (RX) and 8 (TX) automatically

// Radio configuration pins and object
const int RADIO_CS = 38;          // Chip select pin for RF22 module
const int RADIO_INT = 40;         // Interrupt pin for RF22 module
RH_RF22 rf23(RADIO_CS, RADIO_INT, hardware_spi1);

// Image data storage and tracking
uint16_t capturedImageLength = 0;  // Length of captured thermal image

// Buffer for thermal image storage
const uint32_t MAX_IMG = 40000;    // Maximum image buffer size (40KB)
uint8_t imgBuf[MAX_IMG];           // Buffer to store thermal image data

// Radio transmission parameters
const uint8_t PACKET_DATA_SIZE = 45;   // Data payload size per packet
const uint16_t PACKET_DELAY_MS = 20;   // Delay between packet transmissions

/**
 * Setup function - initializes the satellite payload transmitter
 * 
 * Configures serial communication, GPIO pins, radio module, and UART
 * interface. Powers up the Raspberry Pi and displays available commands.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n=== Teensy Flat Sat Transmitter - UART Version ===");
  
  // Power up Raspberry Pi
  pinMode(RPI_ENABLE, OUTPUT);
  digitalWrite(RPI_ENABLE, HIGH);
  
  // Configure trigger pin for RPi communication
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  pinMode(13, OUTPUT); // LED for status indication
  
  // Initialize UART for RPi communication
  Serial2.begin(115200); // High-speed UART for data transfer
  Serial.println("UART initialized at 115200 baud");
  
  initRadioForTransmit();
  
  Serial.println("\nWaiting 5 seconds for RPi boot...");
  delay(5000);
  
  Serial.println("\nCommands:");
  Serial.println(" 'u' - UART thermal capture (fast!)");
  Serial.println(" 'r' - Send captured image via radio");
  Serial.println("\nReady!");
}

/**
 * Initializes the RF22 radio module for transmit mode
 * 
 * Configures SPI1 interface, sets radio frequency to 433MHz, configures
 * modem settings for GFSK modulation, and sets the radio to idle mode
 * ready for transmission. Also configures RX/TX control pins.
 */
void initRadioForTransmit() {
  Serial.println("Initializing radio for transmit...");
  
  // Configure RX/TX control pins
  pinMode(30, OUTPUT);  // RX_ON pin
  pinMode(31, OUTPUT);  // TX_ON pin
  digitalWrite(30, LOW);   // RX_ON = LOW for transmit mode
  digitalWrite(31, HIGH);  // TX_ON = HIGH for transmit mode
  delay(100);
  
  // Configure SPI1 interface for radio communication
  SPI1.setMISO(39);  // Master In, Slave Out
  SPI1.setMOSI(26);  // Master Out, Slave In
  SPI1.setSCK(27);   // Serial Clock
  
  // Initialize RF22 radio module
  if (!rf23.init()) {
    Serial.println("Radio init failed - continuing anyway");
  }
  
  // Configure radio parameters
  rf23.setFrequency(433.0);  // Set frequency to 433MHz
  rf23.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45);  // GFSK modulation, 9.6kbps
  rf23.setTxPower(RH_RF22_TXPOW_20DBM);  // Set transmit power to 20dBm
  rf23.setModeIdle();  // Set radio to idle mode
  delay(100);
  Serial.println("Radio ready for transmit");
}

/**
 * Main loop - handles user commands and system operations
 * 
 * Continuously monitors serial interface for user commands and executes
 * the corresponding operations (capture or transmit).
 */
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();  // Clear input buffer
    
    Serial.print("\nCommand: ");
    Serial.println(cmd);
    
    switch (cmd) {
      case 'u':
      case 'U':
        captureThermalImageUART();  // Capture thermal image via UART
        break;
      case 'r':
      case 'R':
        sendViaRadio();  // Transmit captured image via radio
        break;
      default:
        Serial.println("Unknown command. Use 'u' for capture, 'r' for transmit");
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
void captureThermalImageUART() {
  Serial.println("\n--- UART THERMAL CAPTURE ---");
  Serial.println("Triggering RPi capture...");
  
  // Clear any pending UART data
  while (Serial2.available()) {
    Serial2.read();
  }
  
  // Send trigger signal to RPi (falling edge)
  digitalWrite(TRIGGER_PIN, LOW);
  delay(10);
  digitalWrite(TRIGGER_PIN, HIGH);
  
  Serial.println("Waiting for RPi processing...");
  delay(10000); // Wait for RPi to capture and process //TODO get alert from RPI instead of delay.
  
  Serial.println("\n--- RECEIVING DATA VIA UART ---");
  
  // Wait for data to start arriving with timeout
  unsigned long timeout = millis() + 20000; // 20 second timeout
  while (!Serial2.available() && millis() < timeout) {
    delay(10);
  }
  
  if (!Serial2.available()) {
    Serial.println("ERROR: No UART data received within timeout!");
    return;
  }
  
  Serial.println("UART data detected, receiving...");
  
  // Read header (magic bytes + length)
  uint8_t header[6];
  size_t headerRead = Serial2.readBytes(header, 6);
  
  if (headerRead != 6) {
    Serial.println("ERROR: Incomplete header received!");
    return;
  }
  
  // Validate magic bytes (0xDE 0xAD 0xBE 0xEF)
  if (header[0] != 0xDE || header[1] != 0xAD || header[2] != 0xBE || header[3] != 0xEF) {
    Serial.print("ERROR: Invalid magic bytes: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("0x");
      Serial.print(header[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    return;
  }
  
  Serial.println("Valid header received!");
  
  // Extract data length (little-endian format)
  uint16_t dataLen = header[4] | (header[5] << 8);
  
  Serial.print("Expected data length: ");
  Serial.print(dataLen);
  Serial.println(" bytes");
  
  if (dataLen == 0 || dataLen > MAX_IMG) {
    Serial.println("ERROR: Invalid data length!");
    return;
  }
  
  // Receive image data with progress tracking
  Serial.println("Receiving thermal data via UART...");
  unsigned long startTime = millis();
  uint16_t totalReceived = 0;
  
  while (totalReceived < dataLen) {
    // Check for timeout (30 seconds total)
    if (millis() - startTime > 30000) {
      Serial.println("ERROR: UART receive timeout!");
      break;
    }
    
    // Read available data
    int available = Serial2.available();
    if (available > 0) {
      // Don't read more than we need
      int toRead = min(available, (int)(dataLen - totalReceived));
      size_t actualRead = Serial2.readBytes(&imgBuf[totalReceived], toRead);
      totalReceived += actualRead;
      
      // Progress update every 2KB
      if (totalReceived % 2048 == 0 || totalReceived == dataLen) {
        float progress = (float)totalReceived / dataLen * 100;
        float elapsed = (millis() - startTime) / 1000.0;
        float rate = totalReceived / elapsed;
        
        Serial.print("Progress: ");
        Serial.print(totalReceived);
        Serial.print("/");
        Serial.print(dataLen);
        Serial.print(" (");
        Serial.print(progress, 1);
        Serial.print("%) - ");
        Serial.print(rate, 0);
        Serial.println(" bytes/sec");
      }
    } else {
      delay(1); // Small delay if no data available
    }
  }
  
  // Read end markers
  uint8_t endMarkers[2];
  Serial2.readBytes(endMarkers, 2);
  
  float totalTime = (millis() - startTime) / 1000.0;
  
  Serial.println("\n--- UART RECEPTION COMPLETE ---");
  Serial.print("Received ");
  Serial.print(totalReceived);
  Serial.print(" bytes in ");
  Serial.print(totalTime, 2);
  Serial.println(" seconds");
  
  Serial.print("Average rate: ");
  Serial.print(totalReceived / totalTime, 0);
  Serial.println(" bytes/sec");
  
  capturedImageLength = totalReceived;
  
  // Validate thermal data quality
  if (totalReceived >= 38400) { // Expected thermal image size
    int validPixels = 0;
    for (int i = 0; i < totalReceived - 1; i += 2) {
      uint16_t pixel = imgBuf[i] | (imgBuf[i+1] << 8);
      float tempC = (pixel - 27315) / 100.0;
      if (tempC >= 0 && tempC <= 60) validPixels++; // Reasonable temperature range
    }
    
    float validPct = (float)validPixels * 100.0 / (totalReceived / 2);
    Serial.print("Data quality: ");
    Serial.print(validPct, 1);
    Serial.println("% valid temperature pixels");
    
    if (validPct > 80) {
      Serial.println("✓ Excellent data quality! Ready for radio transmission - press 'r'");
    } else if (validPct > 50) {
      Serial.println("⚠️ Moderate data quality - may still be usable");
    } else {
      Serial.println("❌ Poor data quality detected");
    }
  } else {
    Serial.println("⚠️ Received data size doesn't match expected thermal image size");
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
bool sendPacketReliable(uint8_t* data, uint8_t len) {
  const int MAX_RETRIES = 3;
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    rf23.setModeIdle();  // Set radio to idle mode
    delay(5);
    
    // Clear interrupt flags
    rf23.spiRead(RH_RF22_REG_03_INTERRUPT_STATUS1);
    rf23.spiRead(RH_RF22_REG_04_INTERRUPT_STATUS2);
    
    digitalWrite(13, HIGH);  // Turn on LED during transmission
    
    if (rf23.send(data, len)) {
      if (rf23.waitPacketSent(500)) {
        digitalWrite(13, LOW);  // Turn off LED
        return true;
      }
    }
    
    digitalWrite(13, LOW);  // Turn off LED
    if (retry < MAX_RETRIES - 1) {
      delay(50);  // Delay before retry
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
void sendViaRadio() {
  if (capturedImageLength == 0) {
    Serial.println("No image captured yet!");
    return;
  }
  
  Serial.println("\n--- SENDING THERMAL IMAGE VIA RADIO ---");
  Serial.print("Image size: ");
  Serial.print(capturedImageLength);
  Serial.println(" bytes");
  
  // Calculate total packets needed
  uint16_t totalPackets = (capturedImageLength + PACKET_DATA_SIZE - 1) / PACKET_DATA_SIZE;
  Serial.print("Total packets: ");
  Serial.println(totalPackets);
  
  // Send header packet with image metadata
  Serial.println("Sending header...");
  uint8_t header[10];
  header[0] = 0xFF; //TODO: tf does this mean
  header[1] = 0xFF;
  header[2] = capturedImageLength & 0xFF;  // Image length (little-endian)
  header[3] = (capturedImageLength >> 8) & 0xFF;
  header[4] = totalPackets & 0xFF;  // Total packets (little-endian)
  header[5] = (totalPackets >> 8) & 0xFF;
  header[6] = 0xDE;  // Magic bytes for validation
  header[7] = 0xAD;
  header[8] = 0xBE;
  header[9] = 0xEF;
  
  // Send header with retry logic
  bool headerSent = false;
  for (int retry = 0; retry < 5 && !headerSent; retry++) {
    if (sendPacketReliable(header, 10)) {
      headerSent = true;
      Serial.println("✓ Header sent");
    } else {
      Serial.print("Header retry ");
      Serial.println(retry + 1);
      delay(200);
    }
  }
  
  if (!headerSent) {
    Serial.println("❌ Failed to send header!");
    return;
  }
  
  delay(1000); // Wait for receiver to prepare
  
  // Send data packets
  Serial.println("Sending data packets...");
  uint16_t bytesSent = 0;
  uint16_t packetNum = 0;
  uint16_t successCount = 0;
  uint16_t failCount = 0;
  unsigned long startTime = millis();
  
  while (bytesSent < capturedImageLength) {
    uint16_t chunkSize = min((int)PACKET_DATA_SIZE, capturedImageLength - bytesSent);
    uint8_t packet[PACKET_DATA_SIZE + 2];
    
    // Packet number (little-endian)
    packet[0] = packetNum & 0xFF;
    packet[1] = (packetNum >> 8) & 0xFF;
    
    // Copy image data to packet
    memcpy(&packet[2], &imgBuf[bytesSent], chunkSize);
    
    // Progress update every 50 packets
    if (packetNum % 50 == 0) {
      float progress = (float)bytesSent / capturedImageLength * 100;
      Serial.print("Progress: ");
      Serial.print(progress, 1);
      Serial.println("%");
    }
    
    // Send packet with retry logic
    if (sendPacketReliable(packet, chunkSize + 2)) {
      successCount++;
    } else {
      failCount++;
    }
    
    bytesSent += chunkSize;
    packetNum++;
    delay(PACKET_DELAY_MS);  // Delay between packets
  }
  
  // Send end packet to signal completion
  Serial.println("Sending end packet...");
  uint8_t endPkt[6];
  endPkt[0] = 0xEE;  // End packet magic bytes
  endPkt[1] = 0xEE;
  endPkt[2] = packetNum & 0xFF;  // Final packet count (little-endian)
  endPkt[3] = (packetNum >> 8) & 0xFF;
  endPkt[4] = 0xFF;  // Padding
  endPkt[5] = 0xFF;
  
  // Send end packet multiple times for reliability
  for (int i = 0; i < 3; i++) {
    sendPacketReliable(endPkt, 6);
    delay(200);
  }
  
  // Display transmission summary
  float duration = (millis() - startTime) / 1000.0;
  Serial.println("\n=== TRANSMISSION COMPLETE ===");
  Serial.print("Packets sent: ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(totalPackets);
  Serial.print(" (");
  Serial.print((float)successCount / totalPackets * 100, 1);
  Serial.println("% success rate)");
  
  Serial.print("Duration: ");
  Serial.print(duration, 1);
  Serial.println(" seconds");
  
  Serial.print("Data rate: ");
  Serial.print(capturedImageLength / duration, 0);
  Serial.println(" bytes/sec");
  
  // Quality assessment based on success rate
  if (successCount == totalPackets) {
    Serial.println("\n✅ Perfect transmission!");
  } else if (successCount > totalPackets * 0.95) {
    Serial.println("\n✅ Excellent transmission!");
  } else {
    Serial.println("\n⚠️ Some packets lost");
  }
}
