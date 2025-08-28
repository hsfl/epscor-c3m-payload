// Simplified Transmitter with UART (Essential Functions Only)
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF22.h>
#include <RHHardwareSPI1.h>

// Pin definitions
const uint8_t RPI_ENABLE = 36;
const uint8_t TRIGGER_PIN = 2;
// UART pins: Serial2 uses pins 7 (RX) and 8 (TX) automatically

// Radio pins and object
const int RADIO_CS = 38; 
const int RADIO_INT = 40;
RH_RF22 rf23(RADIO_CS, RADIO_INT, hardware_spi1);

uint16_t capturedImageLength = 0;

// Buffer for thermal image
const uint32_t MAX_IMG = 40000;
uint8_t imgBuf[MAX_IMG];

// Radio parameters
const uint8_t PACKET_DATA_SIZE = 45;
const uint16_t PACKET_DELAY_MS = 20;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n=== Teensy Flat Sat Transmitter - UART Version ===");
  
  // Power RPi
  pinMode(RPI_ENABLE, OUTPUT);
  digitalWrite(RPI_ENABLE, HIGH);
  
  // Configure pins
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  pinMode(13, OUTPUT); // LED for status
  
  // Initialize UART for RPi communication
  Serial2.begin(115200); // High-speed UART
  Serial.println("UART initialized at 115200 baud");
  
  initRadioForTransmit();
  
  Serial.println("\nWaiting 5 seconds for RPi boot...");
  delay(5000);
  
  Serial.println("\nCommands:");
  Serial.println(" 'u' - UART thermal capture (fast!)");
  Serial.println(" 'r' - Send captured image via radio");
  Serial.println("\nReady!");
}

void initRadioForTransmit() {
  Serial.println("Initializing radio for transmit...");
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  digitalWrite(30, LOW);  // RX_ON = LOW for transmit
  digitalWrite(31, HIGH); // TX_ON = HIGH for transmit
  delay(100);
  
  SPI1.setMISO(39);
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  
  if (!rf23.init()) {
    Serial.println("Radio init failed - continuing anyway");
  }
  
  rf23.setFrequency(433.0);
  rf23.setModemConfig(RH_RF22::GFSK_Rb9_6Fd45);
  rf23.setTxPower(RH_RF22_TXPOW_20DBM);
  rf23.setModeIdle();
  delay(100);
  Serial.println("Radio ready for transmit");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    
    Serial.print("\nCommand: ");
    Serial.println(cmd);
    
    switch (cmd) {
      case 'u':
      case 'U':
        captureThermalImageUART();
        break;
      case 'r':
      case 'R':
        sendViaRadio();
        break;
      default:
        Serial.println("Unknown command. Use 'u' for capture, 'r' for transmit");
    }
  }
}

void captureThermalImageUART() {
  Serial.println("\n--- UART THERMAL CAPTURE ---");
  Serial.println("Triggering RPi capture...");
  
  // Clear any pending UART data
  while (Serial2.available()) {
    Serial2.read();
  }
  
  // Send trigger to RPi
  digitalWrite(TRIGGER_PIN, LOW);
  delay(10);
  digitalWrite(TRIGGER_PIN, HIGH);
  
  Serial.println("Waiting for RPi processing...");
  delay(10000); // Wait for RPi to capture and process
  
  Serial.println("\n--- RECEIVING DATA VIA UART ---");
  
  // Wait for data to start arriving
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
  
  // Check magic bytes
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
  
  // Extract data length (little-endian)
  uint16_t dataLen = header[4] | (header[5] << 8);
  
  Serial.print("Expected data length: ");
  Serial.print(dataLen);
  Serial.println(" bytes");
  
  if (dataLen == 0 || dataLen > MAX_IMG) {
    Serial.println("ERROR: Invalid data length!");
    return;
  }
  
  // Receive image data
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
  
  // Quick validation of thermal data
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

bool sendPacketReliable(uint8_t* data, uint8_t len) {
  const int MAX_RETRIES = 3;
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    rf23.setModeIdle();
    delay(5);
    
    rf23.spiRead(RH_RF22_REG_03_INTERRUPT_STATUS1);
    rf23.spiRead(RH_RF22_REG_04_INTERRUPT_STATUS2);
    
    digitalWrite(13, HIGH);
    
    if (rf23.send(data, len)) {
      if (rf23.waitPacketSent(500)) {
        digitalWrite(13, LOW);
        return true;
      }
    }
    
    digitalWrite(13, LOW);
    if (retry < MAX_RETRIES - 1) {
      delay(50);
    }
  }
  return false;
}

void sendViaRadio() {
  if (capturedImageLength == 0) {
    Serial.println("No image captured yet!");
    return;
  }
  
  Serial.println("\n--- SENDING THERMAL IMAGE VIA RADIO ---");
  Serial.print("Image size: ");
  Serial.print(capturedImageLength);
  Serial.println(" bytes");
  
  // Calculate total packets
  uint16_t totalPackets = (capturedImageLength + PACKET_DATA_SIZE - 1) / PACKET_DATA_SIZE;
  Serial.print("Total packets: ");
  Serial.println(totalPackets);
  
  // Send header packet
  Serial.println("Sending header...");
  uint8_t header[10];
  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = capturedImageLength & 0xFF;
  header[3] = (capturedImageLength >> 8) & 0xFF;
  header[4] = totalPackets & 0xFF;
  header[5] = (totalPackets >> 8) & 0xFF;
  header[6] = 0xDE;
  header[7] = 0xAD;
  header[8] = 0xBE;
  header[9] = 0xEF;
  
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
  
  delay(1000); // Wait for receiver
  
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
    
    // Packet number
    packet[0] = packetNum & 0xFF;
    packet[1] = (packetNum >> 8) & 0xFF;
    
    // Copy data
    memcpy(&packet[2], &imgBuf[bytesSent], chunkSize);
    
    // Progress every 50 packets
    if (packetNum % 50 == 0) {
      float progress = (float)bytesSent / capturedImageLength * 100;
      Serial.print("Progress: ");
      Serial.print(progress, 1);
      Serial.println("%");
    }
    
    // Send packet with retry
    if (sendPacketReliable(packet, chunkSize + 2)) {
      successCount++;
    } else {
      failCount++;
    }
    
    bytesSent += chunkSize;
    packetNum++;
    delay(PACKET_DELAY_MS);
  }
  
  // Send end packet
  Serial.println("Sending end packet...");
  uint8_t endPkt[6];
  endPkt[0] = 0xEE;
  endPkt[1] = 0xEE;
  endPkt[2] = packetNum & 0xFF;
  endPkt[3] = (packetNum >> 8) & 0xFF;
  endPkt[4] = 0xFF;
  endPkt[5] = 0xFF;
  
  for (int i = 0; i < 3; i++) {
    sendPacketReliable(endPkt, 6);
    delay(200);
  }
  
  // Summary
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
  
  if (successCount == totalPackets) {
    Serial.println("\n✅ Perfect transmission!");
  } else if (successCount > totalPackets * 0.95) {
    Serial.println("\n✅ Excellent transmission!");
  } else {
    Serial.println("\n⚠️ Some packets lost");
  }
}

