// ================= Arduino Uno Bridge Code =================
// This script receives commands from Raspberry Pi via USB serial
// and forwards them to ESP32 via UART (Serial1)
// It also reads responses from ESP32 and relays them back to Pi
// Author: Pi-Arduino-ESP32 Communication Project

// ================= CONFIGURATION =================
// SoftwareSerial for ESP32 communication
// RX pin (receive from ESP32) = Pin 2
// TX pin (transmit to ESP32) = Pin 3
#include <SoftwareSerial.h>

// Create SoftwareSerial instance for ESP32 communication
// RX, TX - connect to ESP32's TX, RX respectively
SoftwareSerial espSerial(2, 3);

// ================= GLOBAL VARIABLES =================
String piCommand = "";      // Command received from Raspberry Pi
String esp32Feedback = "";  // Feedback received from ESP32

// ================= SETUP FUNCTION =================
void setup() {
  // Initialize serial communication with Raspberry Pi
  // Baud rate: 9600 to match Pi settings
  Serial.begin(9600);
  
  // Initialize software serial for ESP32 communication
  espSerial.begin(9600);
  
  // Wait a moment for serial connections to stabilize
  delay(500);
  
  // Print initialization message
  Serial.println("\n========================================");
  Serial.println("\u2705 Arduino Uno Bridge Initialized");
  Serial.println("========================================");
  Serial.println("Waiting for commands from Raspberry Pi...");
  Serial.println("\ud83d\udcca Serial (Pi) @ 9600 baud");
  Serial.println("\ud83d\udcca SoftwareSerial (ESP32) @ 9600 baud");
  Serial.println("\ud83d\udcca Pin 2 (RX) <- ESP32 TX");
  Serial.println("\ud83d\udcca Pin 3 (TX) -> ESP32 RX");
  Serial.println("\ud83d\udcca GND connected to both");
  Serial.println("========================================\n");
}

// ================= MAIN LOOP =================
void loop() {
  // ===== RECEIVE FROM RASPBERRY PI =====
  // Check if data is available from Raspberry Pi on USB serial
  if (Serial.available()) {
    // Read a character from Pi
    char incomingChar = Serial.read();
    
    // Only process valid command characters
    if (incomingChar == 'f' || incomingChar == 'b' || incomingChar == 'l' ||
        incomingChar == 'r' || incomingChar == 's') {
      
      piCommand = incomingChar;
      
      // Debug: Print received command
      Serial.print("\ud83d\udce9 [Pi -> Arduino] Command: ");
      Serial.println(piCommand);
      
      // Forward command to ESP32
      espSerial.print(piCommand);
      Serial.println("\u27a1\ufe0f  [Arduino -> ESP32] Forwarded");
    }
  }
  
  // ===== RECEIVE FROM ESP32 =====
  // Check if data is available from ESP32
  if (espSerial.available()) {
    // Read data from ESP32
    esp32Feedback = espSerial.readStringUntil('\n');
    
    // Remove any extra whitespace
    esp32Feedback.trim();
    
    if (esp32Feedback.length() > 0) {
      // Debug: Print received feedback
      Serial.print("\ud83d\udce9 [ESP32 -> Arduino] Feedback: ");
      Serial.println(esp32Feedback);
      
      // Optional: Forward feedback to Pi
      // Uncomment the line below to send ESP32 responses back to Pi
      // Serial.print("\u2705 [Arduino -> Pi] ");
      // Serial.println(esp32Feedback);
    }
  }
  
  // Small delay to prevent overwhelming the serial ports
  delay(10);
}

// ================= COMMAND REFERENCE =================
/*
 * Available Commands (Character-based):
 * 'f' - Move Forward      -> Both motors forward
 * 'b' - Move Backward     -> Both motors backward
 * 'l' - Turn Left         -> Left motor slower, right motor faster
 * 'r' - Turn Right        -> Right motor slower, left motor faster
 * 's' - Stop All Motors   -> Both motors stop
 *
 * Wiring (Important!):
 * Arduino Pin 2 (RX)  <-- ESP32 TX (GPIO 17)
 * Arduino Pin 3 (TX)  --> ESP32 RX (GPIO 16)
 * Arduino GND         --- ESP32 GND
 * Arduino 5V          --- ESP32 5V (if needed, with voltage regulator)
 *
 * Serial Communication:
 * - Arduino <-> Raspberry Pi: USB cable at /dev/ttyACM0 (9600 baud)
 * - Arduino <-> ESP32: SoftwareSerial pins 2,3 (9600 baud)
 *
 * Data Flow:
 * Raspberry Pi -> Arduino (USB Serial) -> ESP32 (SoftwareSerial)
 *                                      -> Motors/Servos
 */
