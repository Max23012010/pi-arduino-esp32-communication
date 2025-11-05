// ================= ESP32 Motor Controller Code =================
// This script receives movement commands from Arduino via UART Serial
// and drives the L298N motor driver to control DC motors or servos
// Author: Pi-Arduino-ESP32 Communication Project

// ================= PIN CONFIGURATION =================
// Motor Control Pins for L298N Driver
#define IN1 27   // Motor A direction control (Forward)
#define IN2 26   // Motor A direction control (Backward)
#define IN3 25   // Motor B direction control (Forward)
#define IN4 33   // Motor B direction control (Backward)

// PWM Speed Control Pins (Analog pins for speed control)
#define ENA 14   // Motor A speed control (PWM capable)
#define ENB 32   // Motor B speed control (PWM capable)

// PWM Configuration
#define PWM_CHANNEL_A 0    // PWM channel for Motor A
#define PWM_CHANNEL_B 1    // PWM channel for Motor B
#define PWM_FREQUENCY 1000 // PWM frequency in Hz
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// Motor Speed Values
#define MOTOR_FORWARD_SPEED 200   // Full forward speed (0-255)
#define MOTOR_TURN_SPEED 150      // Speed for turning movements (0-255)

// ================= GLOBAL VARIABLES =================
String receivedCommand = "";  // Command received from Arduino

// ================= SETUP FUNCTION =================
void setup() {
  // Initialize Serial communication with Arduino
  // Default UART0 (GPIO1=TX, GPIO3=RX)
  Serial.begin(9600);
  
  // Small delay to ensure stable serial connection
  delay(1000);
  
  // Initialize motor control pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configure PWM for speed control
  // ledcSetup(channel, frequency, resolution)
  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Attach PWM channels to GPIO pins
  // ledcAttachPin(pin, channel)
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
  
  // Ensure all motors are stopped at startup
  stopMotors();
  
  // Print initialization message
  Serial.println("\n========================================");
  Serial.println("\u2705 ESP32 Motor Controller Initialized");
  Serial.println("========================================");
  Serial.println("Waiting for commands from Arduino...");
  Serial.println("\ud83d\udcca Motor A: Pins IN1(27), IN2(26), ENA(14)");
  Serial.println("\ud83d\udcca Motor B: Pins IN3(25), IN4(33), ENB(32)");
  Serial.println("\ud83d\udcca L298N Motor Driver at 9600 baud");
  Serial.println("\ud83d\udcca Commands: f=forward, b=backward, l=left, r=right, s=stop");
  Serial.println("========================================\n");
}

// ================= MAIN LOOP =================
void loop() {
  // Check if data is available from Arduino
  if (Serial.available()) {
    // Read the incoming command character
    char incomingChar = Serial.read();
    
    // Only process valid command characters
    if (incomingChar == 'f' || incomingChar == 'b' || incomingChar == 'l' ||
        incomingChar == 'r' || incomingChar == 's') {
      
      receivedCommand = incomingChar;
      
      // Debug print
      Serial.print("\ud83d\udce9 [Arduino -> ESP32] Command: ");
      Serial.println(receivedCommand);
      
      // Execute the command
      executeCommand(receivedCommand);
    }
  }
  
  // Small delay to prevent overwhelming the serial port
  delay(10);
}

// ================= COMMAND EXECUTION FUNCTION =================
void executeCommand(String cmd) {
  """
  Execute the movement command received from Arduino
  
  Commands:
  'f' - Move Forward:   Both motors forward
  'b' - Move Backward:  Both motors backward
  'l' - Turn Left:      Left motor slower, right motor faster
  'r' - Turn Right:     Right motor slower, left motor faster
  's' - Stop:           Both motors stop
  """
  
  if (cmd == "f") {
    moveForward();
  }
  else if (cmd == "b") {
    moveBackward();
  }
  else if (cmd == "l") {
    turnLeft();
  }
  else if (cmd == "r") {
    turnRight();
  }
  else if (cmd == "s") {
    stopMotors();
  }
}

// ================= MOVEMENT FUNCTIONS =================

void moveForward() {
  """
  Move robot forward by rotating both motors in the forward direction
  """
  Serial.println("\ud83d\udd04 Action: FORWARD");
  
  // Motor A: Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Motor B: Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set motor speeds to maximum
  ledcWrite(PWM_CHANNEL_A, MOTOR_FORWARD_SPEED);
  ledcWrite(PWM_CHANNEL_B, MOTOR_FORWARD_SPEED);
  
  Serial.println("\u2705 Motors moving forward at" + String(MOTOR_FORWARD_SPEED) + " speed");
}

void moveBackward() {
  """
  Move robot backward by rotating both motors in the backward direction
  """
  Serial.println("\ud83d\udd04 Action: BACKWARD");
  
  // Motor A: Backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Motor B: Backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set motor speeds to maximum
  ledcWrite(PWM_CHANNEL_A, MOTOR_FORWARD_SPEED);
  ledcWrite(PWM_CHANNEL_B, MOTOR_FORWARD_SPEED);
  
  Serial.println("\u2705 Motors moving backward at " + String(MOTOR_FORWARD_SPEED) + " speed");
}

void turnLeft() {
  """
  Turn robot left by slowing left motor and speeding up right motor
  """
  Serial.println("\ud83d\udd04 Action: TURN LEFT");
  
  // Motor A (Left): Slower speed for tighter turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, MOTOR_TURN_SPEED);
  
  // Motor B (Right): Normal speed to maintain forward momentum
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, MOTOR_FORWARD_SPEED);
  
  Serial.println("\u2705 Turning LEFT: Left motor at " + String(MOTOR_TURN_SPEED) + 
                 ", Right motor at " + String(MOTOR_FORWARD_SPEED));
}

void turnRight() {
  """
  Turn robot right by slowing right motor and speeding up left motor
  """
  Serial.println("\ud83d\udd04 Action: TURN RIGHT");
  
  // Motor A (Left): Normal speed to maintain forward momentum
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, MOTOR_FORWARD_SPEED);
  
  // Motor B (Right): Slower speed for tighter turn
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, MOTOR_TURN_SPEED);
  
  Serial.println("\u2705 Turning RIGHT: Left motor at " + String(MOTOR_FORWARD_SPEED) + 
                 ", Right motor at " + String(MOTOR_TURN_SPEED));
}

void stopMotors() {
  """
  Stop all motors immediately
  """
  Serial.println("\u23f9️  Action: STOP ALL MOTORS");
  
  // All direction pins to LOW (no current)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // Set PWM to 0 (no power)
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  
  Serial.println("\u2705 All motors stopped");
}

// ================= WIRING REFERENCE =================
/*
 * L298N Motor Driver Pin Configuration:
 * 
 * L298N Pin   | ESP32 Pin | Purpose
 * ============|===========|================
 * IN1         | GPIO 27   | Motor A Direction (Forward)
 * IN2         | GPIO 26   | Motor A Direction (Backward)
 * IN3         | GPIO 25   | Motor B Direction (Forward)
 * IN4         | GPIO 33   | Motor B Direction (Backward)
 * ENA         | GPIO 14   | Motor A Speed Control (PWM)
 * ENB         | GPIO 32   | Motor B Speed Control (PWM)
 * GND         | GND       | Common Ground
 * 12V         | 12V Batt  | Motor Power Supply
 * 
 * Motor connections:
 * OUT1, OUT2  -> Motor A (Left or Front)
 * OUT3, OUT4  -> Motor B (Right or Rear)
 * 
 * Serial Communication:
 * UART0 RX (GPIO 3) <- Arduino TX (Pin 3)
 * UART0 TX (GPIO 1) -> Arduino RX (Pin 2)
 * 
 * Command Flow:
 * Raspberry Pi -> Arduino -> ESP32 -> L298N Driver -> DC Motors
 * 
 * Speed Values:
 * - MOTOR_FORWARD_SPEED = 200 (out of 255)
 * - MOTOR_TURN_SPEED = 150 (out of 255)
 * - Adjust these values to fine-tune motor performance
 */
