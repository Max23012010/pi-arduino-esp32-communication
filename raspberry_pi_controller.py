#!/usr/bin/env python3
# ================= Raspberry Pi 4 Master Controller =================
# This script sends movement commands to the Arduino Uno via USB serial connection
# The Arduino forwards commands to ESP32 which controls motors and servos
# Author: Pi-Arduino-ESP32 Communication Project

import serial
import time
import sys

# ================= CONFIGURATION =================
# Serial port configuration - adjust based on your system
# For Raspberry Pi: typically /dev/ttyACM0 (Arduino USB) or /dev/ttyUSB0
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
CONNECTION_TIMEOUT = 2

# ================= INITIALIZE CONNECTION =================
def initialize_arduino_connection():
    """
    Initialize serial connection to Arduino Uno
    Returns: serial connection object if successful, None otherwise
    """
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=CONNECTION_TIMEOUT)
        time.sleep(2)  # Wait for Arduino to reset after connection
        print("✅ Successfully connected to Arduino at", SERIAL_PORT)
        print(f"📊 Baud Rate: {BAUD_RATE}")
        return arduino
    except serial.SerialException as e:
        print(f"❌ Failed to connect to Arduino: {e}")
        print("📌 Make sure:")
        print("   - Arduino is connected via USB")
        print("   - Port is correct (check with: ls /dev/ttyACM* or ls /dev/ttyUSB*)")
        print("   - You have permission to access the serial port")
        return None

# ================= COMMAND FUNCTIONS =================
def send_command(arduino, cmd):
    """
    Send a command to Arduino
    
    Available commands:
    - 'f' : Move forward
    - 'b' : Move backward
    - 'l' : Turn left
    - 'r' : Turn right
    - 's' : Stop all motors
    
    Args:
        arduino: Serial connection object
        cmd (str): Command character to send
    """
    if not arduino:
        print("❌ No Arduino connection")
        return False
    
    try:
        # Encode and send command
        arduino.write(cmd.encode())
        print(f"📤 Command sent: '{cmd}'")
        
        # Optional: Read response from Arduino
        if arduino.in_waiting:
            response = arduino.readline().decode('utf-8').strip()
            if response:
                print(f"📩 Response: {response}")
        
        return True
    except Exception as e:
        print(f"❌ Error sending command: {e}")
        return False

def move_forward(arduino):
    """Move robot forward"""
    print("🔄 Moving FORWARD...")
    return send_command(arduino, 'f')

def move_backward(arduino):
    """Move robot backward"""
    print("🔄 Moving BACKWARD...")
    return send_command(arduino, 'b')

def turn_left(arduino):
    """Turn robot left"""
    print("🔄 Turning LEFT...")
    return send_command(arduino, 'l')

def turn_right(arduino):
    """Turn robot right"""
    print("🔄 Turning RIGHT...")
    return send_command(arduino, 'r')

def stop_motors(arduino):
    """Stop all motors"""
    print("⏹️  STOPPING all motors...")
    return send_command(arduino, 's')

# ================= INTERACTIVE MENU =================
def display_menu():
    """
    Display available commands to user
    """
    print("\n" + "="*50)
    print("🤖 Pi-Arduino-ESP32 Robot Controller")
    print("="*50)
    print("Available Commands:")
    print("  [f] - Move FORWARD")
    print("  [b] - Move BACKWARD")
    print("  [l] - Turn LEFT")
    print("  [r] - Turn RIGHT")
    print("  [s] - STOP all motors")
    print("  [h] - Show this help menu")
    print("  [q] - QUIT the program")
    print("="*50 + "\n")

# ================= MAIN LOOP =================
def main():
    """
    Main program loop
    """
    print("🚀 Starting Pi Robot Controller...\n")
    
    # Initialize connection
    arduino = initialize_arduino_connection()
    if not arduino:
        print("\n❌ Failed to initialize. Exiting...")
        sys.exit(1)
    
    display_menu()
    
    try:
        while True:
            # Get user input
            user_input = input("\n➡️  Enter command (or 'h' for help): ").strip().lower()
            
            if not user_input:
                print("⚠️  No command entered, try again")
                continue
            
            # Process commands
            if user_input == 'f':
                move_forward(arduino)
            elif user_input == 'b':
                move_backward(arduino)
            elif user_input == 'l':
                turn_left(arduino)
            elif user_input == 'r':
                turn_right(arduino)
            elif user_input == 's':
                stop_motors(arduino)
            elif user_input == 'h':
                display_menu()
            elif user_input == 'q':
                print("\n👋 Shutting down... Stopping motors first!")
                stop_motors(arduino)
                break
            else:
                print(f"❌ Unknown command: '{user_input}'. Type 'h' for help.")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        stop_motors(arduino)
    
    finally:
        # Clean up
        if arduino:
            print("\n🔌 Closing serial connection...")
            arduino.close()
            print("✅ Connection closed")
        print("\n👋 Program terminated\n")

# ================= ENTRY POINT =================
if __name__ == "__main__":
    main()
