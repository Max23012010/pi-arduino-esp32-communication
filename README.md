# Pi-Arduino-ESP32 Communication System

🤖 **A comprehensive robotics communication framework for Raspberry Pi, Arduino Uno, and ESP32 with motor control via L298N driver**

---

## 📌 Project Overview

This project provides a **complete three-tier communication system** that enables a Raspberry Pi to control robotic movements through an Arduino microcontroller and ESP32 module. It's perfect for:
- 🚀 Robotics enthusiasts
- 🎓 Educational IoT projects
- 🔬 DIY home automation systems
- 🛠️ Prototyping motor control solutions

**Made for everyone** - fully commented code and detailed documentation for all skill levels!

---

## 🔧 System Architecture

```
┌─────────────────────────┐
│   Raspberry Pi 4        │
│  (Master Controller)    │
│  Sends Commands via     │
│  Serial (USB)           │
└────────────┬────────────┘
             │ USB Cable @ 9600 baud
             ▼
┌─────────────────────────┐
│   Arduino Uno           │
│   (Bridge/Relay)        │
│   Receives commands     │
│   Forwards to ESP32     │
└────────────┬────────────┘
             │ Serial TX/RX @ 9600 baud
             │ (SoftwareSerial Pins 2,3)
             ▼
┌─────────────────────────┐
│   ESP32 Module          │
│   (Motor Controller)    │
│   Controls L298N Driver │
└────────────┬────────────┘
             │ GPIO pins 25-33
             ▼
┌─────────────────────────┐
│   L298N Motor Driver    │
│   Powers DC Motors &    │
│   Servo Motors via      │
│   12V external supply   │
└─────────────────────────┘
```

---

## 📂 Files in This Repository

### 1. **raspberry_pi_controller.py** 🐍
**Purpose:** Master controller for the entire system
- **Connection:** USB serial to Arduino Uno
- **Features:**
  - Interactive command menu
  - Real-time motor control (forward, backward, left, right, stop)
  - Serial communication debugging
  - User-friendly interface with emoji feedback
  - Graceful shutdown with motor stopping

**How to use:**
```bash
python3 raspberry_pi_controller.py
```

**Available Commands:**
- `f` - Move FORWARD
- `b` - Move BACKWARD  
- `l` - Turn LEFT
- `r` - Turn RIGHT
- `s` - STOP all motors
- `q` - QUIT program

**Requirements:**
```bash
pip install pyserial
```

---

### 2. **arduino_bridge.ino** 🔧
**Purpose:** Bridge communication between Raspberry Pi and ESP32
- **Receives:** Commands from Pi via USB Serial (baud: 9600)
- **Sends:** Commands to ESP32 via SoftwareSerial (baud: 9600)
- **Features:**
  - Bidirectional serial communication
  - Command validation (only accepts f, b, l, r, s)
  - Debug messages for monitoring
  - Error handling

**Wiring (Arduino Uno):**
| Arduino Pin | Connection | Purpose |
|-------------|-----------|----------|
| USB Port | Raspberry Pi USB | Serial communication |
| Pin 2 (RX) | ESP32 TX (GPIO 17) | Receive from ESP32 |
| Pin 3 (TX) | ESP32 RX (GPIO 16) | Send to ESP32 |
| GND | ESP32 GND | Common ground |

**Upload Instructions:**
1. Connect Arduino to PC via USB
2. Open Arduino IDE
3. Select Board: Arduino Uno
4. Select COM Port
5. Click Upload

---

### 3. **esp32_motor_controller.ino** 🎮
**Purpose:** Motor control logic - receives commands and drives motors
- **Receives:** Commands from Arduino via UART (baud: 9600)
- **Controls:** L298N motor driver for DC motors
- **Features:**
  - PWM speed control (0-255)
  - Differential speed for turning
  - Real-time motor monitoring
  - Comprehensive debug output

**Motor Control Pins (ESP32):**
| GPIO | Purpose | Motor |
|------|---------|-------|
| 27 | IN1 (Direction) | Motor A Forward |
| 26 | IN2 (Direction) | Motor A Backward |
| 25 | IN3 (Direction) | Motor B Forward |
| 33 | IN4 (Direction) | Motor B Backward |
| 14 | ENA (PWM Speed) | Motor A Speed |
| 32 | ENB (PWM Speed) | Motor B Speed |

**Motor Speeds:**
- **Forward/Backward:** 200/255 (Full power)
- **Turning:** 150/255 (Reduced power for precise turns)

**L298N Motor Driver Connections:**
| L298N Pin | ESP32 Pin | Purpose |
|-----------|-----------|----------|
| IN1 | GPIO 27 | Motor A Forward Control |
| IN2 | GPIO 26 | Motor A Backward Control |
| IN3 | GPIO 25 | Motor B Forward Control |
| IN4 | GPIO 33 | Motor B Backward Control |
| ENA | GPIO 14 | Motor A Speed (PWM) |
| ENB | GPIO 32 | Motor B Speed (PWM) |
| GND | GND | Common Ground |
| +12V | 12V Battery | Motor Power Supply |
| OUT1/OUT2 | Motor A | DC Motor Connection |
| OUT3/OUT4 | Motor B | DC Motor Connection |

---

## 🛠️ Hardware Requirements

### Essential Components:
- 🍓 Raspberry Pi 4 (with power supply)
- 🔴 Arduino Uno (with USB cable)
- 📱 ESP32 Development Board
- 🔌 L298N Dual Motor Driver Module
- ⚙️ 2x DC Motors (or equivalent)
- 🔋 12V Battery (for motor power)
- 🧵 Breadboard and jumper wires
- 🔌 USB cables (Pi power, Arduino programming)

### Optional Components:
- 📦 Servo motors for robotic arms
- 🎚️ Potentiometer for speed adjustment
- 💡 LED indicators for motor status
- 🔊 Buzzer for audio feedback

---

## 📋 Complete Wiring Diagram

### Power Distribution:
```
┌─ 5V/3A PSU ──┬──> Raspberry Pi
│              └──> Arduino 5V (optional)
│
└─ 12V Battery ──> L298N GND + 12V
```

### Serial Communication Chain:
```
Raspberry Pi ──[USB]──> Arduino Uno ──[Serial]──> ESP32
  /dev/ttyACM0         Pins 2,3              GPIO 16,17
   (9600 baud)         SoftwareSerial        UART0
                        (9600 baud)
```

### Motor Connections:
```
ESP32 GPIOs (27,26,25,33,14,32) ──> L298N (IN1,IN2,IN3,IN4,ENA,ENB)
                                     ↓
                                  Motors (via OUT pins)
```

---

## 🚀 Quick Start Guide

### Step 1: Prepare Hardware
1. Connect all components according to the wiring diagrams above
2. Ensure all grounds are connected
3. Test USB connections

### Step 2: Upload Arduino Code
1. Open Arduino IDE
2. Load `arduino_bridge.ino`
3. Upload to Arduino Uno

### Step 3: Upload ESP32 Code
1. Install ESP32 board in Arduino IDE
2. Load `esp32_motor_controller.ino`
3. Upload to ESP32

### Step 4: Run Raspberry Pi Controller
1. Install Python dependencies: `pip install pyserial`
2. Run: `python3 raspberry_pi_controller.py`
3. Use interactive menu to control motors

---

## 💡 How It Works

### Command Flow:
```
1. User enters command in Pi interface (e.g., 'f' for forward)
   ↓
2. Pi sends character 'f' via USB serial to Arduino
   ↓
3. Arduino receives 'f', validates it, forwards to ESP32
   ↓
4. ESP32 receives 'f', executes moveForward() function
   ↓
5. Motor pins (IN1,IN2,IN3,IN4) set to forward direction
   ↓
6. PWM pins (ENA, ENB) set to motor speed (200/255)
   ↓
7. L298N driver receives signals and powers motors
   ↓
8. Motors rotate and robot moves forward! 🚗
```

### Speed Control Logic:
**Forward/Backward:**
- Both motors: 200/255 speed
- Result: Straight movement

**Left Turn:**
- Left motor: 150/255 speed (slower)
- Right motor: 200/255 speed (normal)
- Result: Rotates left

**Right Turn:**
- Left motor: 200/255 speed (normal)
- Right motor: 150/255 speed (slower)
- Result: Rotates right

---

## 🔍 Debugging Tips

### Issue: Arduino not connecting
- **Solution:** Check USB cable, run `ls /dev/tty*` on Pi
- Default port: `/dev/ttyACM0` or `/dev/ttyUSB0`

### Issue: Motors not moving
- **Solution:** Check L298N power connections (12V battery)
- Verify GPIO pin connections match code
- Test motor connections directly with battery

### Issue: Serial communication errors
- **Solution:** Ensure all baud rates are 9600
- Check GND connections between all devices
- Verify UART pins (Arduino 2,3 and ESP32 RX/TX)

### Debug Mode:
Uncomment debug lines in code to see serial output:
```cpp
Serial.println("Debug: Motor A speed = " + String(speed));
```

---

## 📚 Code Comments

**All code files include:**
- 📝 Section headers with clear descriptions
- 💬 Inline comments explaining complex logic
- 📋 Function documentation with parameters
- 🔗 Wiring reference diagrams
- ⚠️ Important warnings and notes

**Perfect for learning and customization!**

---

## 🎯 Common Modifications

### Increase Motor Speed:
```cpp
#define MOTOR_FORWARD_SPEED 255  // Maximum (was 200)
```

### Adjust Turning Speed:
```cpp
#define MOTOR_TURN_SPEED 100     // Tighter turns (was 150)
```

### Change Baud Rate (all three files):
```cpp
Serial.begin(115200);  // Faster communication (was 9600)
```

### Add Servo Support:
```cpp
#include <Servo.h>
Servo servo1;
servo1.attach(GPIO_PIN);
servo1.write(90);  // Set position
```

---

## 📖 Additional Resources

- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [Arduino Reference](https://www.arduino.cc/reference/)
- [ESP32 Documentation](https://docs.espressif.com/)
- [L298N Motor Driver Guide](https://lastminuteengineers.com/l298n-dc-motor-driver-arduino-tutorial/)
- [PySerial Documentation](https://pyserial.readthedocs.io/)

---

## 🤝 Contributing

Contributions are welcome! Feel free to:
- 🐛 Report bugs via Issues
- 💡 Suggest improvements
- 🔧 Submit pull requests
- 📝 Improve documentation

---

## 📄 License

This project is open-source and available for educational and personal use.

---

## ❓ FAQ

**Q: Can I use different microcontrollers?**
A: Yes! Modify pin definitions to match your hardware.

**Q: How many motors can I control?**
A: L298N supports 2 motors. Add more modules for additional motors.

**Q: Can I add wireless control?**
A: Yes! Replace USB with Bluetooth/WiFi modules.

**Q: Is this safe for beginners?**
A: Yes! All code is heavily commented and step-by-step guides provided.

---

## 🎉 Getting Started

1. ⭐ Star this repository
2. 🍴 Fork to your account
3. 📥 Clone to your local machine
4. 📖 Follow the Quick Start Guide
5. 🚀 Build amazing robots!

**Happy Building! 🤖✨**
