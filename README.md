# Crawling Bot V2 - PlatformIO Migration

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange.svg)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## 📋 Table of Contents
- [Overview](#overview)
- [Hardware Changes](#hardware-changes)
- [Architecture](#architecture)
- [Pin Configuration](#pin-configuration)
- [Dependencies](#dependencies)
- [Building and Uploading](#building-and-uploading)
- [Documentation](#documentation)
- [Quick Links](#quick-links)

## Overview
This is the PlatformIO version of the crawling bot, migrated from the original Arduino code. The code has been restructured with a clean, library-based architecture for better maintainability and modularity.

**Key Features:**
- 🖥️ OLED display with 128x64 resolution
- 🧭 9-DOF AHRS sensor (MPU9250)
- 🎮 Smooth servo control
- 📡 WiFi AP and OTA updates
- 🤖 Ready for reinforcement learning

## Hardware Changes
- **Display**: Migrated from LCD 16x2 (I2C) to OLED 0.96" SSD1306 display
- **Motion Sensor**: Replaced SRF-05 ultrasonic sensor with MPU9250 AHRS (Attitude and Heading Reference System)
- **Board**: ESP32-S3-DevKitM-1

## Architecture

### Libraries Structure
The project uses a modular library-based architecture with separate responsibilities:

#### 1. Display Library (`lib/Display/`)
Manages the OLED 0.96" display with SSD1306 driver.

**Features:**
- Initialize and configure OLED display
- Print text and numbers at specific coordinates
- Clear display
- Draw progress bars (for OTA updates)
- Cursor positioning

**Usage:**
```cpp
Display display;
display.begin();
display.print("Hello World", 0, 0);
display.clear();
```

#### 2. AHRS Library (`lib/AHRS/`)
Manages the MPU9250 sensor with full AHRS capabilities.

**Features:**
- 9-DOF sensor data (accelerometer, gyroscope, magnetometer)
- Orientation (roll, pitch, yaw)
- Quaternion data
- Motion detection
- Displacement tracking
- Temperature sensing
- Sensor calibration

**Usage:**
```cpp
AHRS ahrs;
ahrs.begin();
ahrs.update(); // Call frequently in loop

float roll = ahrs.getRoll();
float pitch = ahrs.getPitch();
float yaw = ahrs.getYaw();
bool moving = ahrs.isMoving();
```

#### 3. ServoControl Library (`lib/ServoControl/`)
Manages the two servo motors with smooth movement capabilities.

**Features:**
- Control two servos (down and up)
- Instant movement
- Smooth movement with configurable speed
- Position tracking
- Preset positions (initial, test)

**Usage:**
```cpp
ServoControl servos(PIN_DOWN, PIN_UP);
servos.begin();
servos.moveDownSmooth(90, 10);
servos.setInitialPosition();
```

#### 4. Network Library (`lib/Network/`)
Manages WiFi Access Point and OTA (Over-The-Air) updates.

**Features:**
- WiFi AP setup with robot-specific SSID
- OTA update handling with progress display
- Robot number management (EEPROM)
- Async OTA task using FreeRTOS
- mDNS hostname setup

**Usage:**
```cpp
Network network(&display);
network.begin();
network.startOTATask();
uint8_t robotNum = network.getRobotNumber();
```

#### 5. Training Library (`lib/Training/`)
**Currently empty** - Template for future implementation of reinforcement learning.

**Planned Features:**
- Training mode activation
- Model training and learning
- Learned behavior execution
- Model save/load functionality

**Usage:**
```cpp
Training training;
training.begin();
// Future: training.startTraining();
// Future: training.executeLearnedBehavior();
```

## Pin Configuration

```cpp
SERVO_PIN_DOWN = 32  // Lower servo motor
SERVO_PIN_UP = 33    // Upper servo motor
I2C SDA = Default ESP32 SDA pin
I2C SCL = Default ESP32 SCL pin
```

## Dependencies

Defined in `platformio.ini`:
- `adafruit/Adafruit SSD1306` - OLED display driver
- `adafruit/Adafruit GFX Library` - Graphics library
- `adafruit/Adafruit BusIO` - I2C/SPI support
- `hideakitai/MPU9250` - AHRS sensor library
- `madhephaestus/ESP32Servo` - ESP32 servo control

## Building and Uploading

```bash
# Build the project
pio run

# Upload to board
pio run --target upload

# Monitor serial output
pio device monitor

# Build and upload
pio run --target upload && pio device monitor
```

## OTA Updates

After initial upload, you can update wirelessly:
1. Robot creates AP with SSID: `ESP32-AP-{robot_number}`
2. Password: `12345678`
3. OTA hostname: `ESP32-OTA-{robot_number}`
4. Use PlatformIO OTA or Arduino IDE OTA to upload

## Robot Number Configuration

Each robot has a unique number (1-8) stored in EEPROM. On first boot:
1. Robot prompts for number via Serial (115200 baud)
2. Enter number 1-8
3. Number is saved and used for AP SSID and OTA hostname

## Health Check

On startup, the robot performs a health check:
1. Tests AHRS sensor (displays orientation)
2. Tests servo motors (moves to test position and back)
3. Displays status on OLED

## Future Development

The Training library is designed to be filled with:
- Reinforcement learning algorithms
- Neural network integration
- Behavior learning and execution
- Model persistence

## Migration Notes

Changes from original Arduino code:
- ✅ LCD → OLED display
- ✅ SRF-05 → MPU9250 AHRS
- ✅ Monolithic code → Library-based architecture
- ✅ Enhanced AHRS capabilities (9-DOF, quaternions, calibration)
- ✅ Improved code organization and maintainability
- ⏳ Training/Learning (template ready for implementation)

## Documentation

This project includes comprehensive documentation:

### 📚 Quick Links

| Document | Description |
|----------|-------------|
| [QUICKSTART.md](QUICKSTART.md) | Get started in 5 minutes |
| [BUILD.md](BUILD.md) | Detailed build and deployment guide |
| [MIGRATION.md](MIGRATION.md) | Migration guide from Arduino to PlatformIO |
| [EXAMPLES.md](EXAMPLES.md) | Code examples for all libraries |
| [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) | Complete project overview |

### 📊 Project Statistics

- **Code**: ~923 lines (C++/Arduino)
- **Documentation**: ~1,373 lines (Markdown)
- **Libraries**: 5 modular components
- **Examples**: 15+ code snippets

## Contributing

Contributions are welcome! Please:
1. Follow the existing code structure
2. Keep libraries independent and focused
3. Update documentation for any API changes
4. Add examples for new features
5. Test on actual hardware

## License

[Specify your license here]

## Credits

- **Original Arduino Code**: amirali-lll
- **PlatformIO Migration**: GitHub Copilot
- **Libraries**: Adafruit, hideakitai, madhephaestus

---

**Status**: ✅ Migration Complete - Ready for Hardware Testing  
**Version**: 2.0  
**Last Updated**: December 2025
