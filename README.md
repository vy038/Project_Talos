# Project Talos

Scorpion-style hexapod robot with a 6-DOF arm and computer vision. Wanted to build something that required real-time control, inverse kinematics, and vision processing on embedded hardware. Runs on dual ESP32s (WROOM for motors, S3 for camera) with bare metal C and FreeRTOS.

**Hardware:**
- 17 SG90 servos for legs and part of arm (tripod gait)
- 4 MG90S servos for the high stress parts of arm
- 2x PCA9685 I2C servo controller
- MPU6050 IMU, VL53L0X distance sensor, OV2640 camera
- ACS712 current sensing
- 3S2P 18650 pack with BMS, UBEC for servo rail, buck converters for logic

**Firmware:**
- ESP-IDF with FreeRTOS
- Hardware I2C (burst mode for real-time walking)
- UART for ESP32-ESP32 comms
- Inverse kinematics for arm
- TensorFlow Lite Micro for vision

## Current status

Working through driver implementations. 3D printing needs to finish before I lose printer access Dec 19th. Most parts already arrived, JTAG debugger set up.

## Build/Flash

```bash
get_idf
cd Project_Talos
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Structure

```
Project_Talos/
├── main/
│   ├── locomotion/        # gait gen
│   ├── arm/               # IK + control
│   ├── vision/            # camera + detection
│   └── power/             # monitoring
├── components/
│   ├── drivers/           # peripheral drivers
│   └── ...
└── hardware/              # wiring, CAD
```

## Goals

- 50Hz locomotion control
- <5ms IK solve time
- 10Hz vision processing
- Autonomous cube manipulation

Timeline: Dec 2024 - Apr 2025