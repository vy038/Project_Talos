# Project Talos

Scorpion-style hexapod robot with a 3-DOF arm and computer vision. Wanted to build something that required real-time control, inverse kinematics, and vision processing on embedded hardware. Runs on dual ESP32s (WROOM for motors, S3 for camera) with bare metal C and FreeRTOS.

**Hardware:**
- 13 SG90 servos for legs and part of arm (tripod gait)
- 3 MG90S servos for the high-stress parts of arm
- 2x PCA9685 I2C servo controller (want to reduce to 1)
- MPU6050 IMU, VL53L0X distance sensor, OV2640 camera
- ACS712 current sensing
- 3S2P 18650 pack with BMS, UBEC for servo rail, buck converters for logic

**Firmware:**
- ESP-IDF with FreeRTOS
- Hardware I2C (burst mode for real-time walking)
- UART for ESP32-ESP32 comms
- Inverse kinematics for arm
- Basic color detection for vision

## Simulator

Lightsim is a lightweight browser-based simulator that compiles the real firmware C code natively on Linux using HAL stubs, then visualizes the robot via WebSocket.

```bash
cd sim/lightsim
./scripts/build.sh   # build C sim + install npm deps
./scripts/run.sh     # launch sim + server on :3000
```

Then open http://localhost:3000

Three view modes: **Debug** (servo/IMU/I2C dashboard), **2D** (top-down canvas), **3D** (Three.js with orbit camera). Includes D-pad/keyboard control, gait selector (tripod/wave/ripple), manual arm sliders, IK cursor mode, and a physics ball for interaction testing.

The simulator compiles unmodified firmware files (`main.c`, gait, balance, arm, IK) and stubs out the 6 hardware drivers (I2C, servo, MPU6050, ADC, UART, power monitor).

## Current status

Testing the arm and making walking gait better.

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
├── hardware/              # wiring, CAD
└── sim/lightsim/          # browser simulator (see Simulator section)
    ├── sim/               # C simulator + HAL stubs
    ├── server/            # Node.js bridge (Express + ws)
    ├── frontend/          # vanilla JS, Three.js
    └── scripts/           # build.sh, run.sh
```

## Goals

- 50Hz locomotion control
- <5ms IK solve time
- 10Hz vision processing
- Autonomous cube manipulation

Timeline: Dec 2025 - Apr 2026
