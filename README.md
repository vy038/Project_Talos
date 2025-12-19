# Project Talos



**Autonomous hexapod robot with 6-DOF vision-guided manipulator arm**



## 🎯 Overview



Autonomous scorpion-style hexapod robot featuring:

- 18-servo hexapod locomotion (tripod gait)

- 7-servo 6-DOF manipulator arm with inverse kinematics

- Dual ESP32 architecture (WROOM + S3 CAM)

- Computer vision with TensorFlow Lite

- Custom power management with current monitoring



## 🔧 Hardware



**Microcontrollers:**

- ESP32 WROOM (motor control)

- ESP32-S3 N16R8 CAM (vision processing)



**Actuators:**

- 18× SG90 micro servos (legs)

- 6× MG90S metal gear servos (arm)

- 1× MG90S gripper



**Sensors:**

- OV2640 camera (2MP)

- VL53L0X ToF distance sensor

- MPU6050 6-axis IMU

- ACS712 30A current sensor



**Power:**

- 3S2P 18650 battery pack (11.1V, ~5000mAh)

- 3S BMS (40A)

- UBEC 5V 7A (servo power)

- 2× LM2596 buck converters (3.3V logic, 5V camera)



## 💻 Firmware



**Architecture:**

- Bare metal C with ESP-IDF

- FreeRTOS multi-task design

- Hardware I2C for servo control (PCA9685)

- UART inter-MCU communication

- Inverse kinematics library for arm control



**Tasks:**

- Locomotion: 50Hz (tripod gait generation)

- Arm control: 20Hz (IK + servo commands)

- Vision: 10Hz (object detection)

- Power monitoring: 1Hz (current/voltage logging)



## 🛠️ Development Setup



**Prerequisites:**

- ESP-IDF v5.2+

- Python 3.8+

- Git



**Build:**

```bash

get_idf

cd Project_Talos

idf.py build

```



**Flash:**

```bash

idf.py -p /dev/ttyUSB0 flash monitor

```



## 📁 Project Structure

```

Project_Talos/

├── main/                   # Main application

│   ├── locomotion/        # Gait generation

│   ├── arm/               # IK + arm control

│   ├── vision/            # Camera + detection

│   └── power/             # Power management

├── components/            # Hardware drivers

│   ├── servo_control/    # PCA9685 I2C driver

│   └── power_monitor/    # ACS712 ADC driver

├── test/                  # Unit tests

├── docs/                  # Documentation

└── hardware/              # Wiring diagrams, CAD files

```



## 📊 Performance Targets



- Locomotion update rate: 50Hz ± 0.3Hz

- Arm IK solve time: <5ms

- Vision detection: 10Hz

- Autonomous success rate: >70%



## 📄 License



MIT License



## 📧 Contact



Victor - [GitHub](https://github.com/yourusername)



**Status:** In development (Dec 2024 - Apr 2025)





