# Project Talos

![C](https://img.shields.io/badge/C-bare_metal-555?style=flat-square&logo=c&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-6_tasks-555?style=flat-square)
![ESP32](https://img.shields.io/badge/ESP32-dual_MCU-555?style=flat-square)
![Status](https://img.shields.io/badge/status-WIP-orange?style=flat-square)

Scorpion-style hexapod with a 3-DOF arm and computer vision. Wanted to build something that required real-time control, inverse kinematics, and vision processing on embedded hardware. Runs on dual ESP32s (WROOM for motors, S3 for camera) with bare metal C and FreeRTOS.

## Status

- Arm kinematics (tested, working)
- Walking gait (tested, working)
- Simulator is demoable (tripod/wave/ripple, arm IK, physics ball)
- Full RTOS architecture (not on hardware yet)
- Sensors (MPU6050, ACS712, ToF, camera) not wired up yet
- End-to-end autonomous demo (end goal)

---

## Simulator

**[Try it live →](https://huggingface.co/spaces/vy739/talos-lightsim)**

Lightsim is a browser-based simulator that compiles the real firmware C code natively on Linux using HAL stubs, then visualizes the robot over WebSocket. The actual `main.c`, gait, balance, arm, and IK run unmodified + the 6 hardware drivers (I2C, servo, MPU6050, ADC, UART, power monitor) are stubbed out.

```bash
cd sim/lightsim
./scripts/build.sh   # build C sim + install npm deps
./scripts/run.sh     # launch sim + server on :3000
```

Open `http://localhost:3000`

Three view modes: **Debug** (servo/IMU/I2C dashboard), **2D** (top-down canvas), **3D** (Three.js with orbit camera). D-pad/keyboard control, gait selector, manual arm sliders, IK cursor mode, physics ball.

---

## Hardware

| Component | Details |
|---|---|
| Legs | 6× SG90 + 6× MG90 servos (tripod gait) |
| Arm | 3× MG90S (high-torque joints) + 2× gripper SG90 |
| Servo control | 2× PCA9685 I2C, burst-mode writes |
| Vision MCU | ESP32-S3-CAM + OV2640 |
| Ranging | VL53L0X ToF |
| IMU | MPU6050 |
| Current sensing | ACS712 |
| Power | 3S2P 18650 + BMS, UBEC for servo rail, buck converters for logic |

---

## Firmware

- ESP-IDF with FreeRTOS
- Hardware I2C (burst mode for real-time walking)
- UART for ESP32↔ESP32 comms
- Inverse kinematics for arm
- HSV color blob detection for vision

### RTOS Architecture

I used FreeRTOS because concurrent tasks let everything run in real time. Instead of a linear system where tasks block each other, most of the state updates happen in parallel. Deciding what gets a task was simple: figure out what needs to run at the same time, assign priorities, map out shared resources, then wire up queues and semaphores where ordering matters.

```
┌─────────────────┐      ┌──────────────┐     ┌─────────────────┐
│  state_machine  │────▶│  uart_cam    │───▶│  ESP32-S3-CAM   │
│     P7 · 50ms   │      │  P5          │     │  vision task    │
└────────┬────────┘      └──────────────┘     └─────────────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌────────┐  ┌──────────┐
│ gait   │  │ arm_ctrl │   both require xI2CMutex → PCA9685
│ P5·20ms│  │  P6      │
└────────┘  └──────────┘
    ▲
┌───┴─────┐  ┌─────────────┐
│ balance │  │ power_mon   │  P1 normally, escalates to max on issue
│ P5·20ms │  │ P1 · 20ms   │
└─────────┘  └─────────────┘
```

**Shared IPC** — `task_config.h`
```c
QueueHandle_t     xCamFrameQueue;      // depth=1, stale frames are worse than no frames
QueueHandle_t     xPowerEventQueue;    // depth=4
SemaphoreHandle_t xArmSemaphore;       // binary, arm wakes only when state machine signals
SemaphoreHandle_t xI2CMutex;           // binary, guards PCA9685 bus
TaskHandle_t      xUartCamTaskHandle;  // direct notify via cam ping
```

### Tasks

**State machine** `P7 · 50ms` - manages everything. Pings camera each cycle, processes ball detection, decides gait state, signals the arm, handles emergency escalation from power monitor.

**UART cam** `P5` - woken via `xTaskNotify`, sends a framed ping to the S3, awaits the full packet, parses with `bUARTProtoFeedBuf`, pushes to `xCamFrameQueue`. Queue depth is 1 so stale frames get dropped automatically.

**Gait** `P5 · 20ms` - updates all 6 legs, supports tripod/wave/ripple, speed scales down as ball gets closer. Pulls balance corrections from the balance task each cycle. Requires I2C mutex to write to PCA9685.

**Balance** `P5 · 20ms` - reads MPU6050, runs a complementary filter over gyro + accel, computes per-leg knee corrections. Disabled gracefully if IMU isn't detected at boot.

**Arm control** `P6` - sleeps on `xArmSemaphore`. Wakes with ball coordinates from state machine, converts to arm-relative 3D, runs IK, incrementally drives toward target. Camera keeps feeding updated coordinates until gripper closes.

**Power monitor** `P1 · 20ms` - reads ACS712. Uses a consecutive-count filter (2–3 readings) to avoid false positives from servo inrush spikes. On a real fault, self-escalates to max priority so it can post to `xPowerEventQueue` immediately. Won't be starved behind gait.

---

## Walkthrough of Design

There are two ESP32s, each communicating via UART. One manages just the camera and ball detection, and the other manages everything else: state machine, servo commands, etc.

I decided to implement a FreeRTOS architecture because it allows for concurrent tasks, and lets everything run in real time. For example, rather than having a linear system where information is sent in a linear fashion, and tasks have to wait in order to be updated, in this task environment, most of it is updated in real time, and maximizes the use of the ESP32's cores. I needed something clean that works fast.

How I decided what gets a task is simple: I broke down the logic chain onto steps, and decided which of these steps needed to be run at the same time. From there, I was able to determine the priority of those tasks and what resources they shared, and also whether or not queues or semaphores are needed. I could also see what tasks depended on each other so there was some sort of linearity if needed, but others can run in real time as well.

### Task Breakdown

**State machine** - the essential task that manages everything in the system. It is the one that manages the camera ping every time it cycles through. It is the middle layer that manages the input from the camera and the power monitoring task. It decides what gait state for it to be in, decides positioning of arm, processes the ball detection data, and decides if the power is supposed to be in emergency state or not.

**UART cam** - simple: it will get called via `xUartCamTaskHandle` from the state machine, then it will send a distinct signal via UART to the ESP32-S3-CAM, and then it will await the full packet response, and parse with `bUARTProtoFeedBuf`. It will then send the information back into the state machine via a frame queue (only holds one frame to prevent stale frames) for it to process.

**Gait** - updates the gait state every 20ms to keep the gait up and running asynchronously while the state machine regularly changes the state. The update command decides gaits for legs, computes new targets and new states for each one. Gait has many types, including wave, ripple, and tripod, with speed being configurable in the update (speed should go down as ball gets closer). Requires I2C bus mutex in order to send commands.

**Balance** - can be enabled or disabled depending on whether the MPU6050 is detected at startup. It has the same priority as gait (5) and runs on the same 20ms cycle. It reads the MPU6050 and runs a complementary filter with gyro and accel to get pitch and roll, then computes per-leg knee corrections based on the tilt. Gait pulls those corrections directly on its next update, writing to its own internal struct and gait reads it. Some tolerance is built in so small tilts don't cause constant micro-adjustments.

**Arm** - needs a semaphore to be given from the state machine, where the task will then wake and take ball positioning info from the state machine, where it will then convert the coordinates relative to the arm, then process how to move the arm with IK. Because it has priority 6, it will have highest priority, and it will keep updating the arm to slowly move closer to target. Once target is reached, it will grab. Camera should also keep monitoring the ball and sending updated coordinates to the arm, where it will continuously update the IK of the arm and slowly move to the desired position. Requires I2C bus mutex in order to send commands.

**Power monitor** - runs in the background every 20ms at priority 1. It reads the ACS712 current sensor and uses a consecutive-count filter. To prevent false positives, it needs 2-3 high readings in a row before flagging a problem, which filters out normal servo inrush spikes that can read way above normal for a short burst on startup. When it does detect a real issue, it escalates its own priority up to the max so it can immediately post the emergency status to the queue without waiting behind any other task. The state machine picks it up at the top of its next 50ms cycle and transitions to EMERGENCY.

**Vision task (S3-CAM)** - will receive the unique ping, register that as a signal, and then run the task based on it. It will snap a picture quickly, then run the VL53L0X if available to sense accurate depth. It will first detect any red centroid objects in the frame, then try to estimate the difference from the center of the frame to the center of the centroid. Radius will be estimated based on a known value, then distance can be estimated. If the ball is right in front of the robot, VL53 is run so depth can be measured accurately. It will then put that into a packet form (`vUARTProtoBuildDetection`) to be sent to the waiting UART task on the ESP32-WROOM.

---

## Logic Flow

```
BOOT -> IDLE -> SEARCH -> APPROACH -> GRAB -> DONE
                                               |
                                          EMERGENCY (any state, power fault)
```

Robot boots up, the hardware is all initialized in main, and then all tasks are launched.

The state machine is where it starts, and initializes everything else in the tasks, calibrates balancing and others, then idles for a bit before entering search mode.

Once the frame is processed, it will enter searching mode, where the robot will try to find the ball by turning 360.

Gait is used heavily for all walking, and it has auto balance too to make sure the robot is always oriented upwards properly.

Once ball is located, it will transition to approaching, so the robot will walk over to it, and adjust if needed by turning. Speed scales down as it gets closer.

Once it's close enough, then the coordinate information of the ball is sent to the arm where it takes over and grabs it. Camera is also watching to make sure ball doesn't shift.

Arm grabs, lifts, and holds. Program is done, robot stays in DONE state.

There is also power monitoring at all times to halt the robot when it detects sustained overcurrent (multiple consecutive bad readings, not just a spike).

---

## Goals

- 50Hz locomotion control
- <5ms IK solve time
- 10Hz vision processing
- Autonomous ball retrieval

---

## Build / Flash

```bash
get_idf
cd firmware/esp
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Structure

```
Project_Talos/
├── firmware/
│   ├── esp/                        # WROOM firmware (motors, state machine)
│   │   └── main/
│   │       ├── main.c
│   │       ├── tasks/              # FreeRTOS task layer + task_config.h
│   │       ├── state_machine/
│   │       ├── locomotion/         # gait + balance
│   │       ├── arm/                # arm control + IK
│   │       ├── power/
│   │       └── uart_protocol/
│   └── espcam/                     # S3 firmware (camera + vision)
│       └── main/
│           ├── tasks/              # vision_task (ping-driven)
│           ├── vision/             # OV2640 + blob detection + VL53L0X
│           └── uart_protocol/
├── hardware/                       # wiring diagrams, CAD
└── sim/lightsim/                   # browser simulator
    ├── sim/                        # C simulator + HAL stubs
    ├── server/                     # Node.js bridge (Express + ws)
    ├── frontend/                   # vanilla JS, Three.js
    └── scripts/
```
