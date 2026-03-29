# Project Talos (WIP)

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
├── firmware/
│   ├── esp/                        # WROOM firmware (motors, state machine)
│   │   ├── main/
│   │   │   ├── main.c              # hardware init, task launch
│   │   │   ├── tasks/              # FreeRTOS task layer
│   │   │   │   ├── task_config.h   # shared IPC handles (queues, semaphores)
│   │   │   │   ├── state_machine_task.c
│   │   │   │   ├── gait_task.c
│   │   │   │   ├── balance_task.c
│   │   │   │   ├── arm_ctrl_task.c
│   │   │   │   ├── uart_cam_task.c
│   │   │   │   └── power_mon_task.c
│   │   │   ├── state_machine/      # behavior logic
│   │   │   ├── locomotion/         # gait + balance
│   │   │   ├── arm/                # arm control + IK
│   │   │   ├── power/              # power management
│   │   │   └── uart_protocol/      # shared packet format
│   │   └── components/             # hardware drivers (I2C, UART, servo, MPU6050, ADC, power_monitor)
│   └── espcam/                     # S3 firmware (camera + vision)
│       └── main/
│           ├── main.c
│           ├── tasks/              # vision_task (ping-driven)
│           ├── vision/             # OV2640 capture + red ball detection + VL53L0X
│           └── uart_protocol/      # shared packet format
├── hardware/                       # wiring diagrams, CAD
└── sim/lightsim/                   # browser simulator (see Simulator section)
    ├── sim/                        # C simulator + HAL stubs
    ├── server/                     # Node.js bridge (Express + ws)
    ├── frontend/                   # vanilla JS, Three.js
    └── scripts/                    # build.sh, run.sh
```

## Walkthrough of Design

There are two ESP32s, each communicating via UART. One manages just the camera and ball detection, and the other manages everything else: state machine, servo commands, etc. 

I decided to implement a FreeRTOS architecture because it allows for concurrent tasks, and lets everything run in real time. For example, rather than having a linear system where information is sent in a linear fashion, and tasks have to wait in order to be updated, in this task environment, most of it is updated in real time, and maximizes the use of the ESP32's cores. I needed something clean that works fast.

How I decided what gets a task is simple: I broke down the logic chain onto steps, and decided which of these steps needed to be run at the same time. From there, I was able to determine the priority of those tasks and what resources they shared, and also whether or not queues or semaphores are needed. i could also see what tasks depended on each other so there was some sort of linearity if needed, but others can run in real time as well.

TASKS:

The state machine is the essential task that manages everything in the system. It is the one that manages the camera ping every time it cycles through. It is the middle layer that manages the input from the camera and the power monitoring task. It decides what gait state for it to be in, decides positioning of arm, processes the ball detection data, and decides if the power is supposed to be in emergency state or not.

The UART cam task is simple: it will get called via xUartCamTaskHandle from the state machine, then it will send a distinct signal via UART to the ESP32 S3 Cam, and then it will await the full packet response, and parse with bUARTProtoFeedBuf. It will then send the information back into the state machine via a frame queue (only holds one frame to prevent stale frames) for it to process.

The gait task is also relatively simple. This task simply updates the gait state every 20ms to keep the gait up and running asynchronously whole the state machine regularly changes the state. The update command decides gaits for legs, computes new targets and new states for each one. Gait has many types, including wave, ripple, and tripod, with speed being configurable in the update (speed should go down as ball gets closer). Requires I2C bus mutex in order to send commands.

The balance task can be enabled or disabled depending on whether the MPU6050 is detected at startup. It has the same priority as gait (5) and runs on the same 20ms cycle. It reads the MPU6050 and runs a complementary filter with gyro and accel to get pitch and roll, then computes per-leg knee corrections based on the tilt. Gait pulls those corrections directly on its next update, writing to its own internal struct and gait reads it. Some tolerance is built in so small tilts don't cause constant micro-adjustments.

The arm needs a semaphore to be given from the state machine, where the task will then wake and take ball positioning info from the state machine, where it will then convert the coordinates relative to the arm, then process how to move the arm with IK. Because it has priority 6, it will have highest priority, and it will keep updating the arm to slowly move closer to target. Once target is reached, it will grab. Camera should also keep monitoring the ball and sending updated coordinates to the arm, where it will continuously update the ik of the arm and slowly move to the desired position. Requires I2C bus mutex in order to send commands.

Power monitor runs in the background every 20ms at priority 1. It reads the ACS712 current sensor and uses a consecutive-count filter. To prevent false positives, it needs 2-3 high readings in a row before flagging a problem, which filters out normal servo inrush spikes that can read way above normal for a short burst on startup. When it does detect a real issue, it escalates its own priority up to the max so it can immediately post the emergency status to the queue without waiting behind any other task. The state machine picks it up at the top of its next 50ms cycle and transitions to EMERGENCY.

The sending image function on the camera will receive the unique ping, register that as a signal, and then run that task based on it. It will snap a picture quickly, then run the VL53L0X if available to sense accurate. It will first detect any red centroid objects in the frame, then try to estimate difference from the center of the frame to the center of the centroid. Radius will be estimated based on a known value, then distance can be estimated. If ball is right in front of robot, Vl53 is run, so depth can be measured accurately. It will then put that into a packet form (vUARTProtoBuildDetection) to be sent to the waiting UART task on the ESP32 WROOM. 

## Logic Flow

Robot boots up, the hardware is all initialized in main, and then all tasks are launched. 

The state machine is where it starts, and initializes everything else in the tasks, calibrates balancing and others, then idles for a bit before entering search mode.

Once the frame is processed, it will enter searching mode, where the robot will try to find the ball by turning 360. 

Gait is used heavily for all walking, and it has auto balance too to make sure the robot is always oriented upwards properly.

Once ball is located, it will transition to approaching, so the robot will walk over to it, and adjust if needed by turning. Speed scales down as it gets closer.

Once its close enough, then the coordinate information of the ball is sent to the arm where it takes over and grabs it. Camera is also watching to make sure ball doesn't shift.

Arm grabs, lifts, and holds. Program is done, robot stays in DONE state.

There is also power monitoring at all times to halt the robot when it detects sustained overcurrent (multiple consecutive bad readings, not just a spike).

## Goals

- 50Hz locomotion control
- <5ms IK solve time
- 10Hz vision processing
- Autonomous cube manipulation

Timeline: Dec 2025 - Apr 2026
