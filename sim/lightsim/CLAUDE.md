# Lightsim - Project Talos Robot Simulator

Lightweight simulator for the Project Talos hexapod robot. Compiles real firmware
C code natively on Linux with HAL stubs, visualizes in browser via WebSocket.

## Quick Start

```bash
./scripts/build.sh    # Build C simulator + install npm deps
./scripts/run.sh      # Build and launch (sim + server on :3000)
```

Then open http://localhost:3000

## Architecture

```
Browser (Three.js / Canvas / Debug Dashboard)
        |  WebSocket (JSON, 50Hz)
Node.js Bridge Server (Express + ws)     [server/]
        |  JSON lines over stdin/stdout pipe
Native C Simulator Process               [sim/]
  ├── Unmodified firmware logic from Project Talos
  ├── HAL stub layer (replaces hardware drivers)
  └── Simulation engine (state aggregation + JSON)
```

## Three Modes

1. **Debug / Flow** - JTAG-like dashboard: servo bars, I2C bus monitor, IMU data
2. **2D View** - Top-down canvas: body hexagon, leg lines, arm chain
3. **3D View** - Three.js: procedural geometry robot with orbit camera

## Directory Structure

- `sim/hal_stubs/` - ESP-IDF shim headers + driver stub implementations
- `sim/engine/` - State tracking, JSON serialization
- `sim/main_sim.c` - Simulator entry point (wraps firmware's app_main)
- `server/` - Node.js bridge server (Express + ws)
- `frontend/` - Browser frontend (vanilla JS, no bundler)
- `scripts/` - Build and run scripts

## Key Principles

- **Zero modification to firmware files** — fix compilation issues in stubs only
- **Lightweight** — no heavy frameworks, no physics engines
- **stdout = JSON, stderr = logs** — clean separation for bridge server piping

## Coding Conventions

- **C code**: Match Project Talos style — `xFunctionName` returns, `snake_case` locals
- **JS code**: Vanilla ES6+, no frameworks, Three.js via CDN import
- **CSS**: Dark theme, monospace font

## Firmware Source Reference

Firmware lives at `../Project_Talos/firmware/esp/`. The CMakeLists.txt
references these files directly — they compile unmodified.

### Stubbed (6 drivers):
- `components/I2C/i2c.c` → `sim/hal_stubs/i2c_stub.c`
- `components/servo/servo.c` → `sim/hal_stubs/servo_stub.c`
- `components/MPU6050/mpu6050.c` → `sim/hal_stubs/mpu6050_stub.c`
- `components/ADC/adc_helpers.c` → `sim/hal_stubs/adc_stub.c`
- `components/UART/uart.c` → `sim/hal_stubs/uart_stub.c`
- `components/power_monitor/power_monitor.c` → `sim/hal_stubs/power_monitor_stub.c`

### Compiled unmodified:
- `main/main.c`, `main/locomotion/gait_generator.c`, `main/locomotion/balance_control.c`
- `main/arm/arm_control.c`, `main/arm/ik_solver.c`
- All test files in `tests/`

---

## Session Evolution & Detailed Context Log

This section documents the transformation of Lightsim during this session, from a basic hexagon model to a high-fidelity scorpion-ant hybrid with physical interactions.

### 1. Geometry & Visual Overhaul
- **Body Redesign**: Changed from a simple hexagon to a `40x15x80` rectangular prism for a more realistic crawler chassis.
- **Ant-Like Stance**: 
    - Legs L1-L3 (Left) and L4-L6 (Right) placed at sweeping angles (+30°, 0°, -30°).
    - Introduced a two-part **Hip Geometry**: a flat horizontal segment followed by a steep 0.8 rad diagonal tilt.
- **Exaggerated Leg Bends**:
    - Knee joint positioned at the peak of the diagonal hip.
    - **Split-Shin Geometry**: The shin is now two distinct parts. The upper shin flares outward aggressively (1.25 rad Z-axis), while the lower shin rotates back to stay perfectly vertical.
- **Scorpion Arm**: 
    - Increased length by 150% (shoulder 90px, forearm 75px).
    - Tilted the entire base **45 degrees forward** to reach over the body.
    - Added +50°/+60° forward bias to joints (initially, then refined) for an aggressive stance.

### 2. Physics & Stability Engine
- **Jitter Reduction (LERP)**: All servos (legs and arm) pass through a 10% per-frame LERP filter (`smoothed = lerp(current, target, 0.1)`). This fixed the "superposition" glitching and high-frequency twitching.
- **Zero-Degree Falsy Bug**: Fixed a critical JS bug where `0` degree angles were caught by `|| 90` falsy checks, causing legs to snap to 90 degrees.
- **Dynamic Body Gravity**: Implemented a real-time foot-collision system. It calculates the world-position Y of all 6 feet; if the lowest foot is above/below the floor (`y=3`), the entire body position is adjusted to keep it resting on the ground.
- **Differential Drive Odometry**: Walking movement is estimated based on hip changes during stance phase. Turning is computed via the difference between left and right side movements.
- **Friction**: Added friction and odometry deadbands to stop the robot from "sliding" while standing still or twitching.

### 3. Interactive Features
- **Physics Ball**: 
    - Added a ball with real gravity, floor bouncing, air drag, and body-push collision.
    - **Raycaster Interaction**: Implemented mouse click-drag-and-throw. Clicking the ball grabs it; moving projects it on a plane; releasing it applies accumulated velocity.
- **Manual Arm Control**: Added a "Manual Override" mode with 4 range sliders (Base/Shoulder/Elbow/Gripper) and live degree readouts.
- **Movement & Gait**: Added a D-pad for Forward/Back/Turn/Stop commands, a Speed slider, and a Gait Type selector (Tripod/Wave/Ripple).

### 4. Layout & UI Evolution
- **The "Invisible Controls" Incident**: Discovered that `debugview.js` was overwriting the entire `#mode-debug` container's `innerHTML`, effectively deleting the sidebar controls.
- **Redesign**: Overhauled the frontend into a **Split-Screen Panel System**.
    - **Panels**: Controls, Debug, 3D View, and 2D View now live in a flex row.
    - **Toggles**: Top-bar buttons toggle panels on/off; multiple panels can be active side-by-side.
    - **Default Config**: Controls sidebar + Debug info + 3D View all visible at once.

### 5. Known Gotchas & Bugs Fixed
- **Three.js Import map**: Fixed "three" bare specifier resolution by adding an importmap to `index.html`.
- **Gimbal Lock**: Clamped `OrbitControls` polar angles (0.05 to PI-0.05) to prevent camera flips when looking directly up or down.
- **Resize Jitter**: Added `window.dispatchEvent(new Event('resize'))` to panel toggles to ensure Three.js/Canvas elements always fill their containers.
- **Keyboard Shortcuts**: Added Arrow keys for movement and Space for stop.

### 6. Session 2 - Physics, Gait & IK Overhaul
- **2D View Fix**: Fixed `mode-2d` → `pane-view2d` element reference that was breaking the 2D canvas render loop.
- **Client-Side Gait Generator**: D-pad/keyboard now drives a JS-based gait engine (tripod/wave/ripple) that overrides firmware leg values via `window.GaitControl`. Supports forward/backward/turn with speed control.
- **Full Robot Hitboxes**: Ball now collides with every robot mesh (body, all hip/shin segments, arm segments, gripper) using bounding-sphere collision with velocity reflection and restitution coefficient (0.6).
- **Ball Drop Placement**: "Click to Place" toggle lets users click anywhere in the 3D view to drop the ball at that world position. Shift+drag changes ball height.
- **IK Cursor Mode**: Draggable 3D gizmo (cyan sphere + RGB axis lines) at the gripper tip. When dragged, a JS IK solver (port of firmware `ik_solver.c` with sim dimensions: L1=90, L2=75) computes joint angles. Shows workspace sphere wireframe, reachability color (green/red), relative position readout, and auto-updates arm sliders.
- **Server Endpoints**: Added `/api/command` and `/api/test` endpoints to the bridge server.
- **Controls Panel**: Widened from 210px to 280px for better slider/readout visibility.
