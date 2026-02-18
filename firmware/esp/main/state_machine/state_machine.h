/**
 * @file state_machine.h
 * @brief Top-level robot behavior controller
 *
 * Decides WHAT the robot does, not HOW. Transitions between states and
 * sends commands to gait/arm/balance modules.
 *
 * STATE FLOW:
 *   INIT -> CALIBRATE -> IDLE -> SEARCH (rotate looking for ball)
 *     -> ALIGN (turn to center ball in frame)
 *     -> APPROACH (walk forward toward ball)
 *     -> GRAB_PREP (stop, stabilize, lower arm)
 *     -> GRAB (close gripper)
 *     -> LIFT (raise arm)
 *     -> DONE (hold)
 *   Any state -> EMERGENCY (overcurrent or severe tilt)
 *
 * UART PROTOCOL FROM ESP32-S3:
 * S3 processes camera frames, sends detection results over UART.
 * Fixed-length packet, 11 bytes:
 *   [0xAA] [0x55] [type=0x01] [detected] [x_hi] [x_lo] [y_hi] [y_lo] [r_hi] [r_lo] [checksum]
 *   checksum = XOR of bytes 2-9
 *
 * TODO: implement the matching sender on the S3 side.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// must match S3 camera config (QVGA)
#define CAM_FRAME_WIDTH     320
#define CAM_FRAME_HEIGHT    240

// if ball center is within this many px of frame center, consider it "centered"
#define BALL_CENTER_TOLERANCE_X     40

// ball radius in px that means "close enough to grab"
// TODO: calibrate by holding ball at grab distance and reading radius
#define BALL_CLOSE_RADIUS_PX        60

#define SEARCH_TIMEOUT_MS           10000   // how long to search before resetting timer
#define GRAB_PREP_PAUSE_MS          1500    // let robot stabilize before grabbing

// UART protocol constants
#define UART_MSG_START_0        0xAA
#define UART_MSG_START_1        0x55
#define UART_MSG_TYPE_DETECT    0x01
#define UART_MSG_LENGTH         11

typedef struct {
    bool detected;
    uint16_t ball_x;        // 0=left, 319=right
    uint16_t ball_y;        // 0=top, 239=bottom
    uint16_t ball_radius;   // apparent size in px
    bool fresh;             // updated since last read?
} detection_result_t;

typedef enum {
    STATE_INIT,
    STATE_CALIBRATE,
    STATE_IDLE,
    STATE_SEARCH,
    STATE_APPROACH,
    STATE_ALIGN,
    STATE_GRAB_PREP,
    STATE_GRAB,
    STATE_LIFT,
    STATE_DONE,
    STATE_EMERGENCY,
} robot_state_t;

/**
 * @brief Init state machine. Call after all subsystems are initialized.
 * 
 * Performs initial calibration and sets initial state. Returns error if calibration fails (e.g. MPU6050 not responding).
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t xStateMachineInit(void);

/**
 * @brief Run one iteration. Call from main control task at ~50ms interval.
 * 
 * Handles state transitions and calls appropriate handlers for each state. Also checks for emergency conditions (tipping) and transitions to EMERGENCY state if needed.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t xStateMachineUpdate(void);

/**
 * @brief Get current state (for telemetry/debugging)
 * 
 * @return current robot state
 */
robot_state_t xStateMachineGetState(void);

/**
 * @brief Force state transition (mainly for EMERGENCY)
 * 
 * Use with caution, can disrupt normal flow and cause unsafe conditions if used improperly. Designed for critical events like tipping or overcurrent where immediate state change is necessary.
 */
void vStateMachineForceState(robot_state_t new_state);

/**
 * @brief Feed detection result from UART RX task
 * 
 * Call this from the UART receive handler when a new detection message is parsed. This updates the internal state with the latest ball position and detection status, which will be used by the state handlers to make decisions.
 */
void vStateMachineFeedDetection(detection_result_t result);

/**
 * @brief Parse raw UART buffer for detection messages.
 *        Scans for start markers, validates checksum.
 * 
 * Designed to be called from the UART RX task with incoming data. If a valid message is found, fills the result struct and returns true. Otherwise returns false.
 * 
 * @param buf raw UART receive buffer
 * @param len bytes in buffer
 * @param result output: parsed detection
 * @return true if valid message found
 */
bool bStateMachineParseUART(const uint8_t *buf, size_t len, detection_result_t *result);

#endif