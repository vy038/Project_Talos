/**
 * @file gait_generator.h
 * @brief 2-DOF hexapod gait generation
 *
 * Each leg has 2 servos: hip (forward/back) and knee (up/down).
 * This module coordinates all 12 leg servos for walking and turning.
 *
 * GAIT TYPES:
 *   TRIPOD  - 3 legs move at once (alternating triangles). Fastest.
 *   WAVE    - 1 leg at a time. Slowest, most stable.
 *   RIPPLE  - 2 legs at a time. Middle ground.
 *
 * HOW IT WORKS:
 * Time is a repeating cycle from 0.0 to 1.0. Each leg has a phase offset
 * determining when it swings. When a leg's local phase is in [0, duty_cycle]
 * its foot is in the air (swing). Otherwise its foot is on the ground pushing
 * the body (stance).
 *
 * Tripod: legs {0,2,4} offset=0.0, legs {1,3,5} offset=0.5, duty=0.5
 *   -> two alternating triangles, always 3 on ground
 *
 * TURNING:
 * Left and right sides get opposite hip directions.
 * Turn right: right legs push backward, left legs push forward.
 *
 * GAIT DIAGRAM (top view)
 *
 *       [3]                    [12]
 *          \                  /
 *           [0]----FRONT---[15]
 *            |              |
 *      [4]--[1]----BODY----[14]--[11]
 *            |              |
 *           [2]----REAR----[13]
 *          /                  \ 
 *       [5]                    [10]
 *
 *
 * Right side = legs 15,14,13. Left side = legs 0,1,2.
 */

#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "esp_err.h"
#include "servo.h"
#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>

#define NUM_LEGS    6
#define DOF_PER_LEG 2

#define HIP_NEUTRAL_DEG     90      // reference neutral used by compute_leg() — do not change
#define KNEE_NEUTRAL_DEG    90      // reference neutral used by compute_leg() — do not change

// Per-leg knee neutral positions (degrees).
// compute_leg() is written against KNEE_NEUTRAL_DEG=90. Each value here shifts
// the whole leg up/down from that reference — stance height, lift, everything
// moves together. Calibrate with TEST_KNEE_CALIB: note the angle where each
// foot just touches the ground, then set that as the value below.
#define KNEE_NEUTRAL_L0  103    // Leg 0: Front-Right  (knee ch12)
#define KNEE_NEUTRAL_L1  112   // Leg 1: Mid-Right    (knee ch11)
#define KNEE_NEUTRAL_L2  114   // Leg 2: Rear-Right   (knee ch10)
#define KNEE_NEUTRAL_L3  67    // Leg 3: Rear-Left    (knee ch5)
#define KNEE_NEUTRAL_L4  61    // Leg 4: Mid-Left     (knee ch4)
#define KNEE_NEUTRAL_L5  61    // Leg 5: Front-Left   (knee ch3)

// Per-leg hip neutral positions (degrees).
// compute_leg() is written against HIP_NEUTRAL_DEG=90. Each value here shifts
// the leg forward/back from that reference. Calibrate with TEST_KNEE_CALIB (hip
// mode): note the angle where each hip is centered straight out, then set below.
#define HIP_NEUTRAL_L0  110    // Leg 0: Front-Right  (hip ch15)
#define HIP_NEUTRAL_L1  85    // Leg 1: Mid-Right    (hip ch14)
#define HIP_NEUTRAL_L2  67    // Leg 2: Rear-Right   (hip ch13)
#define HIP_NEUTRAL_L3  101    // Leg 3: Rear-Left    (hip ch2)
#define HIP_NEUTRAL_L4  100    // Leg 4: Mid-Left     (hip ch1)
#define HIP_NEUTRAL_L5  85    // Leg 5: Front-Left   (hip ch0)

#define HIP_STRIDE_DEG      80      // hip swing amplitude (degrees). start small.
#define KNEE_LIFT_DEG       25      // how high foot lifts during swing

// #define HIP_STRIDE_DEG      10   // cut in half from 20
// #define KNEE_LIFT_DEG       15   // cut in half from 25

#define STEP_CYCLE_MS       600     // one complete step cycle (ms). 600-1000 typical.
#define GAIT_UPDATE_MS      20      // update interval. matches 50Hz servo PWM.

typedef enum {
    GAIT_TRIPOD,
    GAIT_WAVE,
    GAIT_RIPPLE,
} gait_type_t;

typedef enum {
    MOVE_STOP,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_TURN_LEFT,
    MOVE_TURN_RIGHT,
} move_command_t;

typedef struct {
    float hip_angle[NUM_LEGS];
    float knee_angle[NUM_LEGS];
} leg_angles_t;

/**
 * @brief Init gait system. Sets all legs to neutral. Call after PCA9685 init.
 * 
 * Initializes to tripod gait, stopped, with moderate speed. Call vGaitSetCommand
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xGaitInit(void);

/**
 * @brief Set movement command and speed
 * 
 * Sets the desired movement command (forward/back/turn/stop) and speed (0.0 to 1.0).
 * 
 * @param cmd what to do (forward, turn, stop)
 * @param speed 0.0 to 1.0. affects stride length and cycle speed.
 */
void vGaitSetCommand(move_command_t cmd, float speed);

/**
 * @brief Switch gait pattern. Resets phase.
 * 
 * Sets type of gait (tripod/wave/ripple). Each has different leg phase offsets and duty cycles.
 */
void vGaitSetType(gait_type_t type);

/**
 * @brief Apply computed angles to servos with retry logic
 * 
 * Takes computed leg angles and sends them to the servos. Retries on failure with I2C bus recovery.
 * 
 * @param angles target angles for all legs
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xApplyAngles(const leg_angles_t *angles);

/**
 * @brief Set servo angle with retry logic
 * 
 * Sets angle for a single servo channel, with retries and I2C bus recovery on failure. Used by xGaitUpdate to apply computed angles to servos.
 * 
 * @param channel servo channel
 * @param angle target angle
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xBodySetAngleWithRetry(uint8_t channel, uint8_t angle);

/**
 * @brief Call at GAIT_UPDATE_MS intervals. Advances phase, computes angles, writes servos.
 * @param knee_corrections optional per-leg offsets from balance control. NULL to skip.
 * 
 * Updates leg angles based on current command, speed, and gait pattern. Applies optional knee corrections from balance control.
 */
esp_err_t xGaitUpdate(const float *knee_corrections);

/**
 * @brief Move all legs to neutral standing position
 * 
 * Makes hexapod stand in neutral pose with all legs down. Gradually moves legs to avoid jerks. Call before starting to walk, or to reset pose.
 */
esp_err_t xGaitStandNeutral(void);

/**
 * @brief Get current computed leg angles (for debug/logging)
 * 
 * @return leg_angles_t current hip and knee angles for all legs. Note these are the computed angles before applying balance corrections, used for debugging and logging.
 */
leg_angles_t xGaitGetAngles(void);

#endif