// gait_generator.h
#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Leg segment lengths in mm - measure from CAD
#define LEG_COXA_LENGTH      0.0f  // Hip segment (base rotation to femur joint)
#define LEG_FEMUR_LENGTH     0.0f  // Thigh segment
#define LEG_TIBIA_LENGTH     0.0f  // Shin segment

// Body geometry - measure from CAD
#define BODY_LENGTH          0.0f  // Front to back centerline distance in mm
#define BODY_WIDTH           0.0f  // Side to side centerline distance in mm

// Leg mounting positions relative to body center (x, y in mm)
// Convention: +X = forward, +Y = left, origin at body geometric center
#define LEG_FL_ATTACH_X      0.0f  // Front left X
#define LEG_FL_ATTACH_Y      0.0f  // Front left Y
#define LEG_ML_ATTACH_X      0.0f  // Middle left X
#define LEG_ML_ATTACH_Y      0.0f  // Middle left Y
#define LEG_RL_ATTACH_X      0.0f  // Rear left X
#define LEG_RL_ATTACH_Y      0.0f  // Rear left Y
#define LEG_FR_ATTACH_X      0.0f  // Front right X (should be negative Y)
#define LEG_FR_ATTACH_Y      0.0f
#define LEG_MR_ATTACH_X      0.0f  // Middle right X
#define LEG_MR_ATTACH_Y      0.0f
#define LEG_RR_ATTACH_X      0.0f  // Rear right X
#define LEG_RR_ATTACH_Y      0.0f

// Gait parameters
#define DEFAULT_STANCE_HEIGHT    0.0f  // Body height above ground in mm
#define DEFAULT_STEP_HEIGHT      0.0f  // Leg lift height during swing in mm
#define DEFAULT_STEP_LENGTH      0.0f  // Stride length in mm
#define GAIT_CYCLE_TIME_MS       1000  // Time for one complete step cycle in ms

// Leg indices
typedef enum {
    LEG_FRONT_LEFT = 0,
    LEG_MIDDLE_LEFT = 1,
    LEG_REAR_LEFT = 2,
    LEG_FRONT_RIGHT = 3,
    LEG_MIDDLE_RIGHT = 4,
    LEG_REAR_RIGHT = 5,
    NUM_LEGS = 6
} leg_index_t;

// Gait patterns
typedef enum {
    GAIT_TRIPOD,        // Fast: 3 legs down at once (FL,MR,RL vs FR,ML,RR)
    GAIT_WAVE,          // Slow: Sequential leg lifts (most stable, 5 legs down)
    GAIT_RIPPLE,        // Medium: 4 legs down at once
    GAIT_STATIONARY     // No movement, all legs in stance
} gait_type_t;

// 3D position of leg tip
typedef struct {
    float x;  // Forward/back relative to leg attachment point
    float y;  // Left/right relative to leg attachment point
    float z;  // Up/down (negative = below body)
} leg_position_t;

// Leg joint angles
typedef struct {
    float coxa;   // Hip rotation angle (horizontal plane)
    float femur;  // Thigh pitch angle (vertical plane)
    float tibia;  // Shin pitch angle (vertical plane)
} leg_angles_t;

// State of single leg
typedef struct {
    leg_position_t position;    // Foot position in leg frame
    leg_angles_t angles;        // Joint angles
    bool in_swing;              // True if leg is in air (swing phase)
    float phase;                // Current phase in gait cycle [0.0, 1.0]
} leg_state_t;

// Complete gait state
typedef struct {
    leg_state_t legs[NUM_LEGS];
    float global_phase;         // Overall gait cycle phase [0.0, 1.0]
    gait_type_t type;
    float step_height;
    float stance_height;
} gait_state_t;

// Velocity command from user/controller
typedef struct {
    float forward_velocity;     // mm/s, positive = forward
    float lateral_velocity;     // mm/s, positive = left
    float rotation_velocity;    // deg/s, positive = counter-clockwise
} velocity_command_t;

/**
 * @brief Initialize gait generator with default stance
 * 
 * Initializes internal state and sets default gait pattern. Should be called once at startup.
 * 
 * @return ESP_OK on success
 */
esp_err_t xGaitGeneratorInit(void);

/**
 * @brief Set gait pattern
 * 
 * Set gait type to one of the predefined patterns. This will affect leg timing and coordination.
 * 
 * @param type Gait pattern to use
 * @return ESP_OK on success
 */
esp_err_t xGaitSetType(gait_type_t type);

/**
 * @brief Update gait state based on velocity command and time delta
 * 
 * Update gait state by calculating new leg positions and angles based on the desired velocity and elapsed time. This should be called in a regular control loop.
 * 
 * @param velocity Desired velocity vector
 * @param dt_ms Time elapsed since last update in milliseconds
 * @param state Output gait state with leg positions and angles
 * @return ESP_OK on success
 */
esp_err_t xGaitUpdate(const velocity_command_t *velocity, uint32_t dt_ms, gait_state_t *state);

/**
 * @brief Solve 3-DOF leg IK (coxa, femur, tibia)
 * 
 * Solves 3 DOF inverse kinematics for a single leg to reach a target foot position. Returns joint angles needed to achieve the target position. If the target is out of reach, returns an error.
 * 
 * @param leg_idx Which leg to solve for
 * @param target Desired foot position relative to leg attachment point
 * @param angles Output joint angles
 * @return ESP_OK if solvable, ESP_ERR_INVALID_ARG if unreachable
 */
esp_err_t xGaitLegIK(leg_index_t leg_idx, const leg_position_t *target, leg_angles_t *angles);

/**
 * @brief Get default neutral stance position for a leg
 * 
 * Returns the default neutral stance position for a given leg. This is the position where the leg is extended outward and downward from the body.
 * 
 * @param leg_idx Which leg
 * @param position Output neutral position
 * @return ESP_OK on success
 */
esp_err_t xGaitGetNeutralStance(leg_index_t leg_idx, leg_position_t *position);

#endif