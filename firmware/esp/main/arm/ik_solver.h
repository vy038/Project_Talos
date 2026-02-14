// ik_solver.h
#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Link lengths in mm - measure from your CAD model
#define ARM_BASE_ANGLE       0.0f  // Base mounting angle in degrees (0 = horizontal, 90 = vertical)
#define ARM_LINK1_LENGTH     0.0f  // Shoulder to elbow
#define ARM_LINK2_LENGTH     0.0f  // Elbow to wrist joint 1
#define ARM_LINK3_LENGTH     0.0f  // Wrist joint 1 to wrist joint 2
#define ARM_LINK4_LENGTH     0.0f  // Wrist joint 2 to gripper tip

// Joint limits in degrees - get from servo datasheets and mechanical stops
#define BASE_ROTATION_MIN       0.0f
#define BASE_ROTATION_MAX       180.0f
#define SHOULDER_MIN            0.0f
#define SHOULDER_MAX            180.0f
#define ELBOW_MIN               0.0f
#define ELBOW_MAX               180.0f
#define WRIST_PITCH_MIN         0.0f
#define WRIST_PITCH_MAX         180.0f
#define WRIST_ROLL_MIN          0.0f
#define WRIST_ROLL_MAX          180.0f
#define WRIST_YAW_MIN           0.0f
#define WRIST_YAW_MAX           180.0f

// Workspace limits
#define MAX_REACH               (ARM_LINK1_LENGTH + ARM_LINK2_LENGTH + ARM_LINK3_LENGTH + ARM_LINK4_LENGTH)
#define MIN_REACH               20.0f  // Minimum safe distance from base

// ik_solver.h additions
#define ARM_BASE_HEIGHT      0.0f  // Vertical distance from origin to shoulder pivot (mm)
#define ARM_GRIPPER_ANGLE    0.0f  // Fixed downward angle of gripper relative to wrist (degrees, positive = down)

typedef struct {
    float x;        // Target position in mm (robot frame: x=forward)
    float y;        // Target position in mm (robot frame: y=left)
    float z;        // Target position in mm (robot frame: z=up)
    float roll;     // End effector roll in degrees
    float pitch;    // End effector pitch in degrees
    float yaw;      // End effector yaw in degrees
} ik_target_t;

typedef struct {
    float base_rotation;    // Base turret rotation
    float shoulder;         // Shoulder pitch
    float elbow;            // Elbow pitch
    float wrist_pitch;      // Wrist pitch
    float wrist_roll;       // Wrist roll
    float wrist_yaw;        // Wrist yaw
    bool valid;             // Whether solution is valid
} ik_solution_t;


/**
 * @brief Initialize IK solver
 *
 * Sets up initial parameters and workspace limits for 6-DOF arm. Should be called before any IK solving attempts.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xIkSolverInit(void);

/**
 * @brief Solve inverse kinematics for target position
 * 
 * Solves for joint angles to achieve desired end effector pose. Returns ESP_ERR_INVALID_ARG if target is unreachable.
 * 
 * @param target Desired end effector pose
 * @param solution Output joint angles
 * @return ESP_OK if solution found, ESP_ERR_INVALID_ARG if unreachable
 */
esp_err_t xIKSolve(const ik_target_t *target, ik_solution_t *solution);

/**
 * @brief Validate if target is within workspace
 * 
 * Checks if the given target position is reachable based on arm dimensions and joint limits. Used for pre-validation before attempting to solve IK.
 * 
 * @param target Position to check
 * @return true if reachable
 */
bool bIKIsReachable(const ik_target_t *target);

/**
 * @brief Forward kinematics, position from angles
 * 
 * Find the position of the end effector given a set of joint angles. Useful for testing and visualization.
 * 
 * @param solution Joint angles
 * @param position Output end effector position
 * @return ESP_OK on success
 */
esp_err_t xIKForward(const ik_solution_t *solution, ik_target_t *position);

#endif