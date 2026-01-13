#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include "esp_err.h"
#include <stdbool.h>

#define ARM_DOF 6

/**
 * @brief Initialize IK solver
 *
 * Sets up initial parameters and workspace limits for 6-DOF arm
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xIkSolverInit(void);

/**
 * @brief Calculate forward kinematics
 *
 * Computes end-effector position from joint angles
 *
 * @param joint_angles Array of 6 joint angles in radians
 * @param end_effector_pos Output array [x, y, z] in meters
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xForwardKinematics(float joint_angles[ARM_DOF], float end_effector_pos[3]);

/**
 * @brief Solve inverse kinematics using Jacobian method
 *
 * Iteratively calculates joint angles to reach target position
 *
 * @param target_pos Target position [x, y, z] in meters
 * @param current_angles Current joint angles in radians
 * @param output_angles Output joint angles in radians
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xJacobianInverseIk(float target_pos[3], float current_angles[ARM_DOF], float output_angles[ARM_DOF]);

/**
 * @brief Validate joint angles within servo limits
 *
 * Checks if all joint angles are within mechanical range
 *
 * @param joint_angles Array of 6 joint angles in radians
 * @return bool True if valid, false if any joint exceeds limits
 */
bool bValidateJointLimits(float joint_angles[ARM_DOF]);

/**
 * @brief Interpolate trajectory between two poses
 *
 * Linear interpolation for smooth motion between configurations
 *
 * @param start Starting joint configuration
 * @param end Target joint configuration
 * @param t Interpolation parameter (0.0 to 1.0)
 * @param output Interpolated joint configuration
 */
void vInterpolateTrajectory(float start[ARM_DOF], float end[ARM_DOF], float t, float output[ARM_DOF]);

#endif