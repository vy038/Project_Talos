#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include "esp_err.h"
#include <stdbool.h>

#define LEG_DOF 3

/**
 * @brief Initialize leg controller
 *
 * Sets up leg kinematics parameters
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xLegControllerInit(void);

/**
 * @brief Solve 3-DOF leg inverse kinematics
 *
 * Calculates joint angles for target foot position
 *
 * @param leg_index Leg number (0-5)
 * @param x Target X coordinate in meters
 * @param y Target Y coordinate in meters
 * @param z Target Z coordinate in meters
 * @param hip Output hip angle in radians
 * @param knee Output knee angle in radians
 * @param ankle Output ankle angle in radians
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xLegIk3Dof(int leg_index, float x, float y, float z, float *hip, float *knee, float *ankle);

/**
 * @brief Validate if target is reachable
 *
 * Checks if foot position is within leg workspace
 *
 * @param leg_index Leg number (0-5)
 * @param x Target X coordinate in meters
 * @param y Target Y coordinate in meters
 * @param z Target Z coordinate in meters
 * @return bool True if reachable, false otherwise
 */
bool bValidateReachable(int leg_index, float x, float y, float z);

/**
 * @brief Transform hip frame to world frame
 *
 * Converts local leg coordinates to body-relative coordinates
 *
 * @param leg_index Leg number (0-5)
 * @param local Local coordinates [x, y, z]
 * @param world Output world coordinates [x, y, z]
 */
void vHipToWorldTransform(int leg_index, float local[3], float world[3]);

/**
 * @brief Apply servo calibration offsets
 *
 * Corrects joint angles for servo mounting errors
 *
 * @param leg_index Leg number (0-5)
 * @param angles Raw joint angles [hip, knee, ankle]
 * @param corrected Output corrected angles
 */
void vApplyServoOffsets(int leg_index, float angles[LEG_DOF], float corrected[LEG_DOF]);

#endif