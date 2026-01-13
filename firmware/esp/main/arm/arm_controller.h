#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "esp_err.h"

/**
 * @brief Initialize arm controller
 *
 * Sets up arm controller parameters and initial state
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmControllerInit(void);

/**
 * @brief Move arm to target position
 *
 * Plans trajectory and executes motion to target coordinates
 *
 * @param target_x Target X coordinate in meters
 * @param target_y Target Y coordinate in meters
 * @param target_z Target Z coordinate in meters
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmMoveTo(float target_x, float target_y, float target_z);

/**
 * @brief Execute smooth trajectory
 *
 * Moves arm along interpolated path over specified duration
 *
 * @param start_pos Starting position [x, y, z]
 * @param end_pos Target position [x, y, z]
 * @param duration_ms Movement duration in milliseconds
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmExecuteTrajectory(float start_pos[3], float end_pos[3], uint32_t duration_ms);

/**
 * @brief Set gripper state
 *
 * Opens or closes gripper servo
 *
 * @param closed True to close gripper, false to open
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmSetGripper(bool closed);

/**
 * @brief Get current arm position
 *
 * Returns current end-effector position
 *
 * @param current_pos Output array [x, y, z] in meters
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xArmGetPosition(float current_pos[3]);

#endif