#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize balance controller
 *
 * Sets up PID controllers and IMU integration
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xBalanceControlInit(void);

/**
 * @brief Initialize PID controllers
 *
 * Configures PID gains for pitch and roll control
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xInitPidControllers(float kp, float ki, float kd);

/**
 * @brief Update IMU reading
 *
 * Processes new IMU data for balance calculation
 *
 * @param pitch Current pitch angle in radians
 * @param roll Current roll angle in radians
 */
void vUpdateImuReading(float pitch, float roll);

/**
 * @brief Compute PID correction
 *
 * Calculates stabilization adjustments based on IMU error
 *
 * @param pitch_correction Output pitch correction in radians
 * @param roll_correction Output roll correction in radians
 */
void vComputePidCorrection(float *pitch_correction, float *roll_correction);

/**
 * @brief Apply body tilt compensation
 *
 * Adjusts foot positions to maintain level body orientation
 *
 * @param pitch Body pitch angle in radians
 * @param roll Body roll angle in radians
 * @param foot_offsets Output array of foot offset adjustments [6][3]
 */
void vApplyBodyTilt(float pitch, float roll, float foot_offsets[6][3]);

/**
 * @brief Detect fall condition
 *
 * Checks if robot tilt exceeds safe operating limits
 *
 * @return bool True if fall detected, false otherwise
 */
bool bDetectFallCondition(void);

#endif