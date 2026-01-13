#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "esp_err.h"

#define NUM_LEGS 6

/**
 * @brief Initialize gait generator
 *
 * Sets up gait parameters and initial phase
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xGaitGeneratorInit(void);

/**
 * @brief Initialize tripod gait parameters
 *
 * Configures step height and stride length for tripod walking
 *
 * @param step_height Maximum foot lift height in meters
 * @param stride_length Forward distance per step in meters
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xInitTripodGait(float step_height, float stride_length);

/**
 * @brief Update gait phase
 *
 * Advances gait cycle based on velocity and time delta
 *
 * @param velocity Forward velocity in m/s
 * @param direction Turning rate in rad/s
 * @param dt Time step in seconds
 */
void vUpdateGaitPhase(float velocity, float direction, float dt);

/**
 * @brief Get target foot position
 *
 * Calculates desired foot position for specified leg
 *
 * @param leg_index Leg number (0-5)
 * @param x Output X coordinate in meters
 * @param y Output Y coordinate in meters
 * @param z Output Z coordinate in meters
 */
void vGetFootTargets(int leg_index, float *x, float *y, float *z);

/**
 * @brief Calculate swing trajectory height
 *
 * Returns foot height during swing phase
 *
 * @param phase Gait phase (0.0 to 1.0)
 * @return float Height in meters
 */
float fSwingTrajectory(float phase);

/**
 * @brief Calculate stance trajectory position
 *
 * Returns foot position during ground contact phase
 *
 * @param phase Gait phase (0.0 to 1.0)
 * @return float Forward position in meters
 */
float fStanceTrajectory(float phase);

#endif