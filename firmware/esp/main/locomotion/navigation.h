#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "esp_err.h"
#include <stdbool.h>

#define NUM_DISTANCE_SENSORS 4

/**
 * @brief Initialize navigation system
 *
 * Sets up obstacle detection parameters
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xNavigationInit(void);

/**
 * @brief Read distance sensors
 *
 * Gets readings from all VL53L0X sensors
 *
 * @param distances Output array of distances in meters [4]
 */
void vReadDistanceSensors(float distances[NUM_DISTANCE_SENSORS]);

/**
 * @brief Detect obstacles
 *
 * Checks if any sensor detects object within threshold
 *
 * @param distances Sensor readings in meters [4]
 * @return bool True if obstacle detected, false otherwise
 */
bool bDetectObstacles(float distances[NUM_DISTANCE_SENSORS]);

/**
 * @brief Calculate avoidance maneuver
 *
 * Determines velocity and direction adjustments for obstacle avoidance
 *
 * @param distances Sensor readings in meters [4]
 * @param velocity_adjust Output velocity scaling factor (0.0 to 1.0)
 * @param direction_adjust Output turning adjustment in rad/s
 */
void vCalculateAvoidance(float distances[NUM_DISTANCE_SENSORS], float *velocity_adjust, float *direction_adjust);

/**
 * @brief Update target velocity
 *
 * Outputs commanded walking velocity after avoidance processing
 *
 * @param forward Output forward velocity in m/s
 * @param turn Output turning rate in rad/s
 */
void vUpdateTargetVelocity(float *forward, float *turn);

#endif