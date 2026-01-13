#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize power management
 *
 * Sets up current monitoring and safety thresholds
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xPowerManagementInit(void);

/**
 * @brief Read current sensor
 *
 * Gets current draw from ACS712 sensor
 *
 * @return float Current in amperes
 */
float fReadCurrentSensor(void);

/**
 * @brief Calculate power draw
 *
 * Computes instantaneous power consumption
 *
 * @param current Current in amperes
 * @param voltage Battery voltage in volts
 * @return float Power in watts
 */
float fCalculatePowerDraw(float current, float voltage);

/**
 * @brief Check overcurrent condition
 *
 * Detects if current exceeds safe operating limit
 *
 * @param current Current in amperes
 * @return bool True if overcurrent detected, false otherwise
 */
bool bCheckOvercurrent(float current);

/**
 * @brief Estimate battery life
 *
 * Calculates remaining runtime based on current draw
 *
 * @param current Current in amperes
 * @return float Estimated minutes remaining
 */
float fEstimateBatteryLife(float current);

/**
 * @brief Trigger power limits
 *
 * Reduces servo activity when power limit exceeded
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xTriggerPowerLimits(void);

#endif