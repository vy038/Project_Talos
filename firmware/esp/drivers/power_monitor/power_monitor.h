#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize ACS712
 *
 * Sets up the ACS712 (Make sure ADC init() is called!)
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xACS712Init(void);

/**
 * @brief Measure current through ACS712
 *
 * Measures the current being read, converting the ADC read value after averaging
 * 
 * @param current float to store current
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xACS712ReadCurrent(float *current);

#endif
