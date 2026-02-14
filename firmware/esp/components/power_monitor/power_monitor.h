#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include <stdint.h>

// ACS712 ADC channel (GPIO 34 = ADC1_CHANNEL_6)
#define ACS712_CHANNEL          ADC_CHANNEL_6

/**
 * @brief Initialize ACS712 current sensor
 *
 * Sets up the ADC channel for the ACS712 with voltage divider.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xACS712Init(void);

/**
 * @brief Measure current through ACS712
 *
 * Reads averaged ADC samples, compensates for voltage divider,
 * and converts to current in Amps.
 *
 * @param current Pointer to store current reading (Amps)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xACS712ReadCurrent(float *current);

#endif