#ifndef ADC_HELPERS_H
#define ADC_HELPERS_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

// ADC configuration
#define ADC_DEFAULT_ATTEN       ADC_ATTEN_DB_11     // 0-3.3V range
#define ADC_DEFAULT_BITWIDTH    ADC_BITWIDTH_12      // 12-bit resolution (0-4095)
#define ADC_VREF                3.3f                 // reference voltage

/**
 * @brief Initialize ADC unit with multiple channels
 *
 * Configures ADC unit 1 with 12-bit resolution and 11dB attenuation
 * (0-3.3V range) for specified channels.
 *
 * @param channels ADC channels to configure
 * @param num_channels # of channels to configure
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xAdcInit(const adc_channel_t channels[], int num_channels);

/**
 * @brief Read raw ADC value from a channel
 *
 * @param channel ADC channel to read from
 * @return int Raw ADC value (0-4095 for 12-bit resolution), 0 on failure
 */
int iAnalogReadRaw(adc_channel_t channel);

/**
 * @brief Read voltage from an ADC channel
 *
 * Converts the raw ADC reading to voltage assuming 3.3V reference.
 *
 * @param channel ADC channel to read from
 * @return float Voltage reading in volts (0.0-3.3V)
 */
float fAnalogReadVoltage(adc_channel_t channel);

/**
 * @brief Read averaged voltage from an ADC channel
 *
 * Takes multiple raw samples and returns the averaged voltage.
 *
 * @param channel ADC channel to read from
 * @param num_samples number of samples to average
 * @return float Averaged voltage reading in volts (0.0-3.3V)
 */
float fAnalogReadVoltageAvg(adc_channel_t channel, int num_samples);

#endif