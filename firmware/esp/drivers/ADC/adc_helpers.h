#ifndef ADC_HELPERS_H
#define ADC_HELPERS_H

#include "esp_err.h"
#include"esp_adc/adc_oneshot.h"

adc_oneshot_unit_init_cfg_t config = {
    .unit_id = ;
    .clk_src = ;
    .ulp_mode = ;
}

/**
 * @brief Initialize ADC unit with multiple channels
 *
 * Configures ADC unit 1 with 12-bit resolution and 12dB attenuation
 * (0-3.3V range) for all specified channels.
 *
 * @param channels Array of ADC channel numbers to configure
 * @param num_channels Number of channels in the array
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t vAdcInit(const adc_channel_t channels[], int num_channels);

/**
 * @brief Read raw ADC value from a channel
 *
 * @param channel ADC channel to read from
 * @return int Raw ADC value (0-4095 for 12-bit resolution), 0 if not
 * initialized
 */
int iAnalogPinReadRaw(adc_channel_t channel);

/**
 * @brief Read voltage from an ADC channel
 *
 * Converts the raw ADC reading to voltage assuming 3.3V reference.
 *
 * @param channel ADC channel to read from
 * @return float Voltage reading in volts (0.0-3.3V)
 */
float fAnalogPinReadVoltage(adc_channel_t channel);

#endif
