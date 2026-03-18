/**
 * @file adc_stub.c
 * @brief Virtual ADC replacing components/ADC/adc_helpers.c
 *
 * Returns configurable simulated ADC values.
 */

#include "adc_helpers.h"
#include "esp_log.h"

static const char *TAG = "ADC_SIM";

/* Simulated battery at ~11.1V through voltage divider = ~2.5V at ADC */
static float sim_voltages[10] = {
    [6] = 2.5f,    /* VBAT channel - battery through divider */
    [7] = 1.65f,   /* ACS712 channel - 1.65V = 0A (midpoint) */
};

esp_err_t xAdcInit(const adc_channel_t channels[], int num_channels) {
    ESP_LOGI(TAG, "Virtual ADC initialized with %d channels", num_channels);
    return ESP_OK;
}

int iAnalogReadRaw(adc_channel_t channel) {
    if (channel < 0 || channel >= 10) return 0;
    return (int)(sim_voltages[channel] / ADC_VREF * 4095.0f);
}

float fAnalogReadVoltage(adc_channel_t channel) {
    if (channel < 0 || channel >= 10) return 0.0f;
    return sim_voltages[channel];
}

float fAnalogReadVoltageAvg(adc_channel_t channel, int num_samples) {
    (void)num_samples;
    return fAnalogReadVoltage(channel);
}
