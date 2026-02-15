#include "adc_helpers.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "ADC";

static adc_oneshot_unit_handle_t adc_handle = NULL;
static bool adc_initialized = false;

esp_err_t xAdcInit(const adc_channel_t channels[], int num_channels) {
    if (adc_initialized) {
        ESP_LOGW(TAG, "ADC already initialized, configuring new channels only");
    } else {
        // config to ADC unit 1
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };

        // unit config
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
        adc_initialized = true;
    }

    // channel config
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_DEFAULT_ATTEN,
        .bitwidth = ADC_DEFAULT_BITWIDTH,
    };

    // initialize all channels
    for (int i = 0; i < num_channels; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channels[i], &chan_config));
    }

    ESP_LOGI(TAG, "ADC initialized with %d channels", num_channels);
    return ESP_OK;
}


int iAnalogReadRaw(adc_channel_t channel) {
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, channel, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return raw;
}


float fAnalogReadVoltage(adc_channel_t channel) {
    int raw = iAnalogReadRaw(channel);
    // convert to 3.3V scale (12-bit: 0-4095 maps to 0-3.3V)
    return (raw / 4095.0f) * ADC_VREF;
}

float fAnalogReadVoltageAvg(adc_channel_t channel, int num_samples) {
    if (num_samples <= 0) return 0.0f;

    int32_t sum = 0;
    for (int i = 0; i < num_samples; i++) {
        sum += iAnalogReadRaw(channel);
    }
    return (((float)sum / num_samples) / 4095.0f) * ADC_VREF;
}