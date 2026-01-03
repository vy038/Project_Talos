#include"adc_helpers.h"
#include"esp_log.h"

// Configuration (0-3.3V range)
#define ADC_EXAMPLE_ATTEN   ADC_ATTEN_DB_11

static const char *TAG = "ADC";

static adc_oneshit_unit_handle_t adc_handle = NULL;

esp_err_t xAdcInit(const adc_channel_t channels[], int num_channels) {
    // config to ADC unit 1
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    // unit config
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC unit:%s", esp_err_to_name(ret));
        return ret;
    }

    // channel config
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };

    // initialize all channels
    for (int i = 0; i < num_channels; i++) {
        ret = adc_oneshot_config_channel(adc_handle, channels[i], &chan_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config channel:%s", channels[i]);
            return ret;
        }
    }

    ESP_LOGI(TAG, "ADC initialized with %d channels", num_channels);
    return ESP_OK;
}


int iAnalogReadRaw(adc_channel_t channel) {
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, channel, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed");
        return 0;
    }
    return raw;
}


float fAnalogReadVoltage(adc_channel_t channel) {
    int raw = iAnalogReadRaw(channel);
    // convert to 3.3V scale
    return (raw / 4095.0f) * 3.3f;
}

float fAnalogReadVoltageAvg(adc_channel_t channel, int num_samples) {
    int32_t sum = 0;
    for (int i = 0; i < num_samples; i++) {
        sum += iAnalogReadRaw(channel);
    }
    return (((float)sum / num_samples) / 4095.0f) * 3.3f;
}