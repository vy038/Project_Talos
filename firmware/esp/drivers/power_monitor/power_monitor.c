#include "power_monitor.h"
#include "../adc/adc_helpers.h" 
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "POWER";

// Hardware configuration
#define ACS712_CHANNEL          ADC_CHANNEL_6   // GPIO 34
#define DIVIDER_RATIO           0.667f          // 10k / 10k
#define ACS712_ZERO_VOLTAGE     2.5f            // output at 0 Amps (assuming 5V)
#define ACS712_SENSITIVITY      0.066f          // 66mV per Amp (30A version)

#define NUM_SAMPLES             50              // more samples = more stable

static bool initialized = false;                // prevent double initialization

esp_err_t xACS712Init(void) {
    // configure on given channel
    adc_channel_t channels[] = {ACS712_CHANNEL};
    esp_err_t ret = xAdcInit(channels, 1);
    if (ret != ESP_OK) return ret;
    initialized = true;
    ESP_LOGI(TAG, "Power monitor initialized (with voltage divider)");
    return ESP_OK;
}

esp_err_t xACS712ReadCurrent(float *current) {
    // initialization check
    if (!initialized) {
        ESP_LOGE(TAG, "Power monitor not initialized");
        return ESP_ERR_NOT_ALLOWED;
    }

    /*
       range: 0.26V - 2.24V (after voltage divider) * 2 = 0.52V - 4.48V (actual ACS712 output)
    */

    // read avg ADC voltage directly
    float adc_voltage = fAnalogReadVoltageAvg(ACS712_CHANNEL, NUM_SAMPLES);
    
    // convert voltage divider to actual voltage
    float sensor_voltage = adc_voltage / DIVIDER_RATIO;
    
    // use ACS712 formula for current from voltage
    current = (sensor_voltage - ACS712_ZERO_VOLTAGE) / ACS712_SENSITIVITY;
    
    return ESP_OK;
}
