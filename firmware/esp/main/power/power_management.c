#include "power_management.h"

esp_err_t xPowerManagementInit(void) {
    return ESP_OK;
}

float fReadCurrentSensor(void) {
    return 0.0f;
}

float fCalculatePowerDraw(float current, float voltage) {
    return 0.0f;
}

bool bCheckOvercurrent(float current) {
    return false;
}

float fEstimateBatteryLife(float current) {
    return 0.0f;
}

esp_err_t xTriggerPowerLimits(void) {
    return ESP_OK;
}