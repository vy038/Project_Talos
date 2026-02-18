#include "power_management.h"
#include "power_monitor.h"
#include "esp_log.h"

static const char *TAG = "POWER_MGMT";

static power_info_t info = {0};
static power_event_cb_t event_callback = NULL;
static uint8_t consecutive_warning = 0;
static uint8_t consecutive_emergency = 0;
static bool initialized = false;

esp_err_t xPowerInit(void) {
    // configs for power stats TODO: change values
    info.current_amps = 0;
    info.peak_amps = 0;
    info.status = POWER_OK;
    info.warning_count = 0;
    info.emergency_count = 0;
    consecutive_warning = 0;
    consecutive_emergency = 0;

    initialized = true;
    ESP_LOGI(TAG, "Power management initialized");
    ESP_LOGI(TAG, "  Warning threshold:  %.1fA (after %d consecutive)",
             POWER_WARNING_AMPS, POWER_WARNING_COUNT);
    ESP_LOGI(TAG, "  Emergency threshold: %.1fA (after %d consecutive)",
             POWER_EMERGENCY_AMPS, POWER_EMERGENCY_COUNT);
    return ESP_OK;
}

void vPowerSetCallback(power_event_cb_t callback) {
    event_callback = callback;
}

power_status_t xPowerCheck(void) {
    if (!initialized) return POWER_WARNING;

    // read current from sensor
    float amps = 0;
    esp_err_t ret = xACS712ReadCurrent(&amps);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read current sensor");
        return info.status;
    }

    if (amps < 0) amps = 0;

    info.current_amps = amps;
    if (amps > info.peak_amps) info.peak_amps = amps;

    // if in emergency, wait for consecutive readings before declaring, and trigger callback if status changes
    if (amps >= POWER_EMERGENCY_AMPS) {
        consecutive_emergency++;
        if (consecutive_emergency >= POWER_EMERGENCY_COUNT) {
            if (info.status != POWER_EMERGENCY) {
                ESP_LOGE(TAG, "EMERGENCY: %.2fA exceeds %.1fA limit!",
                         amps, POWER_EMERGENCY_AMPS);
                info.status = POWER_EMERGENCY;
                info.emergency_count++;
                if (event_callback) {
                    event_callback(POWER_EMERGENCY, amps);
                }
            }
        }
    } else {
        consecutive_emergency = 0;
    }

    // if in dangerous territory, reduce amps (hysteresis) before clearing warning
    if (amps >= POWER_WARNING_AMPS && info.status != POWER_EMERGENCY) {
        consecutive_warning++;
        if (consecutive_warning >= POWER_WARNING_COUNT) {
            if (info.status != POWER_WARNING) {
                ESP_LOGW(TAG, "WARNING: Current %.2fA above %.1fA",
                         amps, POWER_WARNING_AMPS);
                info.status = POWER_WARNING;
                info.warning_count++;
                if (event_callback) {
                    event_callback(POWER_WARNING, amps);
                }
            }
        }
    } else if (amps < POWER_WARNING_AMPS && info.status == POWER_WARNING) {
        consecutive_warning = 0;
        info.status = POWER_OK;
        ESP_LOGI(TAG, "Current normalized: %.2fA", amps);
    }

    return info.status;
}

power_info_t xPowerGetInfo(void) {
    return info;
}

void vPowerResetStats(void) {
    // change stats back to default, but keep current reading and status for continuity
    info.peak_amps = info.current_amps;
    info.warning_count = 0;
    info.emergency_count = 0;
    ESP_LOGI(TAG, "Stats reset");
}

float fPowerGetCurrent(void) {
    return info.current_amps;
}