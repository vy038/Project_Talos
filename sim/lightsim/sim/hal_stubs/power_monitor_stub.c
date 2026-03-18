/**
 * @file power_monitor_stub.c
 * @brief Virtual ACS712 current sensor replacing components/power_monitor/power_monitor.c
 *
 * Returns configurable simulated current values.
 */

#include "power_monitor.h"
#include "esp_log.h"

static const char *TAG = "POWER_SIM";
static float sim_current = 0.8f; /* default idle current */

esp_err_t xACS712Init(void) {
    ESP_LOGI(TAG, "Virtual ACS712 initialized (sim current: %.2fA)", sim_current);
    return ESP_OK;
}

esp_err_t xACS712ReadCurrent(float *current) {
    if (!current) return ESP_ERR_INVALID_ARG;
    *current = sim_current;
    return ESP_OK;
}

/* Called by bridge server to set simulated current */
void sim_set_current(float amps) {
    sim_current = amps;
}
