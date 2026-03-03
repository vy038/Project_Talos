/**
 * @file test_power.c
 * @brief Power monitor test - reads current draw over time
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "power_monitor.h"

#define TAG "TEST_POWER"

#define POWER_READ_COUNT    10
#define POWER_READ_MS       500

void test_power(void) {
    printf("Power Monitor Test (%d readings, %dms interval)\n\n", POWER_READ_COUNT, POWER_READ_MS);

    esp_err_t ret = xACS712Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ACS712 init failed: %s", esp_err_to_name(ret));
        return;
    }

    printf("  #   Current (A)\n");
    printf("  --- -----------\n");

    for (int i = 0; i < POWER_READ_COUNT; i++) {
        float current;
        ret = xACS712ReadCurrent(&current);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read %d failed: %s", i, esp_err_to_name(ret));
            continue;
        }
        printf("  %3d %11.3f\n", i, current);
        vTaskDelay(pdMS_TO_TICKS(POWER_READ_MS));
    }
}
