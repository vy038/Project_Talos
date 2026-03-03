#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"

/* ========================================================================== */
/*  Select which test to run (comment out for normal operation)                */
/* ========================================================================== */
#define TEST_SELECT TEST_GAIT
#include "tests.h"

void app_main(void) {
    printf("Project Talos Starting...\n");

    // I2C bus init (needed by most tests)
    esp_err_t ret = xI2cMasterInit();
    if (ret != ESP_OK) {
        printf("I2C init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // always scan the bus first
    test_i2c_scan();
    printf("\n");

    run_test();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
