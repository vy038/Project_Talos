#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vision_processor.h"
#include "uart.h"
#include "uart_protocol.h"
#include "vision_task.h"

/* ========================================================================== */
/*  Select which test to run, or TEST_NONE for production                      */
/* ========================================================================== */
#define TEST_SELECT TEST_RED_BALL
#include "tests.h"

static const char *TAG = "MAIN";

#define BALL_RADIUS_MM     20.0f

void app_main(void) {
    // init uart
    esp_err_t ret = xUARTInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %d", ret);
        return;
    }

    // init visionprocessor
    ret = xVisionProcessorInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Vision init failed: %d", ret);
        return;
    }

    // config radius
    vVisionSetBallRadius(BALL_RADIUS_MM);

    if (TEST_SELECT != TEST_NONE) {
        run_test();
    } else {
        xTaskCreate(vVisionTask, "vision", 8192, NULL, 5, NULL);
    }
}
