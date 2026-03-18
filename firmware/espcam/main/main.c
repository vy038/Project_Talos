#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vision_processor.h"
#include "uart.h"
#include "uart_protocol.h"

/* ========================================================================== */
/*  Select which test to run, or TEST_NONE for production                      */
/* ========================================================================== */
#define TEST_SELECT TEST_RED_BALL
#include "tests.h"

static const char *TAG = "MAIN";

// physical radius of tracked ball in mm
#define BALL_REAL_RADIUS_MM     20.0f

static void vision_task(void *arg) {
    ball_detection_t det;
    int frame = 0;

    while (1) {
        if (xVisionDetectBall(&det) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint16_t ball_r = det.detected ? (uint16_t)det.dist_mm : 0;
        if (ball_r == 0 && det.detected) ball_r = (uint16_t)det.pixel_radius;

        talos_detection_t td = {
            .detected = det.detected,
            .x = (uint16_t)det.centroid_x,
            .y = (uint16_t)det.centroid_y,
            .r = ball_r,
        };
        uint8_t pkt[TALOS_PKT_LEN];
        vUARTProtoBuildDetection(pkt, &td);
        xUARTWrite(pkt, TALOS_PKT_LEN);
        frame++;

        vTaskDelay(pdMS_TO_TICKS(50));  // ~20 FPS
    }
}

void app_main(void) {
    esp_err_t ret = xUARTInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %d", ret);
        return;
    }

    ret = xVisionProcessorInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Vision init failed: %d", ret);
        return;
    }

    vVisionSetBallRadius(BALL_REAL_RADIUS_MM);

#if TEST_SELECT != TEST_NONE
    run_test();
#else
    xTaskCreate(vision_task, "vision", 8192, NULL, 5, NULL);
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
