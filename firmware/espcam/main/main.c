#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "vision_processor.h"
#include "uart.h"

static const char *TAG = "MAIN";

// must match ESP32-WROOM state_machine.h
#define CAM_WIDTH           320
#define CAM_HEIGHT          240
#define UART_MSG_START_0    0xAA
#define UART_MSG_START_1    0x55
#define UART_MSG_TYPE_DET   0x01
#define UART_MSG_LEN        11

// minimum blob size in pixels to count as a detection
#define MIN_BLOB_PIXELS     50

// vision task: capture -> process -> detect -> send over UART
static void vision_task(void *arg) {
    size_t frame_buf_size = CAM_WIDTH * CAM_HEIGHT * 2;     // RGB565
    size_t hsv_buf_size   = CAM_WIDTH * CAM_HEIGHT * 3;     // HSV
    size_t mask_buf_size  = CAM_WIDTH * CAM_HEIGHT;          // binary mask

    uint8_t *frame_buf = heap_caps_malloc(frame_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *hsv_buf   = heap_caps_malloc(hsv_buf_size,   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *mask_buf  = heap_caps_malloc(mask_buf_size,  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!frame_buf || !hsv_buf || !mask_buf) {
        ESP_LOGE(TAG, "Failed to allocate vision buffers - check PSRAM");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Vision task started");

    while (1) {
        // capture frame
        size_t frame_size = frame_buf_size;
        if (xCaptureFrame(frame_buf, &frame_size) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // RGB565 -> HSV -> red threshold -> blob detection
        vRgbToHsv(frame_buf, hsv_buf, CAM_WIDTH, CAM_HEIGHT);
        vThresholdRedRange(hsv_buf, mask_buf, CAM_WIDTH, CAM_HEIGHT);

        int cx = 0, cy = 0;
        int blob_pixels = iFindLargestBlob(mask_buf, CAM_WIDTH, CAM_HEIGHT, &cx, &cy);

        bool detected = (blob_pixels >= MIN_BLOB_PIXELS);
        uint16_t ball_x = detected ? (uint16_t)cx : 0;
        uint16_t ball_y = detected ? (uint16_t)cy : 0;
        uint16_t ball_r = detected ? (uint16_t)sqrtf((float)blob_pixels / M_PI) : 0;

        // use TOF distance to override radius when available
        // TOF gives real mm distance, more reliable than pixel radius for grab decisions
        if (detected && bTofIsAvailable()) {
            uint16_t dist_mm = uiTofReadDistanceMm();
            if (dist_mm > 0 && dist_mm < 2000) {
                ball_r = dist_mm;
            }
        }

        // build UART packet (11 bytes, matches ESP32 bStateMachineParseUART)
        // [0xAA] [0x55] [0x01] [det] [x_hi] [x_lo] [y_hi] [y_lo] [r_hi] [r_lo] [checksum]
        uint8_t pkt[UART_MSG_LEN];
        pkt[0] = UART_MSG_START_0;
        pkt[1] = UART_MSG_START_1;
        pkt[2] = UART_MSG_TYPE_DET;
        pkt[3] = detected ? 1 : 0;
        pkt[4] = (ball_x >> 8) & 0xFF;
        pkt[5] = ball_x & 0xFF;
        pkt[6] = (ball_y >> 8) & 0xFF;
        pkt[7] = ball_y & 0xFF;
        pkt[8] = (ball_r >> 8) & 0xFF;
        pkt[9] = ball_r & 0xFF;

        uint8_t checksum = 0;
        for (int i = 2; i < 10; i++) {
            checksum ^= pkt[i];
        }
        pkt[10] = checksum;

        xUARTWrite(pkt, UART_MSG_LEN);

        // ~20 FPS
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Project Talos - ESP-CAM starting");

    // init UART to ESP32-WROOM
    esp_err_t ret = xUARTInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %d", ret);
        return;
    }

    // init camera + VL53L0X TOF
    ret = xVisionProcessorInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Vision processor init failed: %d", ret);
        return;
    }

    // start vision processing task
    xTaskCreate(vision_task, "vision", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "All systems initialized");
}
