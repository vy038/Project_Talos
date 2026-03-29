// vision_task.c
// Waits for a ping byte from WROOM, runs one detection frame, sends result back.

#include "vision_task.h"
#include "vision_processor.h"
#include "uart_protocol.h"
#include "uart.h"
#include "esp_log.h"

static const char *TAG = "VISION_TASK";

// physical radius of the tracked ball in mm — tune to your ball
#define BALL_REAL_RADIUS_MM     20.0f

void vVisionTask(void *pvParams) {
    vVisionSetBallRadius(BALL_REAL_RADIUS_MM);

    uint8_t ping;
    size_t bytes_read;

    while (1) {
        // block here until WROOM sends a ping byte — zero CPU cost while waiting
        esp_err_t ret = xUARTRead(&ping, 1, &bytes_read);
        if (ret != ESP_OK || bytes_read == 0 || ping != VISION_PING_BYTE) {
            continue;
        }

        // run full detection pipeline: capture, HSV, threshold, blob, TOF fuse
        ball_detection_t det;
        ret = xVisionDetectBall(&det);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Detection failed, skipping frame");
            continue;
        }

        // pack result and send back to WROOM
        talos_detection_t td = {
            .detected = det.detected,
            .x        = (uint16_t)det.centroid_x,
            .y        = (uint16_t)det.centroid_y,
            .r        = det.detected ? (uint16_t)det.dist_mm : 0,
        };

        uint8_t pkt[TALOS_PKT_LEN];
        vUARTProtoBuildDetection(pkt, &td);
        xUARTWrite(pkt, TALOS_PKT_LEN);
    }
}
