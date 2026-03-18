/**
 * @file test_red_ball.c
 * @brief Red ball detection test — logs structured data for test_red_ball_serial.py
 *
 * Set BALL_REAL_RADIUS_MM to your ball's physical radius:
 *   tennis=32mm, ping-pong=20mm, golf=21mm
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vision_processor.h"

#define TAG "TEST_RED_BALL"

#define BALL_REAL_RADIUS_MM     22.0f
#define TEST_FRAME_COUNT        0       // 0 = run forever
#define FRAME_DELAY_MS          50      // ~20 FPS

// UART packet format (matches main.c / state_machine.h)
#define PKT_LEN     11

static void build_packet(const ball_detection_t *det, uint8_t *pkt) {
    uint16_t ball_r = det->detected ? (uint16_t)det->dist_mm : 0;
    if (ball_r == 0 && det->detected) ball_r = (uint16_t)det->pixel_radius;

    pkt[0] = 0xAA;
    pkt[1] = 0x55;
    pkt[2] = 0x01;
    pkt[3] = det->detected ? 1 : 0;
    pkt[4] = ((uint16_t)det->centroid_x >> 8) & 0xFF;
    pkt[5] =  (uint16_t)det->centroid_x & 0xFF;
    pkt[6] = ((uint16_t)det->centroid_y >> 8) & 0xFF;
    pkt[7] =  (uint16_t)det->centroid_y & 0xFF;
    pkt[8] = (ball_r >> 8) & 0xFF;
    pkt[9] =  ball_r & 0xFF;
    uint8_t chk = 0;
    for (int i = 2; i < 10; i++) chk ^= pkt[i];
    pkt[10] = chk;
}

void test_red_ball(void) {
    printf("\n========================================\n");
    printf("  Red Ball Detection Test\n");
    printf("  Ball radius : %.1f mm\n", BALL_REAL_RADIUS_MM);
    printf("  Frame delay : %d ms\n", FRAME_DELAY_MS);
    printf("========================================\n\n");

    vVisionSetBallRadius(BALL_REAL_RADIUS_MM);

    int frame_num = 0;
    ball_detection_t det;

    while (TEST_FRAME_COUNT == 0 || frame_num < TEST_FRAME_COUNT) {
        if (xVisionDetectBall(&det) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint8_t pkt[PKT_LEN];
        build_packet(&det, pkt);

        char pkt_hex[PKT_LEN * 2 + 1];
        for (int i = 0; i < PKT_LEN; i++) sprintf(&pkt_hex[i * 2], "%02X", pkt[i]);

        // structured line parsed by test_red_ball_serial.py
        printf("[RED_BALL] frame=%d det=%d cx=%d cy=%d blob=%d px_r=%.1f "
               "ox=%.3f oy=%.3f dist_mm=%.1f dist_px=%.1f dist_tof=%.1f "
               "brg_deg=%.2f tof=%d pkt=%s\n",
               frame_num,
               det.detected ? 1 : 0,
               det.centroid_x, det.centroid_y,
               det.blob_pixels,
               det.pixel_radius,
               det.offset_x, det.offset_y,
               det.dist_mm, det.dist_px_mm, det.dist_tof_mm,
               det.bearing_deg,
               bTofIsAvailable() ? 1 : 0,
               pkt_hex);

        frame_num++;
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }

    printf("\nTest complete. %d frames processed.\n", frame_num);
}
