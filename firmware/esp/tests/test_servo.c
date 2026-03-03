/**
 * @file test_servo.c
 * @brief Slow servo sweep test (0 to 180 degrees)
 *
 * Initializes PCA9685 on the body board (0x40) and sweeps all 16 channels
 * simultaneously from 0 to 180 degrees in 1-degree increments, then back to 0.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "i2c.h"

#define TAG "TEST_SERVO"

#define SWEEP_ADDR      PCA9685_BODY_ADDR    // 0x40
#define SWEEP_CHANNELS  16
#define SWEEP_STEP_DEG  1
#define SWEEP_DELAY_MS  20                  // delay per degree (~3.6s for full sweep)

void test_servo_sweep(void) {
    printf("Servo Sweep Test\n");
    printf("  Board addr : 0x%02X\n", SWEEP_ADDR);
    printf("  Channels   : 0-%d (all)\n", SWEEP_CHANNELS - 1);
    printf("  Step       : %d deg\n", SWEEP_STEP_DEG);
    printf("  Delay      : %d ms/step\n", SWEEP_DELAY_MS);
    printf("  Range      : 0 -> 180 -> 0\n\n");

    esp_err_t ret = xPCA9685Init(I2C_NUM_0, SWEEP_ADDR, SERVO_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return;
    }

    uint16_t pulses[SWEEP_CHANNELS];

    // sweep 0 -> 180
    // printf("Sweeping 0 -> 180...\n");
    // for (int angle = 0; angle <= 180; angle += SWEEP_STEP_DEG) {
    //     uint16_t pulse_us = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180);
    //     for (int ch = 0; ch < SWEEP_CHANNELS; ch++) pulses[ch] = pulse_us;
    //     ret = xPCA9685SetPwmBurst(I2C_NUM_0, SWEEP_ADDR, 0, SWEEP_CHANNELS, pulses);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "SetPwmBurst(%d deg) failed: %s", angle, esp_err_to_name(ret));
    //         return;
    //     }
    //     if (angle % 30 == 0) printf("  %3d deg\n", angle);
    //     vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    // }

    // printf("Holding at 180 for 1s...\n");
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // sweep 180 -> 0
    // printf("Sweeping 180 -> 0...\n");
    // for (int angle = 180; angle >= 0; angle -= SWEEP_STEP_DEG) {
    //     uint16_t pulse_us = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180);
    //     for (int ch = 0; ch < SWEEP_CHANNELS; ch++) pulses[ch] = pulse_us;
    //     ret = xPCA9685SetPwmBurst(I2C_NUM_0, SWEEP_ADDR, 0, SWEEP_CHANNELS, pulses);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "SetPwmBurst(%d deg) failed: %s", angle, esp_err_to_name(ret));
    //         return;
    //     }
    //     if (angle % 30 == 0) printf("  %3d deg\n", angle);
    //     vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    // }

    int angle = 90;
    uint16_t pulse_us = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180);
    for (int ch = 0; ch < SWEEP_CHANNELS; ch++) pulses[ch] = pulse_us;
    ret = xPCA9685SetPwmBurst(I2C_NUM_0, SWEEP_ADDR, 0, SWEEP_CHANNELS, pulses);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SetPwmBurst(%d deg) failed: %s", angle, esp_err_to_name(ret));
        return;
    }
    if (angle % 30 == 0) printf("  %3d deg\n", angle);
    vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));

    printf("Servo set complete.\n");
}
