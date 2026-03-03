/**
 * @file test_mpu.c
 * @brief MPU6050 IMU read test - init, calibrate, print accel/gyro data
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"

#define TAG "TEST_MPU"

#define MPU_READ_COUNT  10
#define MPU_READ_MS     100

void test_mpu6050(void) {
    printf("MPU6050 IMU Test (%d readings, %dms interval)\n\n", MPU_READ_COUNT, MPU_READ_MS);

    esp_err_t ret = xMPU6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(ret));
        return;
    }

    printf("Calibrating (keep sensor still)...\n");
    ret = xMPU6050_calibrate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(ret));
        return;
    }
    printf("Calibration done.\n\n");

    printf("  #   Ax(m/s2)  Ay(m/s2)  Az(m/s2)  Gx(d/s)  Gy(d/s)  Gz(d/s)\n");
    printf("  --- --------- --------- --------- -------- -------- --------\n");

    mpu6050_data_t data;
    for (int i = 0; i < MPU_READ_COUNT; i++) {
        ret = xMPU6050_read(&data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read %d failed: %s", i, esp_err_to_name(ret));
            continue;
        }
        printf("  %3d %9.2f %9.2f %9.2f %8.2f %8.2f %8.2f\n",
               i, data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x, data.gyro_y, data.gyro_z);
        vTaskDelay(pdMS_TO_TICKS(MPU_READ_MS));
    }
}
