/**
 * @file mpu6050_stub.c
 * @brief Virtual MPU6050 IMU replacing components/MPU6050/mpu6050.c
 *
 * Returns configurable simulated IMU data. Default: flat and still (0g x/y, 1g z).
 */

#include "mpu6050.h"
#include "esp_log.h"
#include "sim_state.h"

static const char *TAG = "MPU6050_SIM";

static float sim_accel_x = 0.0f;
static float sim_accel_y = 0.0f;
static float sim_accel_z = -9.81f;  /* gravity pointing down */
static float sim_gyro_x = 0.0f;
static float sim_gyro_y = 0.0f;
static float sim_gyro_z = 0.0f;

esp_err_t xMPU6050_init(void) {
    ESP_LOGI(TAG, "Virtual MPU6050 initialized (flat, no motion)");
    return ESP_OK;
}

esp_err_t xMPU6050_read(mpu6050_data_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;

    /* Read from sim state if available, otherwise use defaults */
    sim_get_imu_override(&sim_accel_x, &sim_accel_y, &sim_accel_z,
                         &sim_gyro_x, &sim_gyro_y, &sim_gyro_z);

    data->accel_x = sim_accel_x;
    data->accel_y = sim_accel_y;
    data->accel_z = sim_accel_z;
    data->gyro_x = sim_gyro_x;
    data->gyro_y = sim_gyro_y;
    data->gyro_z = sim_gyro_z;

    /* Compute roll/pitch from accelerometer (same as real driver) */
    data->roll  = atan2f(data->accel_y, -data->accel_z) * (180.0f / M_PI);
    data->pitch = atan2f(data->accel_x, -data->accel_z) * (180.0f / M_PI);

    return ESP_OK;
}

esp_err_t xMPU6050_calibrate(void) {
    ESP_LOGI(TAG, "Virtual MPU6050 calibrated (no-op in sim)");
    return ESP_OK;
}

float fMPU6050_get_tilt(void) {
    mpu6050_data_t data;
    if (xMPU6050_read(&data) != ESP_OK) return -1.0f;
    return sqrtf(data.roll * data.roll + data.pitch * data.pitch);
}
