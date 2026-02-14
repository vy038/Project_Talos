/**
 * MPU6050 IMU Driver
 * I2C Address: 0x68
 * Provides: Accelerometer, Gyroscope, Temperature
 */

#include "mpu6050.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

// config values for gyro and accel
#define ACCEL_SENSITIVITY_2G        16384.0f
#define GYRO_SENSITIVITY_250        131.0f

static const char *TAG = "MPU6050";

// calibration data (lives here, NOT in the header)
// if this is static in the header, every .c file gets its own copy
// and calibration values written here would be invisible to other files
static struct {
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    bool calibrated;
} calibration = {0};

esp_err_t xMPU6050_init(void) {
    ESP_LOGI(TAG, "Initializing MPU6050");

    // verify if device is connected
    uint8_t who_am_i;
    esp_err_t ret = xI2cReadByte(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with MPU6050");
        return ret;
    }

    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02X (expected 0x68)", who_am_i);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // awake from sleep and give time to wake up
    ret = xI2cWriteByte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // config gyro: +/- 250 deg/s (sensitivity = 131 LSB/deg/s)
    ret = xI2cWriteByte(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // config accel: +/- 2g (sensitivity = 16384 LSB/g)
    ret = xI2cWriteByte(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 initialized");
    return ESP_OK;
}


esp_err_t xMPU6050_read(mpu6050_data_t *data) {
    uint8_t raw_data[14];

    // reads all 14 adjacent registers and puts them into raw data array
    esp_err_t ret = xI2cReadBytes(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) return ret;

    // accel data (cast BEFORE the OR so sign extension works correctly)
    data->accel_x = ((int16_t)((raw_data[0] << 8) | raw_data[1]) / ACCEL_SENSITIVITY_2G) * 9.81f;
    data->accel_y = ((int16_t)((raw_data[2] << 8) | raw_data[3]) / ACCEL_SENSITIVITY_2G) * 9.81f;
    data->accel_z = ((int16_t)((raw_data[4] << 8) | raw_data[5]) / ACCEL_SENSITIVITY_2G) * 9.81f;

    // temp data (not needed)
    // data->temperature = ((int16_t)((raw_data[6] << 8) | raw_data[7]) / 340.0f) + 36.53f;

    // gyro data (same casting pattern as accel: cast the combined 16-bit value)
    data->gyro_x = (int16_t)((raw_data[8]  << 8) | raw_data[9])  / GYRO_SENSITIVITY_250;
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]) / GYRO_SENSITIVITY_250;
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]) / GYRO_SENSITIVITY_250;

    // Apply calibration
    if (calibration.calibrated) {
        data->accel_x -= calibration.accel_offset_x;
        data->accel_y -= calibration.accel_offset_y;
        data->accel_z -= calibration.accel_offset_z;
        data->gyro_x -= calibration.gyro_offset_x;
        data->gyro_y -= calibration.gyro_offset_y;
        data->gyro_z -= calibration.gyro_offset_z;
    }

    // compute roll and pitch from accelerometer
    // pitch = rotation around Y axis (nose up/down)
    // roll  = rotation around X axis (lean left/right)
    data->pitch = atan2f(data->accel_x, data->accel_z) * (180.0f / M_PI);
    data->roll  = atan2f(data->accel_y, data->accel_z) * (180.0f / M_PI);

    return ESP_OK;
}


esp_err_t xMPU6050_calibrate(void) {
    // assuming flat ground
    ESP_LOGI(TAG, "Calibrating - keep robot LEVEL and STILL!");

    calibration.calibrated = false;

    // initialize
    const int NUM_SAMPLES = 100;
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    // gather samples (100 default)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        mpu6050_data_t sample;
        esp_err_t ret = xMPU6050_read(&sample);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration read failed at sample %d", i);
            return ret;
        }

        accel_sum_x += sample.accel_x;
        accel_sum_y += sample.accel_y;
        accel_sum_z += sample.accel_z;
        gyro_sum_x += sample.gyro_x;
        gyro_sum_y += sample.gyro_y;
        gyro_sum_z += sample.gyro_z;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    calibration.accel_offset_x = accel_sum_x / NUM_SAMPLES;
    calibration.accel_offset_y = accel_sum_y / NUM_SAMPLES;
    // Z should read 9.81 when level, so offset = actual - expected
    calibration.accel_offset_z = (accel_sum_z / NUM_SAMPLES) - 9.81f;

    calibration.gyro_offset_x = gyro_sum_x / NUM_SAMPLES;
    calibration.gyro_offset_y = gyro_sum_y / NUM_SAMPLES;
    calibration.gyro_offset_z = gyro_sum_z / NUM_SAMPLES;

    calibration.calibrated = true;

    ESP_LOGI(TAG, "Calibration complete!");
    ESP_LOGI(TAG, "  Accel offsets: x=%.3f y=%.3f z=%.3f",
             calibration.accel_offset_x, calibration.accel_offset_y, calibration.accel_offset_z);
    ESP_LOGI(TAG, "  Gyro offsets:  x=%.3f y=%.3f z=%.3f",
             calibration.gyro_offset_x, calibration.gyro_offset_y, calibration.gyro_offset_z);
    return ESP_OK;
}


float fMPU6050_get_tilt(void) {
    mpu6050_data_t data;
    if (xMPU6050_read(&data) != ESP_OK) return -1.0f;
    // combined tilt magnitude from roll and pitch (now actually computed in xMPU6050_read)
    return sqrtf(data.roll * data.roll + data.pitch * data.pitch);
}/**
 * MPU6050 IMU Driver
 * I2C Address: 0x68
 * Provides: Accelerometer, Gyroscope, Temperature
 */

#include "mpu6050.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

// config values for gyro and accel
#define ACCEL_SENSITIVITY_2G        16384.0f
#define GYRO_SENSITIVITY_250        131.0f

static const char *TAG = "MPU6050";

// calibration data (lives here, NOT in the header)
// if this is static in the header, every .c file gets its own copy
// and calibration values written here would be invisible to other files
static struct {
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    bool calibrated;
} calibration = {0};

esp_err_t xMPU6050_init(void) {
    ESP_LOGI(TAG, "Initializing MPU6050");

    // verify if device is connected
    uint8_t who_am_i;
    esp_err_t ret = xI2cReadByte(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with MPU6050");
        return ret;
    }

    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02X (expected 0x68)", who_am_i);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // awake from sleep and give time to wake up
    ret = xI2cWriteByte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // config gyro: +/- 250 deg/s (sensitivity = 131 LSB/deg/s)
    ret = xI2cWriteByte(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // config accel: +/- 2g (sensitivity = 16384 LSB/g)
    ret = xI2cWriteByte(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 initialized");
    return ESP_OK;
}


esp_err_t xMPU6050_read(mpu6050_data_t *data) {
    uint8_t raw_data[14];

    // reads all 14 adjacent registers and puts them into raw data array
    esp_err_t ret = xI2cReadBytes(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) return ret;

    // accel data (cast BEFORE the OR so sign extension works correctly)
    data->accel_x = ((int16_t)((raw_data[0] << 8) | raw_data[1]) / ACCEL_SENSITIVITY_2G) * 9.81f;
    data->accel_y = ((int16_t)((raw_data[2] << 8) | raw_data[3]) / ACCEL_SENSITIVITY_2G) * 9.81f;
    data->accel_z = ((int16_t)((raw_data[4] << 8) | raw_data[5]) / ACCEL_SENSITIVITY_2G) * 9.81f;

    // temp data (not needed)
    // data->temperature = ((int16_t)((raw_data[6] << 8) | raw_data[7]) / 340.0f) + 36.53f;

    // gyro data (same casting pattern as accel: cast the combined 16-bit value)
    data->gyro_x = (int16_t)((raw_data[8]  << 8) | raw_data[9])  / GYRO_SENSITIVITY_250;
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]) / GYRO_SENSITIVITY_250;
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]) / GYRO_SENSITIVITY_250;

    // Apply calibration
    if (calibration.calibrated) {
        data->accel_x -= calibration.accel_offset_x;
        data->accel_y -= calibration.accel_offset_y;
        data->accel_z -= calibration.accel_offset_z;
        data->gyro_x -= calibration.gyro_offset_x;
        data->gyro_y -= calibration.gyro_offset_y;
        data->gyro_z -= calibration.gyro_offset_z;
    }

    // compute roll and pitch from accelerometer
    // pitch = rotation around Y axis (nose up/down)
    // roll  = rotation around X axis (lean left/right)
    data->pitch = atan2f(data->accel_x, data->accel_z) * (180.0f / M_PI);
    data->roll  = atan2f(data->accel_y, data->accel_z) * (180.0f / M_PI);

    return ESP_OK;
}


esp_err_t xMPU6050_calibrate(void) {
    // assuming flat ground
    ESP_LOGI(TAG, "Calibrating - keep robot LEVEL and STILL!");

    calibration.calibrated = false;

    // initialize
    const int NUM_SAMPLES = 100;
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    // gather samples (100 default)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        mpu6050_data_t sample;
        esp_err_t ret = xMPU6050_read(&sample);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration read failed at sample %d", i);
            return ret;
        }

        accel_sum_x += sample.accel_x;
        accel_sum_y += sample.accel_y;
        accel_sum_z += sample.accel_z;
        gyro_sum_x += sample.gyro_x;
        gyro_sum_y += sample.gyro_y;
        gyro_sum_z += sample.gyro_z;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    calibration.accel_offset_x = accel_sum_x / NUM_SAMPLES;
    calibration.accel_offset_y = accel_sum_y / NUM_SAMPLES;
    // Z should read 9.81 when level, so offset = actual - expected
    calibration.accel_offset_z = (accel_sum_z / NUM_SAMPLES) - 9.81f;

    calibration.gyro_offset_x = gyro_sum_x / NUM_SAMPLES;
    calibration.gyro_offset_y = gyro_sum_y / NUM_SAMPLES;
    calibration.gyro_offset_z = gyro_sum_z / NUM_SAMPLES;

    calibration.calibrated = true;

    ESP_LOGI(TAG, "Calibration complete!");
    ESP_LOGI(TAG, "  Accel offsets: x=%.3f y=%.3f z=%.3f",
             calibration.accel_offset_x, calibration.accel_offset_y, calibration.accel_offset_z);
    ESP_LOGI(TAG, "  Gyro offsets:  x=%.3f y=%.3f z=%.3f",
             calibration.gyro_offset_x, calibration.gyro_offset_y, calibration.gyro_offset_z);
    return ESP_OK;
}


float fMPU6050_get_tilt(void) {
    mpu6050_data_t data;
    if (xMPU6050_read(&data) != ESP_OK) return -1.0f;
    // combined tilt magnitude from roll and pitch (now actually computed in xMPU6050_read)
    return sqrtf(data.roll * data.roll + data.pitch * data.pitch);
}