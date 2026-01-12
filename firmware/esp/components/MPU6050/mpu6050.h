#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include <stdbool.h>

// i2c addr
#define MPU6050_ADDR                0x68
// sleep mode or cycle mode
#define MPU6050_REG_PWR_MGMT_1      0x6B
// verify connection
#define MPU6050_REG_WHO_AM_I        0x75
// gyro config
#define MPU6050_REG_GYRO_CONFIG     0x1B
// accel config
#define MPU6050_REG_ACCEL_CONFIG    0x1C

/** 
 * Start of multiple reading addresses
 * 0x3B: ACCEL_XOUT_H
 * 0x3C: ACCEL_XOUT_L
 * 0x3D: ACCEL_YOUT_H
 * 0x3E: ACCEL_YOUT_L
 * 0x3F: ACCEL_ZOUT_H
 * 0x40: ACCEL_ZOUT_L
 * 0x41: TEMP_OUT_H
 * 0x42: TEMP_OUT_L
 * 0x43: GYRO_XOUT_H
 * 0x44: GYRO_XOUT_L
 * 0x45: GYRO_YOUT_H
 * 0x46: GYRO_YOUT_L
 * 0x47: GYRO_ZOUT_H
 * 0x48: GYRO_ZOUT_L
 **/
#define MPU6050_REG_ACCEL_XOUT_H    0x3B

// IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;    // m/s²
    float gyro_x, gyro_y, gyro_z;       // degrees/sec
    float roll, pitch;                   // degrees (calculated)
} mpu6050_data_t;

// calibration data structure
static struct {
    float accel_offset_x, accel_offset_y, accel_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    bool calibrated;
} calibration = {0};

/**
 * @brief Initialize MPU
 *
 * Initializes the MPU for usage (Make sure I2C init() is called!)
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xMPU6050_init(void);

/**
 * @brief Reads all sensor data (excluding temp)
 *
 * Reads all the sensor data on the MPu and writes all the info to a data struct
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xMPU6050_read(mpu6050_data_t *data);

/**
 * @brief Calibrates the MPu6050
 *
 * Calibrates MPU6050 to be relative to groudn its currently on
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xMPU6050_calibrate(void);

/**
 * @brief Gets tilt angle magnitudes
 *
 * Gets tilt angle magnitudes, needed for fine adjustment of arm and balancing
 * 
 * @return float value of tilt
 */
float fMPU6050_get_tilt(void);

#endif
