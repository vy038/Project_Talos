/**
 * MPU6050 IMU Driver
 * I2C Address: 0x68
 * Provides: Accelerometer, Gyroscope, Temperature
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "esp_err.h"

// IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;    // m/s²
    float gyro_x, gyro_y, gyro_z;       // degrees/sec
    float temperature;                   // °C
    float roll, pitch;                   // degrees (calculated)
} mpu6050_data_t;

/**
 * Initialize MPU6050
 * NOTE: Call i2c_master_init() first!
 */
esp_err_t xMPU6050_init(void);

/**
 * Read all sensor data
 */
esp_err_t xMPU6050_read(mpu6050_data_t *data);

/**
 * Calibrate (call when robot is level and stationary)
 */
esp_err_t xMPU6050_calibrate(void);

/**
 * Get tilt angle magnitude
 */
float fMPU6050_get_tilt(void);

#endif
