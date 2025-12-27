/**
 * MPU6050 IMU Driver
 * I2C Address: 0x68
 * Provides: Accelerometer, Gyroscope, Temperature
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "mpu6050.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "MPU6050";

#define MPU6050_ADDR         0x68
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

esp_err_t xMPU6050_init(void) {

}

/**
 * Read all sensor data
 */
esp_err_t xMPU6050_read(mpu6050_data_t *data) {

}

/**
 * Calibrate (call when robot is level and stationary)
 */
esp_err_t xMPU6050_calibrate(void) {

}

/**
 * Get tilt angle magnitude
 */
float fMPU6050_get_tilt(char dir) {

}