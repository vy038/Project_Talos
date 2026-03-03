/**
 * @file i2c.h
 * @brief I2C master driver for Project Talos
 *
 * Single-bus I2C master on ESP32 GPIO21 (SDA) / GPIO22 (SCL).
 * Shared by: PCA9685 body (0x40), PCA9685 arm (0x41), MPU6050 (0x68).
 * Uses external 2.2k-4.7k pullups. Includes bus recovery for lockup handling.
 */

#ifndef I2C_H
#define I2C_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>

/* ========================================================================== */
/*  I2C Bus Configuration                                                      */
/* ========================================================================== */

#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_TIMEOUT_MS      1000

/* ========================================================================== */
/*  Bus Init / Recovery                                                        */
/* ========================================================================== */

/**
 * @brief Initialize I2C master interface
 *
 * Configures I2C bus with the settings defined above. Runs bus recovery
 * before init in case a slave is holding SDA low from a previous crash.
 *
 * @return ESP_OK on success
 */
esp_err_t xI2cMasterInit(void);

/**
 * @brief Runtime I2C bus recovery
 *
 * Tears down the I2C driver, bit-bangs 9 clock pulses per the I2C spec to
 * release a stuck SDA line, then reinitializes the driver. Call when a
 * transaction returns ESP_ERR_TIMEOUT to recover from a bus lockup.
 * After recovery, device drivers (PCA9685, etc.) must be reinitialized.
 *
 * @return ESP_OK if bus recovered and driver reinit succeeded
 */
esp_err_t xI2cBusRecovery(void);

/* ========================================================================== */
/*  Read / Write Transactions                                                  */
/* ========================================================================== */

/**
 * @brief Write a single byte to a device register
 *
 * @param dev_addr  7-bit I2C address
 * @param reg_addr  Target register address
 * @param data      Byte to write
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on bus lockup
 */
esp_err_t xI2cWriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Write multiple bytes starting at a register (auto-increment)
 *
 * @param dev_addr  7-bit I2C address
 * @param reg_addr  Starting register address
 * @param data      Buffer of bytes to write
 * @param len       Number of bytes to write
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on bus lockup
 */
esp_err_t xI2cWriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Read a single byte from a device register
 *
 * Sets register pointer via write, then reads via repeated start.
 *
 * @param dev_addr  7-bit I2C address
 * @param reg_addr  Target register address
 * @param data      Pointer to store the read byte
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on bus lockup
 */
esp_err_t xI2cReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Read multiple bytes starting at a register (auto-increment)
 *
 * Sets register pointer via write, then reads len bytes via repeated start.
 *
 * @param dev_addr  7-bit I2C address
 * @param reg_addr  Starting register address
 * @param data      Buffer to store the read bytes
 * @param len       Number of bytes to read
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on bus lockup
 */
esp_err_t xI2cReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

#endif