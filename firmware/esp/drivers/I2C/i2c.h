#ifndef I2C_H
#define I2C_H

#include "esp_err.h"
#include <stdint.h>

// i2c config

#define I2C_MASTER_NUM I2C_NUM_0

#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_SCL_IO   22

#define I2C_MASTER_FREQ_HZ100000


/**
 * @brief Initialize I2C master interface
 *
 * Configures I2C bus with the predetermined settings (SDA 21, SCL 22, 100000Hz, I2C num 0)
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cMasterInit(void);

/**
 * @brief Writes a byte to registers
 *
 * Writes a byte to the I2C bus to configure register and send data
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Which register inside the device
 * @param data      The data to write
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cWriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Writes bytes to registers
 *
 * Writes bytes to the I2C bus to configure register and send data
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Which register inside the device
 * @param data      The data to write
 * @param len       Total amount of bytes
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cWriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t data, size_t len);


/**
 * @brief Writes a byte to registers
 *
 * Writes a byte to the I2C bus to configure register and send data
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Which register inside the device
 * @param data      The data to update
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Writes a byte to registers
 *
 * Writes a byte to the I2C bus to configure register and send data
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Which register inside the device
 * @param data      The data to update
 * @param len       Number of bytes
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);


#endif