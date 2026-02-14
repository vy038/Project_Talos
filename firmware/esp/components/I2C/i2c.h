#ifndef I2C_H
#define I2C_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>

// i2c master configuration (change these to match your wiring)
#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000


/**
 * @brief Initialize I2C master interface
 *
 * Configures I2C bus with the settings defined above.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cMasterInit(void);

/**
 * @brief Writes a byte to a register
 *
 * Writes a byte to the I2C bus to configure register and send data
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Which register inside the device
 * @param data      The data to write
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cWriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Writes multiple bytes starting at a register
 *
 * Writes bytes to the I2C bus starting at reg_addr (auto-increment)
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Starting register address
 * @param data      The data to write
 * @param len       Total amount of bytes
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cWriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Reads a byte from a register
 *
 * Sets register pointer via write, then reads one byte via repeated start
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Which register inside the device
 * @param data      Pointer to store the read byte
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Reads multiple bytes starting at a register
 *
 * Sets register pointer via write, then reads len bytes via repeated start
 * @param dev_addr  7-bit I2C address (0x68 for MPU6050, 0x40 for PCA9685)
 * @param reg_addr  Starting register address
 * @param data      Buffer to store the read bytes
 * @param len       Number of bytes to read
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xI2cReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);


#endif