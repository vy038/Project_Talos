#ifndef UART_H
#define UART_H

#include "esp_err.h"
#include "hal/gpio_types.h"
#include <stdint.h>

// UART configuration for inter-ESP32 communication
#define UART_INTER_ESP_NUM      UART_NUM_2
#define UART_INTER_ESP_BAUD     115200
#define UART_PIN_TX             GPIO_NUM_18
#define UART_PIN_RX             GPIO_NUM_19

/**
 * @brief Initialize UART
 *
 * Initialize UART2 for communication with the ESP32-S3 Camera.
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTInit(void);

/**
 * @brief Write bytes over UART
 *
 * Write data on the UART line
 * @param data The data to write
 * @param len Length of data to write
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTWrite(const uint8_t *data, size_t len);

/**
 * @brief Read bytes from UART
 *
 * Read data from the UART line (blocks until data available)
 * @param data Buffer to store received data
 * @param max_len Maximum number of bytes to read
 * @param bytes_read Actual number of bytes read
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTRead(uint8_t *data, size_t max_len, size_t *bytes_read);

/**
 * @brief Read bytes from UART with timeout
 *
 * Read data from the UART line with a timeout in milliseconds
 * @param data Buffer to store received data
 * @param max_len Maximum number of bytes to read
 * @param bytes_read Actual number of bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, error code on failure/timeout
 */
esp_err_t xUARTReadTimeout(uint8_t *data, size_t max_len, size_t *bytes_read, uint32_t timeout_ms);

#endif