#ifndef UART_H
#define UART_H

#include "esp_err.h"
#include <stdint.h>

#define UART_PIN_TX       GPIO_NUM_1
#define UART_PIN_RX       GPIO_NUM_3

/**
 * @brief Initialize UART
 *
 * Initialize UART with 115000 baud rate, with the ESP32 S3 Camera
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTInit(void);

/**
 * @brief Write a byte with UART
 *
 * Write a byte of data on the UART line
 * @param data The data to write
 * @param len Length of data to write
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTWrite(const uint8_t *data, size_t len);

/**
 * @brief Read a byte from UART
 *
 * Read a byte of data from the UART line, writing to a data byte variable
 * @param data The data variable to write to
 * @param max_len Maximum number of bytes to read and allocate
 * @param bytes_read Actual number of bytes read
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTRead(uint8_t *data, size_t max_len, size_t *bytes_read);

#endif