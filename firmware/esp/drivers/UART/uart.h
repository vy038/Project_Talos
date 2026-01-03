#ifndef UART_H
#define UART_H

#include "esp_err.h"
#include "driver/uart.h"
#include <stdint.h>

#define BAUD_RATE       115000

/**
 * @brief Initialize UART
 *
 * Initialize UART with 115000 baud rate, with the ESP32 S3 Camera
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTInit(void);

/**
 * @brief Read a byte from UART
 *
 * Read a byte of data from the UART line, writing to a data byte
 * @param data      The data to write to
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xUARTRead(uint8_t data);

#endif