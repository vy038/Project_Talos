/**
 * @file uart_stub.c
 * @brief Virtual UART replacing components/UART/uart.c
 *
 * Provides a buffer-backed virtual UART for injecting camera detection
 * packets from the bridge server.
 */

#include "uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UART_SIM";

/* Circular buffer for injected UART data */
#define UART_BUF_SIZE 256
static uint8_t uart_buf[UART_BUF_SIZE];
static size_t uart_buf_head = 0;
static size_t uart_buf_tail = 0;

esp_err_t xUARTInit(void) {
    ESP_LOGI(TAG, "Virtual UART initialized (buffer-backed)");
    uart_buf_head = 0;
    uart_buf_tail = 0;
    return ESP_OK;
}

esp_err_t xUARTWrite(const uint8_t *data, size_t len) {
    /* Log outgoing data - in real robot this goes to ESP32-S3 */
    ESP_LOGD(TAG, "UART TX: %zu bytes", len);
    return ESP_OK;
}

esp_err_t xUARTRead(uint8_t *data, size_t max_len, size_t *bytes_read) {
    *bytes_read = 0;

    while (*bytes_read < max_len && uart_buf_tail != uart_buf_head) {
        data[*bytes_read] = uart_buf[uart_buf_tail];
        uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
        (*bytes_read)++;
    }

    if (*bytes_read == 0) {
        /* Block briefly to simulate waiting for data */
        usleep(10000); /* 10ms */
    }

    return ESP_OK;
}

esp_err_t xUARTReadTimeout(uint8_t *data, size_t max_len, size_t *bytes_read, uint32_t timeout_ms) {
    *bytes_read = 0;
    uint32_t waited = 0;

    while (*bytes_read == 0 && waited < timeout_ms) {
        /* Check for available data */
        while (*bytes_read < max_len && uart_buf_tail != uart_buf_head) {
            data[*bytes_read] = uart_buf[uart_buf_tail];
            uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
            (*bytes_read)++;
        }

        if (*bytes_read == 0) {
            usleep(10000); /* 10ms polling interval */
            waited += 10;
        }
    }

    return ESP_OK;
}

/* Called by bridge server to inject detection packets */
void sim_uart_inject(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        size_t next = (uart_buf_head + 1) % UART_BUF_SIZE;
        if (next == uart_buf_tail) {
            ESP_LOGW(TAG, "UART inject buffer overflow, dropping byte");
            break;
        }
        uart_buf[uart_buf_head] = data[i];
        uart_buf_head = next;
    }
}
