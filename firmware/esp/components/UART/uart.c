#include "uart.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "UART";

static bool uart_initialized = false;

esp_err_t xUARTInit(void) {
    if (uart_initialized) {
        ESP_LOGW(TAG, "UART already initialized");
        return ESP_OK;
    }

    // configure UART parameters FIRST (must come before driver install)
    uart_config_t uart_config = {
        .baud_rate = UART_INTER_ESP_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_INTER_ESP_NUM, &uart_config));

    // set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_INTER_ESP_NUM, UART_PIN_TX, UART_PIN_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // install UART driver AFTER config and pin setup
    ESP_ERROR_CHECK(uart_driver_install(UART_INTER_ESP_NUM, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));

    uart_initialized = true;
    ESP_LOGI(TAG, "UART%d initialized (TX:%d RX:%d @ %d baud)",
             UART_INTER_ESP_NUM, UART_PIN_TX, UART_PIN_RX, UART_INTER_ESP_BAUD);
    return ESP_OK;
}

// *SEND THE BYTE 0x01 TO INTERRUPT CAM SENSOR ON POLLING, THEN POLL AND WAIT FOR BYTES TO SEND*
esp_err_t xUARTWrite(const uint8_t *data, size_t len) {
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int written = uart_write_bytes(UART_INTER_ESP_NUM, (const char*)data, len);
    if (written < 0) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t xUARTRead(uint8_t *data, size_t max_len, size_t *bytes_read) {
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // blocks indefinitely until data arrives
    int len = uart_read_bytes(UART_INTER_ESP_NUM, data, max_len, portMAX_DELAY);
    if (len < 0) return ESP_FAIL;
    *bytes_read = len;
    return ESP_OK;
}

esp_err_t xUARTReadTimeout(uint8_t *data, size_t max_len, size_t *bytes_read, uint32_t timeout_ms) {
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int len = uart_read_bytes(UART_INTER_ESP_NUM, data, max_len, pdMS_TO_TICKS(timeout_ms));
    if (len < 0) return ESP_FAIL;
    *bytes_read = len;
    return ESP_OK;
}