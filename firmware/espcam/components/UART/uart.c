
#include "uart.h"
#include "esp_err.h"
#include "driver/uart.h"
#include <stdbool.h>

#define BAUD_RATE       115200

bool initialized = false;

esp_err_t xUARTInit(void) {
    // setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(UART_Num, TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

// *SEND THE BYTE 0x01 TO INTERRUPT CAM SENSOR ON POLLING, THEN POLL AND WAIT FOR BYTES TO SEND*
esp_err_t xUARTWrite(const uint8_t *data, size_t len) {
    int written = uart_write_bytes(UART_NUM_2, (const char*)data, len);
    if (written < 0) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t xUARTRead(uint8_t *data, size_t max_len, size_t *bytes_read) {
    int len = uart_read_bytes(UART_NUM_2, data, max_len, portMAX_DELAY);
    if (len < 0) return ESP_FAIL;
    *bytes_read = len;
    return ESP_OK;
}