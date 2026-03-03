/**
 * @file test_uart.c
 * @brief UART loopback test - sends a message and reads the response
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart.h"

#define TAG "TEST_UART"

void test_uart(void) {
    printf("UART Communication Test\n\n");

    esp_err_t ret = xUARTInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(ret));
        return;
    }

    const char *msg = "HELLO";
    ret = xUARTWrite((uint8_t *)msg, strlen(msg));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART write failed: %s", esp_err_to_name(ret));
        return;
    }
    printf("Sent: %s\n", msg);

    printf("Waiting for response...\n");
    uint8_t rx[128];
    size_t len = 0;
    ret = xUARTRead(rx, sizeof(rx) - 1, &len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART read failed: %s", esp_err_to_name(ret));
        return;
    }

    rx[len] = '\0';
    printf("Received: %s (%d bytes)\n", rx, (int)len);
}
