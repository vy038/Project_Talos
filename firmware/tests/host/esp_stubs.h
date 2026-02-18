/**
 * @file esp_stubs.h
 * @brief Minimal ESP-IDF type stubs for host-side compilation of pure logic.
 */
#ifndef ESP_STUBS_H
#define ESP_STUBS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* esp_err_t */
typedef int esp_err_t;
#define ESP_OK              0
#define ESP_FAIL            -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_SIZE 0x105

/* Logging stubs — silent during tests */
#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGE(tag, fmt, ...)
#define ESP_LOGD(tag, fmt, ...)

#endif
