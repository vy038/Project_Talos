/**
 * @file uart_stub.c
 * @brief Virtual UART replacing components/UART/uart.c
 *
 * Provides a buffer-backed virtual UART for injecting camera detection
 * packets from the bridge server. Thread-safe: sim_uart_inject() is called
 * from the stdin reader thread while xUARTReadTimeout() is called from
 * uart_cam_task — both protected by uart_mutex.
 */

#include "uart.h"
#include "esp_log.h"
#include <string.h>
#include <pthread.h>
#include <unistd.h>

static const char *TAG = "UART_SIM";

#define UART_BUF_SIZE 256
static uint8_t  uart_buf[UART_BUF_SIZE];
static size_t   uart_buf_head = 0;
static size_t   uart_buf_tail = 0;
static pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  uart_cond  = PTHREAD_COND_INITIALIZER;

esp_err_t xUARTInit(void) {
    ESP_LOGI(TAG, "Virtual UART initialized (buffer-backed, thread-safe)");
    pthread_mutex_lock(&uart_mutex);
    uart_buf_head = 0;
    uart_buf_tail = 0;
    pthread_mutex_unlock(&uart_mutex);
    return ESP_OK;
}

esp_err_t xUARTWrite(const uint8_t *data, size_t len) {
    ESP_LOGD(TAG, "UART TX: %zu bytes (ping to S3)", len);
    return ESP_OK;
}

esp_err_t xUARTRead(uint8_t *data, size_t max_len, size_t *bytes_read) {
    *bytes_read = 0;
    pthread_mutex_lock(&uart_mutex);
    while (uart_buf_tail == uart_buf_head) {
        pthread_cond_wait(&uart_cond, &uart_mutex);
    }
    while (*bytes_read < max_len && uart_buf_tail != uart_buf_head) {
        data[(*bytes_read)++] = uart_buf[uart_buf_tail];
        uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    }
    pthread_mutex_unlock(&uart_mutex);
    return ESP_OK;
}

esp_err_t xUARTReadTimeout(uint8_t *data, size_t max_len, size_t *bytes_read, uint32_t timeout_ms) {
    *bytes_read = 0;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }

    pthread_mutex_lock(&uart_mutex);
    while (uart_buf_tail == uart_buf_head) {
        if (pthread_cond_timedwait(&uart_cond, &uart_mutex, &ts) != 0) break;
    }
    while (*bytes_read < max_len && uart_buf_tail != uart_buf_head) {
        data[(*bytes_read)++] = uart_buf[uart_buf_tail];
        uart_buf_tail = (uart_buf_tail + 1) % UART_BUF_SIZE;
    }
    pthread_mutex_unlock(&uart_mutex);

    if (*bytes_read > 0)
        ESP_LOGI(TAG, "UART RX: %zu bytes received", *bytes_read);

    return ESP_OK;
}

void sim_uart_inject(const uint8_t *data, size_t len) {
    pthread_mutex_lock(&uart_mutex);
    // Overwrite semantics: flush stale data so only the latest packet is ever
    // in the buffer.  Without this, no-detection packets pile up while the ball
    // is off-screen and the real detection packet has to wait ~15 × 50ms cycles
    // to drain them, causing a ~750ms search-state reaction delay.
    uart_buf_head = 0;
    uart_buf_tail = 0;

    size_t injected = 0;
    for (size_t i = 0; i < len; i++) {
        size_t next = (uart_buf_head + 1) % UART_BUF_SIZE;
        if (next == uart_buf_tail) {
            ESP_LOGW(TAG, "UART inject buffer overflow, dropping %zu bytes", len - i);
            break;
        }
        uart_buf[uart_buf_head] = data[i];
        uart_buf_head = next;
        injected++;
    }
    if (injected > 0) {
        ESP_LOGD(TAG, "UART injected %zu bytes [%02X %02X %02X ...]",
                 injected, data[0],
                 len > 1 ? data[1] : 0,
                 len > 2 ? data[2] : 0);
        pthread_cond_signal(&uart_cond);
    }
    pthread_mutex_unlock(&uart_mutex);
}
