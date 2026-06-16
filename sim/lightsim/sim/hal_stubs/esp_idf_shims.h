/**
 * @file esp_idf_shims.h
 * @brief Master shim replacing all ESP-IDF headers for native Linux compilation
 *
 * Provides type definitions, macros, and function stubs for all ESP-IDF APIs
 * used by Project Talos firmware. This file is included by all the thin shim
 * headers in include/ (esp_err.h, esp_log.h, driver/i2c.h, etc.).
 */

#ifndef ESP_IDF_SHIMS_H
#define ESP_IDF_SHIMS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>

/* Redirect firmware printf to stderr so stdout is clean JSON */
#define printf(...) fprintf(stderr, __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  esp_err.h                                                                  */
/* ========================================================================== */

typedef int esp_err_t;

#define ESP_OK                  0
#define ESP_FAIL                (-1)
#define ESP_ERR_NO_MEM          0x101
#define ESP_ERR_INVALID_ARG     0x102
#define ESP_ERR_INVALID_STATE   0x103
#define ESP_ERR_INVALID_SIZE    0x104
#define ESP_ERR_NOT_FOUND       0x105
#define ESP_ERR_NOT_SUPPORTED   0x106
#define ESP_ERR_TIMEOUT         0x107

static inline const char *esp_err_to_name(esp_err_t code) {
    switch (code) {
        case ESP_OK:                return "ESP_OK";
        case ESP_FAIL:              return "ESP_FAIL";
        case ESP_ERR_NO_MEM:        return "ESP_ERR_NO_MEM";
        case ESP_ERR_INVALID_ARG:   return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_TIMEOUT:       return "ESP_ERR_TIMEOUT";
        default:                    return "UNKNOWN_ERROR";
    }
}

/* ========================================================================== */
/*  esp_log.h                                                                  */
/* ========================================================================== */

#define LOG_COLOR_RED     "\033[0;31m"
#define LOG_COLOR_GREEN   "\033[0;32m"
#define LOG_COLOR_YELLOW  "\033[0;33m"
#define LOG_COLOR_BLUE    "\033[0;34m"
#define LOG_COLOR_RESET   "\033[0m"

/* All firmware log output goes to stderr so stdout stays clean for JSON */
#define ESP_LOGE(tag, fmt, ...) \
    fprintf(stderr, LOG_COLOR_RED    "E (%s) " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) \
    fprintf(stderr, LOG_COLOR_YELLOW "W (%s) " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) \
    fprintf(stderr, LOG_COLOR_GREEN  "I (%s) " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) \
    fprintf(stderr, LOG_COLOR_BLUE   "D (%s) " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)

/* ========================================================================== */
/*  esp_timer.h                                                                */
/* ========================================================================== */

static inline int64_t esp_timer_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)ts.tv_nsec / 1000LL;
}

/* ========================================================================== */
/*  FreeRTOS shims                                                             */
/* ========================================================================== */

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef void (*TaskFunction_t)(void *);

#define portTICK_PERIOD_MS      1
#define pdMS_TO_TICKS(ms)       ((TickType_t)(ms))
#define portMAX_DELAY           0xFFFFFFFFU
#define pdTRUE                  1
#define pdFALSE                 0
#define pdPASS                  pdTRUE
#define configMINIMAL_STACK_SIZE 1024
#define configMAX_PRIORITIES    25

/* Forward declare sim_on_tick for state emission during delays */
extern void sim_on_tick(uint32_t delay_ms);

/* Check if current task is suspended; block until resumed if so */
extern void sim_check_suspend(void);

/* ---------- task handle --------------------------------------------------- */

typedef struct sim_task_s {
    pthread_t       thread;
    uint32_t        notify_value;
    pthread_mutex_t notify_mutex;
    pthread_cond_t  notify_cond;
    int             suspended;
    pthread_mutex_t suspend_mutex;
    pthread_cond_t  suspend_cond;
} sim_task_t;

typedef sim_task_t *TaskHandle_t;

/* ---------- queue handle -------------------------------------------------- */

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    uint8_t        *buf;
    size_t          item_size;
    size_t          depth;
    size_t          head, tail, count;
} sim_queue_t;

typedef sim_queue_t *QueueHandle_t;

/* ---------- semaphore handle ---------------------------------------------- */

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    int count;
    int max;
} sim_sem_t;

typedef sim_sem_t *SemaphoreHandle_t;

/* ---------- timing -------------------------------------------------------- */

static inline TickType_t xTaskGetTickCount(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (TickType_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL);
}

/* lets TIME SCALE speed up/slow down the backend firmware tasks themselves,
 * not just the frontend visualization */
extern float sim_get_time_scale(void);

static inline void vTaskDelay(TickType_t ticks) {
    sim_check_suspend();
    usleep((useconds_t)(ticks * 1000U / sim_get_time_scale()));
    sim_on_tick((uint32_t)ticks);
}

static inline void vTaskDelayUntil(TickType_t *pxPreviousWakeTime, TickType_t xTimeIncrement) {
    sim_check_suspend();
    TickType_t target = *pxPreviousWakeTime + xTimeIncrement;
    TickType_t now    = xTaskGetTickCount();
    if ((int32_t)(target - now) > 0) {
        uint32_t sleep_ms = target - now;
        usleep((useconds_t)(sleep_ms * 1000U / sim_get_time_scale()));
        sim_on_tick(sleep_ms);
    } else {
        sim_on_tick(0);
    }
    *pxPreviousWakeTime = xTaskGetTickCount();
}

static inline void vTaskPrioritySet(TaskHandle_t handle, UBaseType_t prio) {
    (void)handle; (void)prio; /* priority not enforced in sim */
}

/* ---------- task creation / control (implemented in freertos_sim.c) ------- */

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t func, const char *name,
    uint32_t stack, void *params, UBaseType_t prio,
    TaskHandle_t *handle_out, BaseType_t core);

static inline BaseType_t xTaskCreate(TaskFunction_t func, const char *name,
    uint32_t stack, void *params, UBaseType_t prio, TaskHandle_t *handle_out)
{
    return xTaskCreatePinnedToCore(func, name, stack, params, prio, handle_out, 1);
}

void vTaskSuspend(TaskHandle_t handle);
void vTaskResume(TaskHandle_t handle);

/* ---------- task notifications -------------------------------------------- */

void xTaskNotifyGive(TaskHandle_t handle);
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout);

/* ---------- queues -------------------------------------------------------- */

QueueHandle_t xQueueCreate(UBaseType_t depth, UBaseType_t item_size);
BaseType_t    xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks);
BaseType_t    xQueueReceive(QueueHandle_t q, void *buf, TickType_t ticks);
BaseType_t    xQueueOverwrite(QueueHandle_t q, const void *item);

/* ---------- semaphores ---------------------------------------------------- */

SemaphoreHandle_t xSemaphoreCreateBinary(void);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t        xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks);
BaseType_t        xSemaphoreGive(SemaphoreHandle_t sem);

/* ========================================================================== */
/*  GPIO types                                                                 */
/* ========================================================================== */

typedef int gpio_num_t;

#define GPIO_NUM_0   0
#define GPIO_NUM_1   1
#define GPIO_NUM_2   2
#define GPIO_NUM_16  16
#define GPIO_NUM_17  17
#define GPIO_NUM_21  21
#define GPIO_NUM_22  22
#define GPIO_NUM_34  34
#define GPIO_NUM_35  35

typedef enum {
    GPIO_MODE_DISABLE = 0,
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_INPUT_OUTPUT,
} gpio_mode_t;

static inline esp_err_t gpio_set_direction(gpio_num_t gpio, gpio_mode_t mode) {
    (void)gpio; (void)mode;
    return ESP_OK;
}

static inline esp_err_t gpio_set_level(gpio_num_t gpio, uint32_t level) {
    (void)gpio; (void)level;
    return ESP_OK;
}

static inline int gpio_get_level(gpio_num_t gpio) {
    (void)gpio;
    return 0;
}

/* ========================================================================== */
/*  I2C driver types                                                           */
/* ========================================================================== */

typedef int i2c_port_t;

#define I2C_NUM_0           0
#define I2C_NUM_1           1
#define I2C_MODE_MASTER     1
#define I2C_MASTER_WRITE    0
#define I2C_MASTER_READ     1

/* I2C command link API stubs (used by test_i2c.c for bus scan) */
typedef void *i2c_cmd_handle_t;

static inline i2c_cmd_handle_t i2c_cmd_link_create(void) {
    return (i2c_cmd_handle_t)1; /* non-null dummy */
}

static inline esp_err_t i2c_master_start(i2c_cmd_handle_t cmd) {
    (void)cmd;
    return ESP_OK;
}

static inline esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd, uint8_t data, bool ack_en) {
    (void)cmd; (void)data; (void)ack_en;
    return ESP_OK;
}

static inline esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd) {
    (void)cmd;
    return ESP_OK;
}

/**
 * Simulated I2C bus scan - returns ESP_OK for known device addresses
 * (PCA9685 body=0x40, PCA9685 arm=0x41, MPU6050=0x68)
 */
static inline esp_err_t i2c_master_cmd_begin(i2c_port_t port, i2c_cmd_handle_t cmd, TickType_t timeout) {
    (void)port; (void)cmd; (void)timeout;
    /* The actual address is encoded in the cmd chain - we can't easily extract it
       from this shim. For the I2C scan test, we'll always return ESP_FAIL.
       The scan test is informational only. */
    return ESP_FAIL;
}

static inline void i2c_cmd_link_delete(i2c_cmd_handle_t cmd) {
    (void)cmd;
}

/* ========================================================================== */
/*  ADC types                                                                  */
/* ========================================================================== */

typedef int adc_channel_t;
typedef int adc_atten_t;
typedef int adc_bitwidth_t;

#define ADC_CHANNEL_6       6
#define ADC_CHANNEL_7       7
#define ADC_ATTEN_DB_11     3
#define ADC_BITWIDTH_12     12

/* ========================================================================== */
/*  UART types                                                                 */
/* ========================================================================== */

#define UART_NUM_0  0
#define UART_NUM_1  1
#define UART_NUM_2  2

/* ========================================================================== */
/*  ROM delay                                                                  */
/* ========================================================================== */

static inline void esp_rom_delay_us(uint32_t us) {
    usleep(us);
}

#ifdef __cplusplus
}
#endif

#endif /* ESP_IDF_SHIMS_H */
