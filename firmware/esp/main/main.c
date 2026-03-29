#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"
#include "power_monitor.h"
#include "state_machine/state_machine.h"
#include "power/power_management.h"
#include "tasks/task_config.h"
#include "tasks/state_machine_task.h"
#include "tasks/gait_task.h"
#include "tasks/arm_ctrl_task.h"
#include "tasks/power_mon_task.h"
#include "tasks/uart_cam_task.h"
#include "tasks/balance_task.h"

// IPC object definitions (extern declared in task_config.h)
QueueHandle_t xFrameQueue;
QueueHandle_t xPowerQueue;
SemaphoreHandle_t xArmSemaphore;
SemaphoreHandle_t xI2CMutex;
TaskHandle_t xGaitTaskHandle;
TaskHandle_t xUartCamTaskHandle;

// select test if testing
#define TEST_SELECT TEST_NONE
#include "tests.h"

void app_main(void) {

    // declare queues/semaphores
    xFrameQueue   = xQueueCreate(1, sizeof(detection_result_t)); // 1 to prevent stale frames
    xPowerQueue   = xQueueCreate(3, sizeof(power_status_t)); // 3 needed for power warning
    xArmSemaphore = xSemaphoreCreateBinary();
    xI2CMutex     = xSemaphoreCreateMutex();

    // init hardware peripherals before any task starts
    esp_err_t ret = xI2cMasterInit();
    if (ret != ESP_OK) {
        printf("I2C init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = xUARTInit();
    if (ret != ESP_OK) {
        printf("UART init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = xACS712Init();
    if (ret != ESP_OK) {
        printf("ACS712 init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // try to init MPU6050 — if not found, run without balance
    bool mpu_available = (xMPU6050_init() == ESP_OK);
    if (!mpu_available) {
        printf("MPU6050 not detected — balance task disabled\n");
    }

    if (TEST_SELECT != TEST_NONE) {
         run_test();
    }

    // make tasks
    xTaskCreatePinnedToCore(vPowerMonTask,     "power_mon",  2048, NULL, 1, NULL,                   1);
    xTaskCreatePinnedToCore(vUartCamTask,      "uart_cam",   3072, NULL, 3, &xUartCamTaskHandle,    1);
    xTaskCreatePinnedToCore(vGaitTask,         "gait",       4096, NULL, 5, &xGaitTaskHandle,       1);
    xTaskCreatePinnedToCore(vArmCtrlTask,      "arm_ctrl",   4096, NULL, 6, NULL,                   1);
    if (mpu_available) {
        xTaskCreatePinnedToCore(vBalanceTask,  "balance",    3072, NULL, 5, NULL,                   1);
    }
    xTaskCreatePinnedToCore(vStateMachineTask, "state_mach", 4096, NULL, 4, NULL,                   1);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
