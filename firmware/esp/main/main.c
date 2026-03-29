#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"

// task config
#include "task_config.h"
QueueHandle_t xFrameQueue;
QueueHandle_t xPowerQueue;
SemaphoreHandle_t xArmSemaphore;
SemaphoreHandle_t xI2CMutex;
TaskHandle_t xGaitTaskHandle;
TaskHandle_t xArmTaskHandle;

// select test if testing
// #define TEST_SELECT TEST_GAIT
// #include "tests.h"

void app_main(void) {

    // declare queues/semaphores
    xFrameQueue   = xQueueCreate(1, sizeof(detection_result_t)); // 1 to prevent stale frames
    xPowerQueue   = xQueueCreate(3, sizeof(power_status_t)); // 3 needed for power warning
    xArmSemaphore = xSemaphoreCreateBinary();
    xI2CMutex     = xSemaphoreCreateMutex();

    // make tasks
    xTaskCreatePinnedToCore(vPowerMonTask, "power_mon", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vUartCamTask, "uart_cam", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vGaitTask, "gait", 4096, NULL, 5, &xGaitTaskHandle, 1);
    xTaskCreatePinnedToCore(vArmCtrlTask, "arm_ctrl",   4096, NULL, 6, &xArmTaskHandle, 1);
    xTaskCreatePinnedToCore(vStateMachineTask, "state_mach", 4096, NULL, 4, NULL, 1);

    // init i2c master
    esp_err_t ret = xI2cMasterInit();
    if (ret != ESP_OK) {
        printf("I2C init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // run_test(); (not testing)

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
