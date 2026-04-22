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

// select test if testing
#define TEST_SELECT TEST_NONE
#include "tests.h"

// IPC object definitions (extern declared in task_config.h)
QueueHandle_t xFrameQueue;
QueueHandle_t xPowerQueue;
SemaphoreHandle_t xArmSemaphore;
SemaphoreHandle_t xI2CMutex;
TaskHandle_t xGaitTaskHandle;
TaskHandle_t xUartCamTaskHandle;

// void vStackMonitorTask(void *pvParameters) {
//     const char *task_names[] = {"power_mon", "uart_cam", "gait", "arm_ctrl", "balance", "state_mach", "IDLE"};
//     const int num_tasks = sizeof(task_names) / sizeof(task_names[0]);

//     while (1) {
//         printf("\n=== Stack High Water Mark ===\n");
//         for (int i = 0; i < num_tasks; i++) {
//             TaskHandle_t handle = xTaskGetHandle(task_names[i]);
//             if (handle != NULL) {
//                 UBaseType_t hwm = uxTaskGetStackHighWaterMark(handle);
//                 printf("%s: %u bytes free\n", task_names[i], hwm * sizeof(StackType_t));
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }
// }

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

    // try to init MPU6050, if not found, run without balance
    bool mpu_available = (xMPU6050_init() == ESP_OK);
    if (!mpu_available) {
        printf("MPU6050 not detected — balance task disabled\n");
    }

    if (TEST_SELECT != TEST_NONE) {
         run_test();
    }

    // make all tasks

    /*  stack usage
        power_mon:  2292 bytes free
        uart_cam:   2472 bytes free
        gait:       2204 bytes free
        arm_ctrl:   2224 bytes free
        balance:    2076 bytes free
        state_mach: 2208 bytes free
        IDLE:       1036 bytes free
    */

    xTaskCreatePinnedToCore(vPowerMonTask,     "power_mon",  4096, NULL, 1, NULL,                   1);
    xTaskCreatePinnedToCore(vUartCamTask,      "uart_cam",   3072, NULL, 3, &xUartCamTaskHandle,    1);
    xTaskCreate            (vGaitTask,         "gait",       4096, NULL, 5, &xGaitTaskHandle         );
    xTaskCreate            (vArmCtrlTask,      "arm_ctrl",   4096, NULL, 6, NULL                     );
    if (mpu_available) {
        xTaskCreatePinnedToCore(vBalanceTask,  "balance",    4096, NULL, 5, NULL,                   1);
    }
    xTaskCreatePinnedToCore(vStateMachineTask, "state_mach", 4096, NULL, 4, NULL,                   1);
    // xTaskCreatePinnedToCore(vStackMonitorTask, "stack_mon",  2048, NULL, 1, NULL,                   0);
}
