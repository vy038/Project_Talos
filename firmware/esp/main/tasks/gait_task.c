// gait_task.c
#include "gait_task.h"
#include "task_config.h"
#include "locomotion/gait_generator.h"
#include "locomotion/balance_control.h"

void vGaitTask(void *pvParams) {
    // init gait generator
    xGaitInit();
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // get corrections from balance control
        balance_correction_t corrections = xBalanceGetCorrections();

        // update gait generator with corrections, taking and giving i2c mutex as needed
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        xGaitUpdate(corrections.knee_offset);
        xSemaphoreGive(xI2CMutex);

        // 20ms cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
