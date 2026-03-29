// power_mon_task.c
#include "power_mon_task.h"
#include "task_config.h"

void vPowerMonTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: read power monitor queue and call xPowerMonUpdate()
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 20ms = POWER_POLL_MS
    }
}
