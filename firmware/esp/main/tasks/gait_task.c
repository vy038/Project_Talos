// gait_task.c
#include "gait_task.h"
#include "task_config.h"

void vGaitTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: read gait queue and call xGaitUpdate()
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 20ms = GAIT_UPDATE_MS
    }
}
