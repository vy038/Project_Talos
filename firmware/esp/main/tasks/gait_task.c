// gait_task.c
#include "gait_task.h"
#include "task_config.h"

void vGaitTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: await state machine command and call xGaitUpdate()
        /*
        gait update from state machine will update the gait speed and decision

        it updates every 20ms in real time and takes updates,
        keeps gait up at all times and just sends an update every 20 ms
        */

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 20ms = GAIT_UPDATE_MS
    }
}
