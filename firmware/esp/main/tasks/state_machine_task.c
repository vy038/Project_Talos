// state_machine_task.c
#include "state_machine_task.h"
#include "task_config.h"

void vStateMachineTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: read state machine queue and call xStateMachineUpdate()
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));  // 50ms = STATE_MACHINE_UPDATE_MS
    }
}
