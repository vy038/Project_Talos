// power_mon_task.c
#include "power_mon_task.h"
#include "task_config.h"
#include "power/power_management.h"

void vPowerMonTask(void *pvParams) {
    // init power management
    xPowerInit();
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // grab power status, check state,a nd send to state machine if necessary
        power_status_t status = xPowerCheck();
        // only sends for warning and emergencies
        if (status == POWER_EMERGENCY) {
            xQueueSend(xPowerQueue, &status, 0);
        } else if (status == POWER_WARNING) {
            xQueueSend(xPowerQueue, &status, 0);
        }

        // 20ms cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
