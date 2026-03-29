// power_mon_task.c
#include "power_mon_task.h"
#include "task_config.h"

void vPowerMonTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: read power monitor queue and call xPowerMonUpdate()
        /*
        will monitor power and then send power status to state machine via xPowerQueue
        the state machine will then decide whether to trigger the arm or not based on the power status

        task might need to escalate priority if power problems emerge
        and signal to state machine, state machine might need priority increase too
        move the kill switch into this task instead?
        can try directly calling vGaitSetCommand(MOVE_STOP, 0), but wont be as nice
        */

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 20ms = POWER_POLL_MS
    }
}
