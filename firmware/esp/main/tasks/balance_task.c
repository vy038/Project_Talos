// balance_task.c
#include "balance_task.h"
#include "task_config.h"
#include "locomotion/balance_control.h"

void vBalanceTask(void *pvParams) {
    // initialize balance control
    xBalanceInit();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // update balance every 20ms
        xBalanceUpdate();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
