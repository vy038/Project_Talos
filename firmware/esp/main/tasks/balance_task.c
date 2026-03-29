// balance_task.c
#include "balance_task.h"
#include "task_config.h"

void vBalanceTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: run balance update and apply corrections to gait
        /*
        runs complementary filter on MPU6050 data every 20ms to get pitch/roll
        computes per-leg knee corrections based on tilt
        passes corrections into gait via xBalanceGetCorrections() — gait reads
        them on its next xGaitUpdate(knee_corrections) call

        also checks bBalanceIsTipping(), if tilt exceeds threshold,
        signal state machine to go to EMERGENCY
        */

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // 20ms = BALANCE_UPDATE_PERIOD_MS
    }
}
