// arm_ctrl_task.c
#include "arm_ctrl_task.h"
#include "task_config.h"
#include "arm/arm_control.h"

void vArmCtrlTask(void *pvParams) {
    // initialize arm control
    xArmControlInit();

    while (1) {
        // wait for state machine to trigger arm control, will keep giving if arm controls state
        xSemaphoreTake(xArmSemaphore, portMAX_DELAY);

        while (1) {
            // take i2c mutex and update arm control, if can't get mutex in time, skip cycle to avoid blocking other tasks
            if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                xArmUpdate();
                xSemaphoreGive(xI2CMutex);
            }

            // 20ms cycle time
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
