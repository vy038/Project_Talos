// arm_ctrl_task.c
#include "arm_ctrl_task.h"
#include "task_config.h"
#include "arm/arm_control.h"

void vArmCtrlTask(void *pvParams) {
    xArmControlInit();
    while (1) {
        // frozen here until state_machine gives the semaphore
        xSemaphoreTake(xArmSemaphore, portMAX_DELAY);
    }
}