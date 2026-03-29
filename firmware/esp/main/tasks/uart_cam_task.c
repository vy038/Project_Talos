// uart_cam_task.c
#include "uart_cam_task.h"
#include "task_config.h"

void vUartCamTask(void *pvParams) {
    while (1) {
        // frozen here until state_machine wakes
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}