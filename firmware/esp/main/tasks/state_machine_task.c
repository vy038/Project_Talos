// state_machine_task.c
#include "state_machine_task.h"
#include "task_config.h"

void vStateMachineTask(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO: read state machine queue and call xStateMachineUpdate()
        /*
        get input from camera task, the frame queue (one frame)
        check power status from power monitor task, the power queue (one status)
        read + process input from cam
        decide on arm or gait
        send appropriate commands to arm if needed, using semaphores for arm, or
        command gait state, gait task updates on its own (20ms) based on state
        recieve real time camera updates so arm/gait commands update in real time as needed
        */

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));  // 50ms = STATE_MACHINE_UPDATE_MS
    }
}
