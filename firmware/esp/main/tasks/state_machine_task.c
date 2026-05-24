// state_machine_task.c
#include "state_machine_task.h"
#include "task_config.h"
#include "state_machine/state_machine.h"
#include "power/power_management.h"

void vStateMachineTask(void *pvParams) {
    // init state machine
    xStateMachineInit();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // initial state
    robot_state_t prev_state = STATE_INIT;

    while (1) {
        // checks power queue first
        power_status_t pwr;
        if (xQueueReceive(xPowerQueue, &pwr, 0) == pdTRUE) {
            if (pwr == POWER_EMERGENCY) vStateMachineForceState(STATE_EMERGENCY);
        }

        // checks camera queue
        detection_result_t frame;
        if (xQueueReceive(xFrameQueue, &frame, 0) == pdTRUE) {
            vStateMachineFeedDetection(frame);
        }

        // update state accordingly
        xStateMachineUpdate();

        // processes state transition for arm control and camera task triggering
        robot_state_t cur_state = xStateMachineGetState();
        if (prev_state != STATE_GRAB_PREP && cur_state == STATE_GRAB_PREP) {
            // NOTE: do NOT vTaskSuspend(xGaitTaskHandle) here. The gait task holds
            // xI2CMutex during xGaitUpdate(). Suspending it mid-hold permanently
            // deadlocks the arm task (which needs xI2CMutex to run xArmUpdate()).
            // Gait was already stopped via MOVE_STOP at end of handle_approach.
            xSemaphoreGive(xArmSemaphore);
        }
        prev_state = cur_state;

        // trigger camera task for states that require active tracking
        if (cur_state == STATE_SEARCH   ||
            cur_state == STATE_ALIGN    ||
            cur_state == STATE_APPROACH ||
            cur_state == STATE_GRAB_PREP) {
            xTaskNotifyGive(xUartCamTaskHandle);
        }

        // 50ms cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
