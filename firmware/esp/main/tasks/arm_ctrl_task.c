// arm_ctrl_task.c
#include "arm_ctrl_task.h"
#include "task_config.h"
#include "arm/arm_control.h"

void vArmCtrlTask(void *pvParams) {
    xArmControlInit();
    while (1) {
        // frozen here until state_machine gives the semaphore
        xSemaphoreTake(xArmSemaphore, portMAX_DELAY);

        // TODO: logic
        /*
        once semaphore is given, arm_ctrl will execute the corresponding logic 
        based on the current state of the state machine

        it will take the coordinates given from the state machine,
        then slowly with each call move the arm towards those coordinates
        camera will continuously send commands to the state machine, and 
        the state machine will update the coordinates for the arm in real time
        */
    }
}