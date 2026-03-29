/**
 * @file state_machine_task.h
 * @brief FreeRTOS task wrapper for the state machine.
 *
 * Runs at 50ms intervals. Each cycle it drains the frame queue and power queue,
 * feeds new data into the state machine, then calls xStateMachineUpdate().
 *
 * Also handles task coordination:
 *   - Pings uart_cam via task notification when in vision states (SEARCH, ALIGN, APPROACH, GRAB_PREP)
 *   - Suspends gait task and gives arm semaphore on transition into GRAB_PREP
 */

#ifndef STATE_MACHINE_TASK_H
#define STATE_MACHINE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void vStateMachineTask(void *pvParams);

#endif
