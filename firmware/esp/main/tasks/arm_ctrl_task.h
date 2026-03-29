/**
 * @file arm_ctrl_task.h
 * @brief FreeRTOS task for arm control during the grab sequence.
 *
 * Blocks on xArmSemaphore at startup. The state machine gives the semaphore
 * when transitioning into GRAB_PREP, at which point the gait task is also
 * suspended so there is no I2C contention.
 *
 * Once unblocked, calls xArmUpdate() every 20ms to incrementally move the
 * arm toward the target angles set by the state machine. The state machine
 * continues to receive camera updates during this phase and adjusts the
 * target coordinates in real time via xArmSetAngles().
 *
 * Holds xI2CMutex during each servo write to share the I2C bus safely.
 */

#ifndef ARM_CTRL_TASK_H
#define ARM_CTRL_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void vArmCtrlTask(void *pvParams);

#endif
