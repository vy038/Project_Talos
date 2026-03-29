/**
 * @file gait_task.h
 * @brief FreeRTOS task for gait execution.
 *
 * Calls xGaitUpdate() every 20ms using vTaskDelayUntil to keep timing exact.
 * Reads balance corrections from the balance task each cycle and passes them
 * into the gait update. If the balance task is not running, corrections are
 * zero and gait runs normally.
 *
 * The state machine calls vGaitSetCommand() directly to change direction and
 * speed. This task does not read from any queue, it just keeps the gait
 * cycling at the correct rate.
 *
 * Holds xI2CMutex during each servo write to share the I2C bus safely with
 * the arm task.
 */

#ifndef GAIT_TASK_H
#define GAIT_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void vGaitTask(void *pvParams);

#endif
