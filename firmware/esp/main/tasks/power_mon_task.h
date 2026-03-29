/**
 * @file power_mon_task.h
 * @brief FreeRTOS task for background current monitoring.
 *
 * Polls the ACS712 current sensor every 20ms at priority 1. Uses a
 * consecutive-count filter to ignore short inrush spikes from servo startup.
 *
 * On POWER_WARNING, posts status to xPowerQueue for the state machine to handle.
 * On POWER_EMERGENCY, escalates its own priority to configMAX_PRIORITIES - 1
 * via vTaskPrioritySet before posting, so the message reaches the state
 * machine immediately without waiting behind other tasks.
 */

#ifndef POWER_MON_TASK_H
#define POWER_MON_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void vPowerMonTask(void *pvParams);

#endif
