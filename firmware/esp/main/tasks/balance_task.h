/**
 * @file balance_task.h
 * @brief FreeRTOS task for IMU-based balance correction.
 *
 * Only created at startup if the MPU6050 is detected on the I2C bus.
 * Runs at 20ms intervals, matching the gait update rate.
 *
 * Each cycle it calls xBalanceUpdate() which runs a complementary filter
 * on the MPU6050 accelerometer and gyroscope data to compute pitch and roll.
 * Per-leg knee corrections are stored internally and read by the gait task
 * via xBalanceGetCorrections() on its next update cycle.
 *
 * Also monitors tilt via bBalanceIsTipping(). If tilt exceeds the threshold,
 * the state machine is signaled to transition to EMERGENCY.
 */

#ifndef BALANCE_TASK_H
#define BALANCE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void vBalanceTask(void *pvParams);

#endif
