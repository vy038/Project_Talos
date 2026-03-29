#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// IPC queues
extern QueueHandle_t xFrameQueue;   // uart_cam → state_machine (detection_result_t)
extern QueueHandle_t xPowerQueue;   // power_mon → state_machine (power_status_t)

// Semaphores / mutexes
extern SemaphoreHandle_t xArmSemaphore;  // state_machine → arm_ctrl (one-time trigger)
extern SemaphoreHandle_t xI2CMutex;      // gait + arm_ctrl share I2C bus

// Task handles (needed for suspend/notify)
extern TaskHandle_t xGaitTaskHandle;
extern TaskHandle_t xUartCamTaskHandle;
