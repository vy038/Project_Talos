#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Ping byte the WROOM sends to request a frame
#define VISION_PING_BYTE    0xAA

// Timeout waiting for WROOM ping (ms). portMAX_DELAY in practice.
#define VISION_PING_TIMEOUT_MS  portMAX_DELAY

void vVisionTask(void *pvParams);

#endif
