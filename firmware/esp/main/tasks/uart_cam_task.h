/**
 * @file uart_cam_task.h
 * @brief FreeRTOS task for UART communication with the ESP32-S3 camera.
 *
 * Blocks on ulTaskNotifyTake until the state machine sends a task notification
 * at the end of its 50ms cycle (only in vision states).
 *
 * On notification, sends a single ping byte (CAM_PING_BYTE) over UART to wake
 * the S3 vision task, then blocks waiting for the 11-byte detection packet
 * response. Parses the response using bUARTProtoFeedBuf and overwrites
 * xFrameQueue with the latest detection result.
 *
 * Queue depth is 1 to prevent the state machine from acting on stale frames.
 */

#ifndef UART_CAM_TASK_H
#define UART_CAM_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAM_PING_BYTE   0xAA

void vUartCamTask(void *pvParams);

#endif
