// uart_cam_task.c
#include "uart_cam_task.h"
#include "task_config.h"
#include "uart_protocol/uart_protocol.h"
#include "state_machine/state_machine.h"
#include "uart.h"

void vUartCamTask(void *pvParams) {
    // creates frame and ping buffers
    talos_framer_t framer = {0};
    uint8_t ping = CAM_PING_BYTE; // 0xAA

    while (1) {
        // waits for notification from state machine task that a new frame is needed
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // sends ping to camera to request a frame
        xUARTWrite(&ping, 1);

        // reads the response from the camera with a timeout (200ms)
        uint8_t rx[TALOS_PKT_LEN];
        size_t bytes_read;
        esp_err_t ret = xUARTReadTimeout(rx, TALOS_PKT_LEN, &bytes_read, 200);
        if (ret != ESP_OK || bytes_read == 0) continue;

        // feeds the received bytes into the frame queue to extract detection data
        talos_detection_t td;
        size_t consumed;
        if (bUARTProtoFeedBuf(&framer, rx, bytes_read, &consumed, &td)) {
            detection_result_t result = {
                .detected     = td.detected,
                .ball_x       = td.x,
                .ball_y       = td.y,
                .pixel_radius = td.px_r,
                .tof_dist_mm  = td.tof_mm,
                .fresh        = true,
            };
            xQueueOverwrite(xFrameQueue, &result);
        }
    }
}
