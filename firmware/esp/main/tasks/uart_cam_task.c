// uart_cam_task.c
#include "uart_cam_task.h"
#include "task_config.h"
#include "uart_protocol/uart_protocol.h"
#include "state_machine/state_machine.h"
#include "uart.h"
#include "esp_log.h"

static const char *TAG = "UART_CAM";

void vUartCamTask(void *pvParams) {
    talos_framer_t framer = {0};
    uint8_t ping = CAM_PING_BYTE;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xUARTWrite(&ping, 1);

        uint8_t rx[TALOS_PKT_LEN];
        size_t bytes_read = 0;
        esp_err_t ret = xUARTReadTimeout(rx, TALOS_PKT_LEN, &bytes_read, 200);

        if (ret != ESP_OK || bytes_read == 0) {
            ESP_LOGW(TAG, "No response from cam (ret=%d, bytes=%d)", ret, bytes_read);
            continue;
        }

        ESP_LOGD(TAG, "Rx %d bytes: %02X %02X %02X ...", bytes_read, rx[0], rx[1], rx[2]);

        talos_detection_t td;
        size_t consumed;
        if (bUARTProtoFeedBuf(&framer, rx, bytes_read, &consumed, &td)) {
            ESP_LOGI(TAG, "Detection: det=%d x=%d y=%d px_r=%d tof=%dmm",
                     td.detected, td.x, td.y, td.px_r, td.tof_mm);
            detection_result_t result = {
                .detected     = td.detected,
                .ball_x       = td.x,
                .ball_y       = td.y,
                .pixel_radius = td.px_r,
                .tof_dist_mm  = td.tof_mm,
                .fresh        = true,
            };
            xQueueOverwrite(xFrameQueue, &result);
        } else {
            ESP_LOGW(TAG, "Parse failed after %d bytes", bytes_read);
        }
    }
}
