// uart_cam_task.c
#include "uart_cam_task.h"
#include "task_config.h"

void vUartCamTask(void *pvParams) {
    while (1) {
        // frozen here until state_machine wakes
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // TODO: logic + impliment "ping" signal to wake up esp32 s3
        /*
        once signal for ping is given (clock from state_machine's end cycle), 
        this task wakes, then send ping to esp32 s3 to wake it from blocking

        then it will wait for cam output,
        then read the frame, decode the data,
        then send the data to state_machine via queue (only 1 to prevent stale frames)

        void vUARTProtoBuildDetection(uint8_t *buf, const talos_detection_t *det)
        {
            buf[0]  = TALOS_PKT_START0;
            buf[1]  = TALOS_PKT_START1;
            buf[2]  = TALOS_PKT_TYPE_DETECT;
            buf[3]  = det->detected ? 1u : 0u;
            buf[4]  = (uint8_t)((det->x >> 8) & 0xFF);
            buf[5]  = (uint8_t)( det->x       & 0xFF);
            buf[6]  = (uint8_t)((det->y >> 8) & 0xFF);
            buf[7]  = (uint8_t)( det->y       & 0xFF);
            buf[8]  = (uint8_t)((det->r >> 8) & 0xFF);
            buf[9]  = (uint8_t)( det->r       & 0xFF);
            buf[10] = compute_checksum(buf);
        }

        use this (bUARTProtoFeedBuf) to decode the data from cam, then send the decoded data to state_machine via queue
        */
    }
}