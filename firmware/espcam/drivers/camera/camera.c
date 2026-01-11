#include "camera.h"
#include "esp_log.h"

static const char *TAG = "camera";

// pin definitions (VERIFY WITH ESP32-S3)
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    GPIO_NUM_0
#define CAM_PIN_SIOD    GPIO_NUM_26
#define CAM_PIN_SIOC    GPIO_NUM_27
#define CAM_PIN_D7      GPIO_NUM_35
#define CAM_PIN_D6      GPIO_NUM_34
#define CAM_PIN_D5      GPIO_NUM_39
#define CAM_PIN_D4      GPIO_NUM_36
#define CAM_PIN_D3      GPIO_NUM_21
#define CAM_PIN_D2      GPIO_NUM_19
#define CAM_PIN_D1      GPIO_NUM_18
#define CAM_PIN_D0      GPIO_NUM_5
#define CAM_PIN_VSYNC   GPIO_NUM_25
#define CAM_PIN_HREF    GPIO_NUM_23
#define CAM_PIN_PCLK    GPIO_NUM_22

esp_err_t camera_init(void) {
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
    }
    return err;
}
