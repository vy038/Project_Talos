#include "vision_processor.h"
#include "camera.h"
#include "esp_camera.h"
#include "vl53l0x.h"
#include "i2c.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "VISION";

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

// VL53L0X TOF sensor I2C pins and config
#define TOF_I2C_PORT    I2C_NUM_1
#define TOF_SCL_PIN     41
#define TOF_SDA_PIN     40
#define TOF_ADDRESS     0x29
#define TOF_TIMEOUT_MS  500

static vl53l0x_t *tof_dev = NULL;
static uint16_t last_distance_mm = 0;

esp_err_t xVisionProcessorInit(void) {

    // init camera
    esp_err_t ret = xCameraInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return ret;
    }

    // init VL53L0X TOF sensor on separate I2C bus
    tof_dev = vl53l0x_config(TOF_I2C_PORT, TOF_SCL_PIN, TOF_SDA_PIN, -1, TOF_ADDRESS, 1);
    if (!tof_dev) {
        ESP_LOGE(TAG, "VL53L0X config failed - check wiring");
        return ESP_FAIL;
    }

    const char *err = vl53l0x_init(tof_dev);
    if (err) {
        ESP_LOGE(TAG, "VL53L0X init failed: %s", err);
        vl53l0x_end(tof_dev);
        tof_dev = NULL;
        return ESP_FAIL;
    }

    vl53l0x_setTimeout(tof_dev, TOF_TIMEOUT_MS);
    vl53l0x_startContinuous(tof_dev, 0);

    ESP_LOGI(TAG, "Vision processor initialized (camera + TOF)");
    return ESP_OK;
}

esp_err_t xCaptureFrame(uint8_t *frame_buffer, size_t *frame_size) {

    // capture image
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }

    // check if frame fits in buffer
    if (fb->len > *frame_size) {
        ESP_LOGW(TAG, "Frame too large: %d bytes (buffer: %d)", fb->len, *frame_size);
        esp_camera_fb_return(fb);
        return ESP_ERR_INVALID_SIZE;
    }

    // transfer image from camera to esp32 memory
    memcpy(frame_buffer, fb->buf, fb->len);
    *frame_size = fb->len;

    // free camera frame buffer (RAM freed)
    esp_camera_fb_return(fb);
    return ESP_OK;
}

void vRgbToHsv(uint8_t *rgb_frame, uint8_t *hsv_frame, int width, int height) {
    int num_pixels = width * height;

    // for all pixels, convert RGB565 to HSV and store in output buffer (saves memory by converting in-place without intermediate RGB888 step)
    for (int i = 0; i < num_pixels; i++) {
        uint16_t rgb565 = ((uint16_t)rgb_frame[i*2] << 8) | rgb_frame[i*2 + 1];

        uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
        uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
        uint8_t b = (rgb565 & 0x1F) * 255 / 31;

        uint8_t max_val = MAX(MAX(r, g), b);
        uint8_t min_val = MIN(MIN(r, g), b);
        uint8_t delta = max_val - min_val;

        uint8_t h = 0;
        uint8_t s = 0;
        uint8_t v = max_val;

        if (delta != 0) {
            s = (delta * 255) / max_val;

            if (max_val == r) {
                int h_temp = ((g - b) * 60) / delta;
                if (h_temp < 0) h_temp += 360;
                h = h_temp / 2;
            } else if (max_val == g) {
                h = (((b - r) * 60) / delta + 120) / 2;
            } else {
                h = (((r - g) * 60) / delta + 240) / 2;
            }
        }

        hsv_frame[i*3 + 0] = h;
        hsv_frame[i*3 + 1] = s;
        hsv_frame[i*3 + 2] = v;
    }
}

void vThresholdRedRange(uint8_t *hsv_frame, uint8_t *binary_mask, int width, int height) {
    int num_pixels = width * height;

    // create mask for red objects based on HSV thresholds 
    for (int i = 0; i < num_pixels; i++) {
        uint8_t h = hsv_frame[i*3 + 0];
        uint8_t s = hsv_frame[i*3 + 1];
        uint8_t v = hsv_frame[i*3 + 2];

        bool is_red = ((h < 10 || h > 170) && s > 100 && v > 50);

        binary_mask[i] = is_red ? 255 : 0;
    }
}

int iFindLargestBlob(uint8_t *binary_mask, int width, int height, int *centroid_x, int *centroid_y) {
    int largest_blob_size = 0;
    int sum_x = 0;
    int sum_y = 0;

    // find largest blob in binary mask and calculate centroid by averaging pixel coordinates TODO: optimize for performance
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            if (binary_mask[idx] == 255) {
                largest_blob_size++;
                sum_x += x;
                sum_y += y;
            }
        }
    }

    // approximate centroid as average of all pixels in blob
    if (largest_blob_size > 0) {
        *centroid_x = sum_x / largest_blob_size;
        *centroid_y = sum_y / largest_blob_size;
    }

    return largest_blob_size;
}

float fPixelToAngle(int pixel_x, int frame_width) {
    // find angle of pixel relative to center
    float fov_horizontal = 60.0f * (M_PI / 180.0f);
    float center_x = frame_width / 2.0f;
    float offset = (pixel_x - center_x) / (frame_width / 2.0f);
    return offset * (fov_horizontal / 2.0f);
}

uint16_t uiTofReadDistanceMm(void) {
    if (!tof_dev) return 0;

    // continuous mode allows us to just read the latest value without waiting for a new measurement
    uint16_t distance = vl53l0x_readRangeContinuousMillimeters(tof_dev);

    if (vl53l0x_timeoutOccurred(tof_dev)) {
        ESP_LOGW(TAG, "TOF timeout");
        return last_distance_mm;
    }

    // 65535 = out of range
    if (distance == 65535) {
        return last_distance_mm;
    }

    last_distance_mm = distance;
    return distance;
}

bool bTofIsAvailable(void) {
    return (tof_dev != NULL);
}
