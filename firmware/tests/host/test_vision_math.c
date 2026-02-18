/**
 * @file test_vision_math.c
 * @brief Host-side tests for vision processing pure math functions.
 *
 * Tests RGB->HSV conversion, red thresholding, blob finding, and
 * pixel-to-angle mapping. All operate on memory buffers with no HW deps.
 */

#include "esp_stubs.h"

/* We only need the pure math functions, stub out camera/TOF types */
typedef void camera_fb_t;

#include "vision_processor.h"
/* Include the .c for the implementations we test */
/* We need to stub out the HW-dependent functions, so include selectively */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "test_harness.h"

/* --- Re-implement the pure functions here from vision_processor.c --- */
/* This avoids pulling in camera.h/vl53l0x.h/esp_camera.h dependencies */

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

void vRgbToHsv(uint8_t *rgb_frame, uint8_t *hsv_frame, int width, int height) {
    int num_pixels = width * height;
    for (int i = 0; i < num_pixels; i++) {
        uint16_t rgb565 = ((uint16_t)rgb_frame[i*2] << 8) | rgb_frame[i*2 + 1];
        uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
        uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
        uint8_t b = (rgb565 & 0x1F) * 255 / 31;
        uint8_t max_val = MAX(MAX(r, g), b);
        uint8_t min_val = MIN(MIN(r, g), b);
        uint8_t delta = max_val - min_val;
        uint8_t h = 0, s = 0, v = max_val;
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
    int sum_x = 0, sum_y = 0;
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
    if (largest_blob_size > 0) {
        *centroid_x = sum_x / largest_blob_size;
        *centroid_y = sum_y / largest_blob_size;
    }
    return largest_blob_size;
}

float fPixelToAngle(int pixel_x, int frame_width) {
    float fov_horizontal = 60.0f * (M_PI / 180.0f);
    float center_x = frame_width / 2.0f;
    float offset = (pixel_x - center_x) / (frame_width / 2.0f);
    return offset * (fov_horizontal / 2.0f);
}

/* ---------- helpers ---------- */

/* Pack an RGB565 pixel (big-endian, matching vision_processor.c) */
static void pack_rgb565(uint8_t *buf, uint8_t r5, uint8_t g6, uint8_t b5) {
    uint16_t pixel = ((r5 & 0x1F) << 11) | ((g6 & 0x3F) << 5) | (b5 & 0x1F);
    buf[0] = (pixel >> 8) & 0xFF;
    buf[1] = pixel & 0xFF;
}

/* ---------- tests ---------- */

TEST(pixel_to_angle_center) {
    /* Center pixel should give ~0 angle */
    float angle = fPixelToAngle(160, 320);
    ASSERT_NEAR(angle, 0.0f, 0.01f);
}

TEST(pixel_to_angle_edges) {
    /* Left edge should be negative, right edge positive */
    float left = fPixelToAngle(0, 320);
    float right = fPixelToAngle(319, 320);

    ASSERT_TRUE(left < 0);
    ASSERT_TRUE(right > 0);

    /* FOV is 60°, so edges should be ~±30° = ±0.524 rad */
    float half_fov = 30.0f * M_PI / 180.0f;
    ASSERT_NEAR(fabs(left), half_fov, 0.02f);
    ASSERT_NEAR(fabs(right), half_fov, 0.02f);
}

TEST(rgb_to_hsv_pure_red) {
    /* Pure red: R=31, G=0, B=0 in RGB565 */
    uint8_t rgb[2];
    pack_rgb565(rgb, 31, 0, 0);

    uint8_t hsv[3];
    vRgbToHsv(rgb, hsv, 1, 1);

    /* H should be 0 (red), S should be 255, V should be 255 */
    ASSERT_EQ(hsv[0], 0);
    ASSERT_EQ(hsv[1], 255);
    ASSERT_EQ(hsv[2], 255);
}

TEST(rgb_to_hsv_pure_green) {
    uint8_t rgb[2];
    pack_rgb565(rgb, 0, 63, 0);

    uint8_t hsv[3];
    vRgbToHsv(rgb, hsv, 1, 1);

    /* H should be ~60 (green in 0-180 range), S=255, V=255 */
    ASSERT_EQ(hsv[0], 60);
    ASSERT_EQ(hsv[1], 255);
    ASSERT_EQ(hsv[2], 255);
}

TEST(rgb_to_hsv_pure_blue) {
    uint8_t rgb[2];
    pack_rgb565(rgb, 0, 0, 31);

    uint8_t hsv[3];
    vRgbToHsv(rgb, hsv, 1, 1);

    /* H should be ~120 (blue in 0-180 range) */
    ASSERT_EQ(hsv[0], 120);
    ASSERT_EQ(hsv[1], 255);
    ASSERT_EQ(hsv[2], 255);
}

TEST(rgb_to_hsv_black) {
    uint8_t rgb[2] = {0, 0};
    uint8_t hsv[3];
    vRgbToHsv(rgb, hsv, 1, 1);

    /* Black: H=0, S=0, V=0 */
    ASSERT_EQ(hsv[0], 0);
    ASSERT_EQ(hsv[1], 0);
    ASSERT_EQ(hsv[2], 0);
}

TEST(threshold_red_detects_red) {
    /* Create HSV pixel that's clearly red: H=5, S=200, V=200 */
    uint8_t hsv[3] = {5, 200, 200};
    uint8_t mask[1];

    vThresholdRedRange(hsv, mask, 1, 1);
    ASSERT_EQ(mask[0], 255);
}

TEST(threshold_red_rejects_green) {
    /* Green: H=60, S=200, V=200 */
    uint8_t hsv[3] = {60, 200, 200};
    uint8_t mask[1];

    vThresholdRedRange(hsv, mask, 1, 1);
    ASSERT_EQ(mask[0], 0);
}

TEST(threshold_red_rejects_low_saturation) {
    /* Red hue but low saturation (grayish) */
    uint8_t hsv[3] = {5, 50, 200};
    uint8_t mask[1];

    vThresholdRedRange(hsv, mask, 1, 1);
    ASSERT_EQ(mask[0], 0);
}

TEST(threshold_red_rejects_low_value) {
    /* Red hue but very dark */
    uint8_t hsv[3] = {5, 200, 30};
    uint8_t mask[1];

    vThresholdRedRange(hsv, mask, 1, 1);
    ASSERT_EQ(mask[0], 0);
}

TEST(threshold_red_wraps_high_hue) {
    /* Red also wraps at high hue values (H > 170) */
    uint8_t hsv[3] = {175, 200, 200};
    uint8_t mask[1];

    vThresholdRedRange(hsv, mask, 1, 1);
    ASSERT_EQ(mask[0], 255);
}

TEST(blob_find_empty) {
    /* All-zero mask should return 0 */
    uint8_t mask[16];
    memset(mask, 0, sizeof(mask));

    int cx, cy;
    int size = iFindLargestBlob(mask, 4, 4, &cx, &cy);
    ASSERT_EQ(size, 0);
}

TEST(blob_find_single_pixel) {
    uint8_t mask[16];
    memset(mask, 0, sizeof(mask));
    mask[5] = 255;  /* row=1, col=1 in 4x4 grid */

    int cx, cy;
    int size = iFindLargestBlob(mask, 4, 4, &cx, &cy);

    ASSERT_EQ(size, 1);
    ASSERT_EQ(cx, 1);
    ASSERT_EQ(cy, 1);
}

TEST(blob_find_centroid) {
    /* 3x3 block at top-left of 8x8 grid */
    uint8_t mask[64];
    memset(mask, 0, sizeof(mask));
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++)
            mask[y * 8 + x] = 255;

    int cx, cy;
    int size = iFindLargestBlob(mask, 8, 8, &cx, &cy);

    ASSERT_EQ(size, 9);
    ASSERT_EQ(cx, 1);  /* centroid of 0,1,2 = 1 */
    ASSERT_EQ(cy, 1);
}

/* ---------- main ---------- */

int main(void) {
    printf("Vision Math Tests:\n");

    RUN_TEST(pixel_to_angle_center);
    RUN_TEST(pixel_to_angle_edges);
    RUN_TEST(rgb_to_hsv_pure_red);
    RUN_TEST(rgb_to_hsv_pure_green);
    RUN_TEST(rgb_to_hsv_pure_blue);
    RUN_TEST(rgb_to_hsv_black);
    RUN_TEST(threshold_red_detects_red);
    RUN_TEST(threshold_red_rejects_green);
    RUN_TEST(threshold_red_rejects_low_saturation);
    RUN_TEST(threshold_red_rejects_low_value);
    RUN_TEST(threshold_red_wraps_high_hue);
    RUN_TEST(blob_find_empty);
    RUN_TEST(blob_find_single_pixel);
    RUN_TEST(blob_find_centroid);

    TEST_REPORT();
}
