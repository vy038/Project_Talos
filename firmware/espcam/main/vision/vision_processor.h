#ifndef VISION_PROCESSOR_H
#define VISION_PROCESSOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize vision processor
 *
 * Sets up camera interface and processing buffers
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xVisionProcessorInit(void);

/**
 * @brief Capture camera frame
 *
 * Grabs frame from OV2640 camera
 *
 * @param frame_buffer Output buffer for frame data
 * @param frame_size Output frame size in bytes
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xCaptureFrame(uint8_t *frame_buffer, size_t *frame_size);

/**
 * @brief Convert RGB to HSV color space
 *
 * Transforms RGB frame to HSV for color filtering
 *
 * @param rgb_frame Input RGB frame buffer
 * @param hsv_frame Output HSV frame buffer
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 */
void vRgbToHsv(uint8_t *rgb_frame, uint8_t *hsv_frame, int width, int height);

/**
 * @brief Threshold red color range
 *
 * Creates binary mask for red objects
 *
 * @param hsv_frame Input HSV frame buffer
 * @param binary_mask Output binary mask (0 or 255)
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 */
void vThresholdRedRange(uint8_t *hsv_frame, uint8_t *binary_mask, int width, int height);

/**
 * @brief Find largest blob in binary mask
 *
 * Locates largest connected component and calculates centroid
 *
 * @param binary_mask Input binary mask
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 * @param centroid_x Output centroid X coordinate
 * @param centroid_y Output centroid Y coordinate
 * @return int Blob size in pixels, 0 if no blob found
 */
int iFindLargestBlob(uint8_t *binary_mask, int width, int height, int *centroid_x, int *centroid_y);

/**
 * @brief Convert pixel coordinate to bearing angle
 *
 * Maps pixel X position to horizontal angle
 *
 * @param pixel_x Pixel X coordinate
 * @param frame_width Frame width in pixels
 * @return float Bearing angle in radians
 */
float fPixelToAngle(int pixel_x, int frame_width);

/**
 * @brief Read distance from VL53L0X TOF sensor
 *
 * Returns last valid reading on timeout or out-of-range.
 * Uses continuous mode for fast readings.
 *
 * @return distance in millimeters, 0 if sensor not available
 */
uint16_t uiTofReadDistanceMm(void);

/**
 * @brief Check if TOF sensor is initialized and available
 *
 * @return true if sensor is ready
 */
bool bTofIsAvailable(void);

#endif