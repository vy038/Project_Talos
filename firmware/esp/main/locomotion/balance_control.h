/**
 * @file balance_control.h
 * @brief Complementary filter orientation + per-leg balance corrections
 *
 * Fuses MPU6050 accelerometer (noisy but no drift) with gyroscope (smooth
 * but drifts) using a complementary filter. Output is pitch/roll in degrees.
 *
 * COMPLEMENTARY FILTER:
 *   angle = alpha * (angle + gyro_rate * dt) + (1-alpha) * accel_angle
 *   alpha=0.96 means 96% gyro trust. Accel slowly corrects drift.
 *
 * BALANCE CORRECTIONS:
 * Each leg gets a knee offset based on its position relative to body center.
 * If robot tilts forward, front legs extend and rear legs contract.
 *   correction = -KP * tilt * position_factor
 * The negative sign means corrections OPPOSE the tilt.
 */

#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "esp_err.h"
#include <stdbool.h>

// higher alpha = smoother but more drift. 0.96-0.98 typical.
#define BALANCE_COMP_FILTER_ALPHA   0.96f

// update rate in ms. 20ms = 50Hz. matches servo update and gait update.
#define BALANCE_UPDATE_PERIOD_MS    20

// P gain for corrections. too high = oscillation. start at 0.3, increase carefully.
#define BALANCE_KP_PITCH            0.5f
#define BALANCE_KP_ROLL             0.5f

// max correction per leg in degrees. prevents slamming servos.
#define BALANCE_MAX_CORRECTION_DEG  15.0f

// ignore tilts below this. prevents constant micro-jitter.
#define BALANCE_DEADBAND_DEG        1.5f

typedef struct {
    float pitch;        // degrees, positive = nose up
    float roll;         // degrees, positive = right side up
    float yaw_rate;     // deg/sec from gyro Z (no absolute yaw without magnetometer)
} body_orientation_t;

typedef struct {
    float knee_offset[6];   // correction angle per leg knee, in degrees
} balance_correction_t;

/**
 * @brief Init balance system. Call after MPU6050 init + calibrate.
 *        Seeds filter with initial accel reading.
 * 
 * @return ESP_OK if successful
 */
esp_err_t xBalanceInit(void);

/**
 * @brief Run one filter iteration. Call at BALANCE_UPDATE_PERIOD_MS intervals.
 *        Reads MPU6050 internally.
 * 
 * @return ESP_OK if successful, or error code from MPU6050 read
 */
esp_err_t xBalanceUpdate(void);

/**
 * @brief Get filtered pitch/roll
 * 
 * @return body_orientation_t struct with current pitch/roll in degrees and yaw_rate in deg/sec. Returns zeros if not initialized.
 */
body_orientation_t xBalanceGetOrientation(void);

/**
 * @brief Get per-leg knee corrections. Add these to gait knee angles.
 *        
 * @return balance_correction_t struct with knee offsets in degrees to apply to each leg. Returns zeros if disabled or tilt within deadband.
 */
balance_correction_t xBalanceGetCorrections(void);

/**
 * @brief Enable/disable corrections without stopping the filter
 * 
 * @param enable true to enable corrections, false to disable (but still update orientation)
 */
void vBalanceEnable(bool enable);

/**
 * @brief Returns true if total tilt exceeds threshold (emergency check)
 * 
 * @param threshold_deg tilt angle in degrees above which we consider the robot to be tipping over
 * @return true if tipping, false if safe
 */
bool bBalanceIsTipping(float threshold_deg);

#endif