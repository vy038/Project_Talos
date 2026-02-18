#include "balance_control.h"
#include "mpu6050.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "BALANCE";

static body_orientation_t orientation = {0};
static bool balance_enabled = true;
static bool initialized = false;

// per-leg correction factors based on physical position TODO: verify these with actual robot geometry. Assumes legs are numbered 0-5 starting at front-right and going clockwise.
// positive pitch_factor = front of body, positive roll_factor = right side
static const float leg_pitch_factor[6] = {
     0.7f,   // Leg 0: Front-Right
     0.0f,   // Leg 1: Mid-Right
    -0.7f,   // Leg 2: Rear-Right
    -0.7f,   // Leg 3: Rear-Left
     0.0f,   // Leg 4: Mid-Left
     0.7f,   // Leg 5: Front-Left
};

static const float leg_roll_factor[6] = {
     0.7f,   // Leg 0: Front-Right
     1.0f,   // Leg 1: Mid-Right
     0.7f,   // Leg 2: Rear-Right
    -0.7f,   // Leg 3: Rear-Left
    -1.0f,   // Leg 4: Mid-Left
    -0.7f,   // Leg 5: Front-Left
};

esp_err_t xBalanceInit(void) {
    // initialize balance checking, make sure mpu is initialized before this
    mpu6050_data_t imu;
    esp_err_t ret = xMPU6050_read(&imu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 for init");
        return ret;
    }

    // TODO: verify MPU6050 mounting orientation. X=forward, Y=left, Z=down
    // calculate roll, pitch, and yaw
    orientation.pitch = atan2f(imu.accel_x, -imu.accel_z) * (180.0f / M_PI);
    orientation.roll  = atan2f(imu.accel_y, -imu.accel_z) * (180.0f / M_PI);
    orientation.yaw_rate = 0.0f;

    initialized = true;
    ESP_LOGI(TAG, "Balance initialized. Pitch: %.1f, Roll: %.1f",
             orientation.pitch, orientation.roll);
    return ESP_OK;
}

esp_err_t xBalanceUpdate(void) {
    if (!initialized) return ESP_ERR_INVALID_STATE;

    mpu6050_data_t imu;
    esp_err_t ret = xMPU6050_read(&imu);
    if (ret != ESP_OK) return ret;

    // how much time has passed since the last update
    const float dt = BALANCE_UPDATE_PERIOD_MS / 1000.0f;

    // pitch and roll calculations
    float accel_pitch = atan2f(imu.accel_x, -imu.accel_z) * (180.0f / M_PI);
    float accel_roll  = atan2f(imu.accel_y, -imu.accel_z) * (180.0f / M_PI);

    // TODO: verify if pitch/roll are swapped (orientation)
    float gyro_pitch = orientation.pitch + imu.gyro_y * dt;
    float gyro_roll  = orientation.roll  + imu.gyro_x * dt;

    // filtering to clean noise: complementary filter blending gyro and accel data
    orientation.pitch = BALANCE_COMP_FILTER_ALPHA * gyro_pitch
                      + (1.0f - BALANCE_COMP_FILTER_ALPHA) * accel_pitch;
    orientation.roll  = BALANCE_COMP_FILTER_ALPHA * gyro_roll
                      + (1.0f - BALANCE_COMP_FILTER_ALPHA) * accel_roll;

    orientation.yaw_rate = imu.gyro_z;

    return ESP_OK;
}

body_orientation_t xBalanceGetOrientation(void) {
    return orientation;
}

balance_correction_t xBalanceGetCorrections(void) {
    // get corrections required from legs
    balance_correction_t corr = {0};
    if (!balance_enabled || !initialized) return corr;

    float pitch = orientation.pitch;
    float roll  = orientation.roll;

    if (fabsf(pitch) < BALANCE_DEADBAND_DEG) pitch = 0.0f;
    if (fabsf(roll)  < BALANCE_DEADBAND_DEG) roll  = 0.0f;

    // update legs according to their position on the body, with limits to prevent over-correction
    // TODO: update positions of legs accoridngly (gait structure for angles)
    for (int i = 0; i < 6; i++) {
        float correction = 0.0f;
        correction += -BALANCE_KP_PITCH * pitch * leg_pitch_factor[i];
        correction += -BALANCE_KP_ROLL  * roll  * leg_roll_factor[i];

        if (correction > BALANCE_MAX_CORRECTION_DEG) correction = BALANCE_MAX_CORRECTION_DEG;
        if (correction < -BALANCE_MAX_CORRECTION_DEG) correction = -BALANCE_MAX_CORRECTION_DEG;

        corr.knee_offset[i] = correction;
    }

    return corr;
}

void vBalanceEnable(bool enable) {
    // toggle for balance
    balance_enabled = enable;
    ESP_LOGI(TAG, "Balance corrections %s", enable ? "enabled" : "disabled");
}

bool bBalanceIsTipping(float threshold_deg) {
    // check if tipping beyond a certain threshold, used for safety shutdowns
    float total_tilt = sqrtf(orientation.pitch * orientation.pitch
                           + orientation.roll * orientation.roll);
    return total_tilt > threshold_deg;
}