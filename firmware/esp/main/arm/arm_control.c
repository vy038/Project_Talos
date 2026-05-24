/**
 * @file arm_control.c
 * @brief 4-DOF robotic arm control (base, shoulder, elbow, gripper)
 *
 * Drives 4 servos on the arm PCA9685 (0x41). Moves joints incrementally
 * towards target angles each update cycle. All I2C transactions use retry
 * with bus recovery on timeout.
 */

#include "arm_control.h"
#include "servo.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "ARM";

#define ARM_BASE_CH      6
#define ARM_SHOULDER_CH  7
#define ARM_ELBOW_CH     8
#define ARM_GRIPPER_CH   9

#define ARM_STEP_DEG         0.5f    // arm joints (base, shoulder, elbow)
#define ARM_GRIPPER_STEP_DEG 4.0f   // gripper servo rate is faster: 170 deg in ~0.85s vs regular 13.6s
#define ANGLE_TOLERANCE      2.0f
#define I2C_RETRIES          3

static arm_angles_t current_angles = {90.0f, 90.0f, 90.0f, 90.0f};
static arm_angles_t target_angles  = {90.0f, 90.0f, 90.0f, 90.0f};
static arm_state_t current_state = ARM_IDLE;

esp_err_t xArmControlInit(void) {
    // initialize arm by setting all servos to 90 degrees (neutral position)
    current_state = ARM_IDLE;

    esp_err_t ret = xPCA9685Init(I2C_MASTER_NUM, PCA9685_ARM_ADDR, SERVO_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 arm init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = xArmSetAngleWithRetry(ARM_BASE_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize base servo");
        return ret;
    }

    ret = xArmSetAngleWithRetry(ARM_SHOULDER_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize shoulder servo");
        return ret;
    }

    ret = xArmSetAngleWithRetry(ARM_ELBOW_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize elbow servo");
        return ret;
    }

    ret = xArmSetAngleWithRetry(ARM_GRIPPER_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize gripper servo");
        return ret;
    }

    ESP_LOGI(TAG, "Arm control initialized at 0x%02X", PCA9685_ARM_ADDR);
    return ESP_OK;
}

esp_err_t xArmSetAngles(const arm_angles_t *angles) {
    // just set target angles and let update function handle the rest
    if (!angles) {
        return ESP_ERR_INVALID_ARG;
    }
    target_angles = *angles;
    current_state = ARM_MOVING;
    return ESP_OK;
}

esp_err_t xArmSetAngleWithRetry(uint8_t channel, uint8_t angle) {
    // same as xPCA9685SetAngle but with retry logic and bus recovery on timeout, returns last error if all retries fail
    esp_err_t ret;
    for (int attempt = 0; attempt < I2C_RETRIES; attempt++) {
        ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, channel, angle);
        if (ret == ESP_OK) return ESP_OK;
        ESP_LOGW(TAG, "I2C retry %d for arm ch%d", attempt + 1, channel);
        if (ret == ESP_ERR_TIMEOUT) {
            xI2cBusRecovery();
            xPCA9685Init(I2C_MASTER_NUM, PCA9685_ARM_ADDR, SERVO_PWM_FREQ_HZ);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ret;
}


// TODO: calibrate arm properly
esp_err_t xArmUpdate(void) {
    if (current_state != ARM_MOVING) {
        return ESP_OK;
    }

    // structure current and target angles 
    float *cur[4] = {
        &current_angles.base, &current_angles.shoulder,
        &current_angles.elbow, &current_angles.gripper
    };
    const float *tgt[4] = {
        &target_angles.base, &target_angles.shoulder,
        &target_angles.elbow, &target_angles.gripper
    };

    // move each joint towards target, gripper uses a faster step rate
    bool still_moving = false;
    for (int i = 0; i < 4; i++) {
        float step = (i == 3) ? ARM_GRIPPER_STEP_DEG : ARM_STEP_DEG;
        float err = *tgt[i] - *cur[i];
        if (fabsf(err) > step) {
            *cur[i] += (err > 0.0f) ? step : -step;
            still_moving = true;
        } else {
            *cur[i] = *tgt[i];
        }
    }

    // Send clamped updated angles to servos
    uint8_t base_angle     = (uint8_t)fmaxf(0, fminf(180, current_angles.base));
    uint8_t shoulder_angle = (uint8_t)fmaxf(0, fminf(180, current_angles.shoulder));
    uint8_t elbow_angle    = (uint8_t)fmaxf(0, fminf(180, current_angles.elbow));
    uint8_t gripper_angle  = (uint8_t)fmaxf(0, fminf(180, current_angles.gripper));

    esp_err_t ret = xArmSetAngleWithRetry(ARM_BASE_CH, base_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    ret = xArmSetAngleWithRetry(ARM_SHOULDER_CH, shoulder_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    ret = xArmSetAngleWithRetry(ARM_ELBOW_CH, elbow_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    ret = xArmSetAngleWithRetry(ARM_GRIPPER_CH, gripper_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    // change state once target is reached
    if (!still_moving) {
        current_state = ARM_IDLE;
        ESP_LOGD(TAG, "Arm at target: base=%.1f, shoulder=%.1f, elbow=%.1f, gripper=%.1f",
                 current_angles.base, current_angles.shoulder,
                 current_angles.elbow, current_angles.gripper);
    }

    return ESP_OK;
}

esp_err_t xArmGripper(float angle) {
    target_angles.gripper = fmaxf(0.0f, fminf(180.0f, angle));
    current_state = ARM_MOVING;
    return ESP_OK;
}

arm_state_t xArmGetState(void) {
    return current_state;
}

bool bArmAtTarget(void) {
    // check how far angles are from target angles, if all within tolerance then we are at target
    float base_err     = fabsf(current_angles.base     - target_angles.base);
    float shoulder_err = fabsf(current_angles.shoulder - target_angles.shoulder);
    float elbow_err    = fabsf(current_angles.elbow    - target_angles.elbow);
    float gripper_err  = fabsf(current_angles.gripper  - target_angles.gripper);

    return (base_err     < ANGLE_TOLERANCE &&
            shoulder_err < ANGLE_TOLERANCE &&
            elbow_err    < ANGLE_TOLERANCE &&
            gripper_err  < ANGLE_TOLERANCE);
}
