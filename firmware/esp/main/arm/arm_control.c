#include "arm_control.h"
#include "servo.h"
#include "i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "ARM";

#define ARM_BASE_CH      0
#define ARM_SHOULDER_CH  1
#define ARM_ELBOW_CH     2
#define ARM_GRIPPER_CH   5

#define ARM_STEP_DEG    2.0f
#define ANGLE_TOLERANCE 2.0f

static arm_angles_t current_angles = {90.0f, 90.0f, 90.0f, 90.0f};
static arm_angles_t target_angles  = {90.0f, 90.0f, 90.0f, 90.0f};
static arm_state_t current_state = ARM_IDLE;

esp_err_t xArmControlInit(void) {
    // initialize arm by setting all servos to 90 degrees (neutral position)
    current_state = ARM_IDLE;

    esp_err_t ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_BASE_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize base servo");
        return ret;
    }

    ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_SHOULDER_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize shoulder servo");
        return ret;
    }

    ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_ELBOW_CH, 90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize elbow servo");
        return ret;
    }

    ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_GRIPPER_CH, 90);
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

    // move each joint towards target by ARM_STEP_DEG, but don't overshoot
    bool still_moving = false;
    for (int i = 0; i < 4; i++) {
        float err = *tgt[i] - *cur[i];
        if (fabsf(err) > ARM_STEP_DEG) {
            *cur[i] += (err > 0.0f) ? ARM_STEP_DEG : -ARM_STEP_DEG;
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

    esp_err_t ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_BASE_CH, base_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_SHOULDER_CH, shoulder_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_ELBOW_CH, elbow_angle);
    if (ret != ESP_OK) { current_state = ARM_ERROR; return ret; }

    ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_ARM_ADDR, ARM_GRIPPER_CH, gripper_angle);
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
