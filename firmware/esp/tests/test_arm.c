/**
 * @file test_arm.c
 * @brief Arm range-of-motion test with coordinate logging
 *
 * Slowly sweeps each arm joint (base, shoulder, elbow) through its range
 * one at a time, logging servo angles and computed gripper XYZ position (mm)
 * using forward kinematics. Gripper is left at neutral.
 *
 *   - ARM_LINK1_LENGTH (shoulder-to-elbow, mm)
 *   - ARM_LINK2_LENGTH (elbow-to-gripper, mm)
 *   - ARM_BASE_HEIGHT  (body frame to shoulder pivot, mm)
 *   - ARM_BASE_ANGLE   (base mounting tilt, degrees)
 *
 * After running, compare logged positions against physical measurements
 * and update those constants.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "i2c.h"
#include "arm_control.h"
#include "ik_solver.h"

#define TAG "TEST_ARM"

/* arm channels on the shared PCA9685 body board (ch 6-9) */
#define ARM_BASE_CH      6
#define ARM_SHOULDER_CH  7
#define ARM_ELBOW_CH     8
#define ARM_GRIPPER_CH   9

#define SWEEP_STEP_DEG   2          /* degrees per step */
#define SWEEP_DELAY_MS   80         /* ms between steps — slow and safe */
#define NEUTRAL_DEG      90

/* helper: set a single arm channel directly (bypasses arm_control state) */
static esp_err_t set_arm_ch(uint8_t channel, uint8_t angle) {
    esp_err_t ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_BODY_ADDR, channel, angle);
    if (ret != ESP_OK) {
        /* Bus recovery already happened inside i2c.c on timeout. If the device
         * was power-cycled and just reconnected, it needs a fresh PCA9685 init
         * before it will ACK commands. Re-init and retry once. */
        ESP_LOGW(TAG, "ch%d failed (%s) — reiniting PCA9685 and retrying",
                 channel, esp_err_to_name(ret));
        ret = xPCA9685Init(I2C_MASTER_NUM, PCA9685_BODY_ADDR, SERVO_PWM_FREQ_HZ);
        if (ret == ESP_OK) {
            ret = xPCA9685SetAngle(I2C_MASTER_NUM, PCA9685_BODY_ADDR, channel, angle);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set ch%d to %d after recovery: %s",
                     channel, angle, esp_err_to_name(ret));
        }
    }
    return ret;
}

/* helper: compute and log gripper XYZ for a given set of joint angles */
static void log_position(uint8_t base_deg, uint8_t shoulder_deg, uint8_t elbow_deg) {
    ik_solution_t sol = {
        .base_rotation = (float)base_deg,
        .shoulder      = (float)shoulder_deg,
        .elbow         = (float)elbow_deg,
        .valid         = true,
    };
    ik_target_t pos = {0};
    xIKForward(&sol, &pos);

    /* CSV-friendly output: angle_base, angle_shoulder, angle_elbow, x_mm, y_mm, z_mm */
    printf("ARM_POS, %3d, %3d, %3d, %7.1f, %7.1f, %7.1f\n",
           base_deg, shoulder_deg, elbow_deg, pos.x, pos.y, pos.z);
}

/* sweep a single joint from start to end while others stay at neutral */
static void sweep_joint(const char *name, uint8_t channel,
                        uint8_t start, uint8_t end,
                        uint8_t base_fixed, uint8_t shoulder_fixed, uint8_t elbow_fixed,
                        int joint_index) {
    printf("\n--- Sweeping %s: %d -> %d (step %d, %d ms/step) ---\n",
           name, start, end, SWEEP_STEP_DEG, SWEEP_DELAY_MS);
    printf("ARM_POS, base, shoulder, elbow,    x_mm,    y_mm,    z_mm\n");

    int step = (end > start) ? SWEEP_STEP_DEG : -SWEEP_STEP_DEG;
    for (int angle = start; (step > 0) ? (angle <= end) : (angle >= end); angle += step) {
        esp_err_t ret = set_arm_ch(channel, (uint8_t)angle);
        if (ret != ESP_OK) return;

        uint8_t b = base_fixed, s = shoulder_fixed, e = elbow_fixed;
        if (joint_index == 0) b = (uint8_t)angle;
        if (joint_index == 1) s = (uint8_t)angle;
        if (joint_index == 2) e = (uint8_t)angle;

        log_position(b, s, e);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    }

    /* return to neutral */
    set_arm_ch(channel, NEUTRAL_DEG);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void test_arm(void) {
    printf("=== Arm Range-of-Motion Test ===\n");
    printf("PCA9685 addr : 0x%02X (shared body board)\n", PCA9685_BODY_ADDR);
    printf("Channels     : base=%d, shoulder=%d, elbow=%d, gripper=%d\n",
           ARM_BASE_CH, ARM_SHOULDER_CH, ARM_ELBOW_CH, ARM_GRIPPER_CH);
    printf("Sweep        : %d deg/step, %d ms/step\n\n", SWEEP_STEP_DEG, SWEEP_DELAY_MS);

    /* init PCA9685 (may already be init'd, but safe to call again) */
    esp_err_t ret = xPCA9685Init(I2C_MASTER_NUM, PCA9685_BODY_ADDR, SERVO_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* init IK solver so forward kinematics works */
    xIKSolverInit();

    /* park all arm servos at neutral first */
    // set_arm_ch(ARM_BASE_CH, NEUTRAL_DEG);
    // set_arm_ch(ARM_SHOULDER_CH, NEUTRAL_DEG);
    // set_arm_ch(ARM_ELBOW_CH, NEUTRAL_DEG);
    // set_arm_ch(ARM_GRIPPER_CH, NEUTRAL_DEG);
    printf("All joints at %d deg (neutral). Waiting 5s...\n", NEUTRAL_DEG);
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* sweep each joint one at a time, others stay neutral */
    /* joint_index: 0=base, 1=shoulder, 2=elbow */

    // sweep_joint("BASE (rotation)", ARM_BASE_CH,
    //             0, 180, NEUTRAL_DEG, NEUTRAL_DEG, NEUTRAL_DEG, 0);

    // sweep_joint("SHOULDER", ARM_SHOULDER_CH,
    //             0, 180, NEUTRAL_DEG, NEUTRAL_DEG, NEUTRAL_DEG, 1);

    // sweep_joint("ELBOW", ARM_ELBOW_CH,
    //             0, 180, NEUTRAL_DEG, NEUTRAL_DEG, NEUTRAL_DEG, 2);

    set_arm_ch(ARM_BASE_CH, 90); // (50 - 130)
    set_arm_ch(ARM_SHOULDER_CH, 90); // (75 - 150)
    set_arm_ch(ARM_ELBOW_CH, 90); // (55 - 125)
    set_arm_ch(ARM_GRIPPER_CH, 90); // (100 - 140)

    // /* quick gripper test: open -> close -> neutral */
    // printf("\n--- Gripper: 0 -> 180 -> 90 ---\n");
    // for (int angle = 0; angle <= 180; angle += SWEEP_STEP_DEG) {
    //     set_arm_ch(ARM_GRIPPER_CH, (uint8_t)angle);
    //     vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    // }
    // vTaskDelay(pdMS_TO_TICKS(500));
    // for (int angle = 180; angle >= 90; angle -= SWEEP_STEP_DEG) {
    //     set_arm_ch(ARM_GRIPPER_CH, (uint8_t)angle);
    //     vTaskDelay(pdMS_TO_TICKS(SWEEP_DELAY_MS));
    // }

    printf("\n=== Arm test complete ===\n");
    printf("\n");
    printf("TODO: Fill in arm geometry in ik_solver.h then re-run:\n");
    printf("  ARM_LINK1_LENGTH  — measure shoulder pivot to elbow pivot (mm)\n");
    printf("  ARM_LINK2_LENGTH  — measure elbow pivot to gripper tip (mm)\n");
    printf("  ARM_BASE_HEIGHT   — measure body frame origin to shoulder pivot (mm)\n");
    printf("  ARM_BASE_ANGLE    — base mounting tilt (0=horizontal, degrees)\n");
    printf("\n");
    printf("Once filled in, the x/y/z columns above will show computed positions.\n");
    printf("Compare against physical measurements to validate kinematics.\n");
}
