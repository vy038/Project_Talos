/**
 * @file test_knee_calibration.c
 * @brief Interactive per-leg knee and hip neutral calibration
 *
 * Controls each knee and hip servo interactively over serial so you can dial
 * in the exact neutral for each leg.
 *
 * Controls (type in idf.py monitor):
 *   0-5   select active leg
 *   k     switch to knee mode
 *   h     switch to hip mode
 *   [     move active servo -1 deg
 *   ]     move active servo +1 deg
 *   p     print all current angles (copy into gait_generator.h)
 *   q     quit
 *
 * Leg / channel reference:
 *   Leg 0 Front-Right  hip=ch15  knee=ch12
 *   Leg 1 Mid-Right    hip=ch14  knee=ch11
 *   Leg 2 Rear-Right   hip=ch13  knee=ch10
 *   Leg 3 Rear-Left    hip=ch2   knee=ch5
 *   Leg 4 Mid-Left     hip=ch1   knee=ch4
 *   Leg 5 Front-Left   hip=ch0   knee=ch3
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "i2c.h"
#include "gait_generator.h"

#define TAG "CALIB"

static const uint8_t knee_ch[6] = {12, 11, 10, 5, 4, 3};
static const uint8_t hip_ch[6]  = {15, 14, 13, 2, 1, 0};

static const char *leg_name[6] = {
    "Leg0 Front-Right",
    "Leg1 Mid-Right  ",
    "Leg2 Rear-Right ",
    "Leg3 Rear-Left  ",
    "Leg4 Mid-Left   ",
    "Leg5 Front-Left ",
};

#define SERVO_MIN  0
#define SERVO_MAX  180

static const int default_knee[6] = {
    KNEE_NEUTRAL_L0, KNEE_NEUTRAL_L1, KNEE_NEUTRAL_L2,
    KNEE_NEUTRAL_L3, KNEE_NEUTRAL_L4, KNEE_NEUTRAL_L5,
};
static const int default_hip[6] = {
    HIP_NEUTRAL_L0, HIP_NEUTRAL_L1, HIP_NEUTRAL_L2,
    HIP_NEUTRAL_L3, HIP_NEUTRAL_L4, HIP_NEUTRAL_L5,
};

static void print_angles(int knee[6], int hip[6]) {
    printf("\n--- Current neutrals ---\n");
    for (int i = 0; i < 6; i++) {
        printf("  %s  hip ch%2d = %3d deg  knee ch%2d = %3d deg\n",
               leg_name[i], hip_ch[i], hip[i], knee_ch[i], knee[i]);
    }
    printf("\nPaste into gait_generator.h:\n");
    printf("  #define KNEE_NEUTRAL_L0  %d\n", knee[0]);
    printf("  #define KNEE_NEUTRAL_L1  %d\n", knee[1]);
    printf("  #define KNEE_NEUTRAL_L2  %d\n", knee[2]);
    printf("  #define KNEE_NEUTRAL_L3  %d\n", knee[3]);
    printf("  #define KNEE_NEUTRAL_L4  %d\n", knee[4]);
    printf("  #define KNEE_NEUTRAL_L5  %d\n", knee[5]);
    printf("  #define HIP_NEUTRAL_L0   %d\n", hip[0]);
    printf("  #define HIP_NEUTRAL_L1   %d\n", hip[1]);
    printf("  #define HIP_NEUTRAL_L2   %d\n", hip[2]);
    printf("  #define HIP_NEUTRAL_L3   %d\n", hip[3]);
    printf("  #define HIP_NEUTRAL_L4   %d\n", hip[4]);
    printf("  #define HIP_NEUTRAL_L5   %d\n\n", hip[5]);
}

void test_knee_calibration(void) {
    printf("\n========================================\n");
    printf("  Knee & Hip Neutral Calibration\n");
    printf("========================================\n");
    printf("  0-5  select leg\n");
    printf("  k    knee mode\n");
    printf("  h    hip mode\n");
    printf("  [    active servo -1 deg\n");
    printf("  ]    active servo +1 deg\n");
    printf("  p    print all angles\n");
    printf("  q    quit\n");
    printf("========================================\n\n");

    esp_err_t ret = xPCA9685Init(I2C_NUM_0, PCA9685_BODY_ADDR, SERVO_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return;
    }

    int knee_angles[6], hip_angles[6];
    for (int i = 0; i < 6; i++) {
        knee_angles[i] = default_knee[i];
        hip_angles[i]  = default_hip[i];
    }

    for (int i = 0; i < 6; i++) {
        xPCA9685SetAngle(I2C_NUM_0, PCA9685_BODY_ADDR, hip_ch[i],  (uint8_t)default_hip[i]);
        xPCA9685SetAngle(I2C_NUM_0, PCA9685_BODY_ADDR, knee_ch[i], (uint8_t)default_knee[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    int active = 0;
    int mode = 0;  // 0 = knee, 1 = hip
    printf("Active: %s  [KNEE]  = %d deg\n", leg_name[active], knee_angles[active]);

    int c;
    while ((c = getchar()) != 'q') {
        if (c == EOF || c < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (c >= '0' && c <= '5') {
            active = c - '0';
            if (mode == 0)
                printf("Active: %s  [KNEE]  = %d deg\n", leg_name[active], knee_angles[active]);
            else
                printf("Active: %s  [HIP]   = %d deg\n", leg_name[active], hip_angles[active]);

        } else if (c == 'k') {
            mode = 0;
            printf("Mode: KNEE  |  %s  knee = %d deg\n", leg_name[active], knee_angles[active]);

        } else if (c == 'h') {
            mode = 1;
            printf("Mode: HIP   |  %s  hip  = %d deg\n", leg_name[active], hip_angles[active]);

        } else if (c == '[') {
            if (mode == 0) {
                if (knee_angles[active] > SERVO_MIN) {
                    knee_angles[active]--;
                    xPCA9685SetAngle(I2C_NUM_0, PCA9685_BODY_ADDR, knee_ch[active], (uint8_t)knee_angles[active]);
                    printf("  %s  [KNEE]  = %3d deg\n", leg_name[active], knee_angles[active]);
                } else {
                    printf("  %s  [KNEE] already at min (%d)\n", leg_name[active], SERVO_MIN);
                }
            } else {
                if (hip_angles[active] > SERVO_MIN) {
                    hip_angles[active]--;
                    xPCA9685SetAngle(I2C_NUM_0, PCA9685_BODY_ADDR, hip_ch[active], (uint8_t)hip_angles[active]);
                    printf("  %s  [HIP]   = %3d deg\n", leg_name[active], hip_angles[active]);
                } else {
                    printf("  %s  [HIP]  already at min (%d)\n", leg_name[active], SERVO_MIN);
                }
            }

        } else if (c == ']') {
            if (mode == 0) {
                if (knee_angles[active] < SERVO_MAX) {
                    knee_angles[active]++;
                    xPCA9685SetAngle(I2C_NUM_0, PCA9685_BODY_ADDR, knee_ch[active], (uint8_t)knee_angles[active]);
                    printf("  %s  [KNEE]  = %3d deg\n", leg_name[active], knee_angles[active]);
                } else {
                    printf("  %s  [KNEE] already at max (%d)\n", leg_name[active], SERVO_MAX);
                }
            } else {
                if (hip_angles[active] < SERVO_MAX) {
                    hip_angles[active]++;
                    xPCA9685SetAngle(I2C_NUM_0, PCA9685_BODY_ADDR, hip_ch[active], (uint8_t)hip_angles[active]);
                    printf("  %s  [HIP]   = %3d deg\n", leg_name[active], hip_angles[active]);
                } else {
                    printf("  %s  [HIP]  already at max (%d)\n", leg_name[active], SERVO_MAX);
                }
            }

        } else if (c == 'p') {
            print_angles(knee_angles, hip_angles);
        }
    }

    print_angles(knee_angles, hip_angles);
    printf("Calibration done.\n");
}
