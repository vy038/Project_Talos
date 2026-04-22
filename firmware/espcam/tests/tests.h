/**
 * @file tests.h
 * @brief Unified test runner for Project Talos ESP-CAM tests
 *
 * Usage: In main.c, define TEST_SELECT before including:
 *   #define TEST_SELECT TEST_RED_BALL
 *   #include "tests.h"
 *
 * Then call run_test() from app_main after hardware init.
 * Leave TEST_SELECT as TEST_NONE for normal production operation.
 */

#ifndef TESTS_H
#define TESTS_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

/* ========================================================================== */
/*  Test IDs                                                                   */
/* ========================================================================== */

#define TEST_NONE       0
#define TEST_CAMERA     1
#define TEST_UART       2
#define TEST_VL53L0X    3
#define TEST_RED_BALL   4
#define TEST_I2C_SCAN   5
#define TEST_GPIO       6

/* ========================================================================== */
/*  Test function declarations                                                 */
/* ========================================================================== */

void test_camera(void);
void test_uart(void);
void test_vl53l0x(uint8_t scl, uint8_t sda);
void test_red_ball(void);
void test_i2c_scan(uint8_t scl, uint8_t sda);
void test_gpio(void);

/* ========================================================================== */
/*  Test runner                                                                */
/* ========================================================================== */

#ifndef TEST_SELECT
#define TEST_SELECT TEST_NONE
#endif

// TOF sensor pins (matches vision_processor.c)
#define TESTS_TOF_SCL   41
#define TESTS_TOF_SDA   40

static inline void run_test(void) {
    printf("\n========================================\n");
    printf("  Project Talos ESP-CAM - Test Runner\n");
    printf("========================================\n\n");

    switch (TEST_SELECT) {
        case TEST_CAMERA:
            printf("[TEST] Camera\n");
            test_camera();
            break;
        case TEST_UART:
            printf("[TEST] UART\n");
            test_uart();
            break;
        case TEST_VL53L0X:
            printf("[TEST] VL53L0X TOF\n");
            test_vl53l0x(TESTS_TOF_SCL, TESTS_TOF_SDA);
            break;
        case TEST_RED_BALL:
            printf("[TEST] Red Ball Detection\n");
            test_red_ball();
            break;
        case TEST_I2C_SCAN:
            printf("[TEST] I2C Address Scanner\n");
            test_i2c_scan(TESTS_TOF_SCL, TESTS_TOF_SDA);
            break;
        case TEST_GPIO:
            printf("[TEST] GPIO 40/41 Toggle\n");
            test_gpio();
            break;
        default:
            printf("[INFO] No test selected.\n");
            break;
    }

    printf("\n[DONE] Test complete.\n");
}

#endif
