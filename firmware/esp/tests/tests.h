/**
 * @file tests.h
 * @brief Unified test runner for Project Talos hardware tests
 *
 * Usage: In main.c, define TEST_SELECT to the desired test before including:
 *   #define TEST_SELECT TEST_SERVO_SWEEP
 *   #include "tests.h"
 *
 * Then call run_test() from app_main after hardware init.
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

#define TEST_NONE               0
#define TEST_I2C_SCAN           1
#define TEST_SERVO_SWEEP        2
#define TEST_MPU6050            3
#define TEST_ADC                4
#define TEST_UART               5
#define TEST_POWER              6
#define TEST_GAIT               7
#define TEST_ARM                8

/* ========================================================================== */
/*  Test function declarations                                                 */
/* ========================================================================== */

void test_i2c_scan(void);
void test_servo_sweep(void);
void test_mpu6050(void);
void test_adc(void);
void test_uart(void);
void test_power(void);
void test_gait(void);
void test_arm(void);

/* ========================================================================== */
/*  Test runner                                                                */
/* ========================================================================== */

#ifndef TEST_SELECT
#define TEST_SELECT TEST_NONE
#endif

static inline void run_test(void) {
    printf("\n========================================\n");
    printf("  Project Talos - Hardware Test Runner\n");
    printf("========================================\n\n");

    test_i2c_scan();
    printf("\n");

    switch (TEST_SELECT) {
        case TEST_I2C_SCAN:
            printf("[TEST] I2C Bus Scan\n");
            test_i2c_scan();
            break;
        case TEST_SERVO_SWEEP:
            printf("[TEST] Servo Sweep\n");
            test_servo_sweep();
            break;
        case TEST_MPU6050:
            printf("[TEST] MPU6050 IMU\n");
            test_mpu6050();
            break;
        case TEST_ADC:
            printf("[TEST] ADC Channels\n");
            test_adc();
            break;
        case TEST_UART:
            printf("[TEST] UART Communication\n");
            test_uart();
            break;
        case TEST_POWER:
            printf("[TEST] Power Monitor\n");
            test_power();
            break;
        case TEST_GAIT:
            printf("[TEST] Gait Generator\n");
            test_gait();
            break;
        case TEST_ARM:
            printf("[TEST] Arm Range-of-Motion\n");
            test_arm();
            break;
        default:
            printf("[INFO] No test selected. Set TEST_SELECT in main.c\n");
            break;
    }

    printf("\n[DONE] Test complete.\n");
}

#endif
