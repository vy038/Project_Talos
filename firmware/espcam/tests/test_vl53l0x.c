/**
 * @file test_vl53l0x.c
 * @brief VL53L0X TOF sensor test (isolated, no camera interference)
 *
 * Standalone test that doesn't initialize camera or other hardware.
 * Tests only the VL53L0X sensor on provided I2C pins.
 *
 * Wire up:
 *   VL53L0X SDA → GPIO sda_pin
 *   VL53L0X SCL → GPIO scl_pin
 *   VL53L0X GND → GND
 *   VL53L0X VCC → 3.3V
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "vl53l0x.h"

#define TOF_I2C_PORT    I2C_NUM_0
#define TOF_ADDRESS     0x29

void test_vl53l0x(uint8_t scl, uint8_t sda) {
    printf("\n========================================\n");
    printf("  VL53L0X Standalone Test\n");
    printf("  I2C Port 0: SDA=GPIO%d, SCL=GPIO%d\n", sda, scl);
    printf("  Address: 0x%02X\n", TOF_ADDRESS);
    printf("  Note: Using internal ESP32 I2C pull-ups\n");
    printf("========================================\n\n");

    // External 4.7k pull-ups on SDA/SCL — disable internal ones
    gpio_set_pull_mode(scl, GPIO_FLOATING);
    gpio_set_pull_mode(sda, GPIO_FLOATING);

    // Configure: (port, scl, sda, xshut, address, io_2v8)
    printf("Initializing VL53L0X...\n");
    vl53l0x_t *dev = vl53l0x_config(TOF_I2C_PORT, scl, sda, -1, TOF_ADDRESS, 1);
    if (!dev) {
        printf("ERROR: vl53l0x_config failed\n");
        return;
    }

    // Set generous timeout before init — SPAD calibration needs more than the 100ms default
    vl53l0x_setTimeout(dev, 500);

    const char *err = vl53l0x_init(dev);
    if (err) {
        printf("ERROR: vl53l0x_init failed: %s\n", err);
        vl53l0x_end(dev);
        return;
    }

    printf("SUCCESS: Sensor initialized\n");
    vl53l0x_setTimeout(dev, 500);
    vl53l0x_startContinuous(dev, 0);

    printf("\nReading distances (point your hand at sensor):\n");
    printf("---\n");

    for (int i = 0; i < 100; i++) {
        uint16_t distance = vl53l0x_readRangeContinuousMillimeters(dev);

        if (vl53l0x_timeoutOccurred(dev)) {
            printf("[%3d] TIMEOUT\n", i);
        } else {
            printf("[%3d] Distance: %5d mm\n", i, distance);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    printf("---\n");
    vl53l0x_end(dev);
    printf("Test complete.\n");
}