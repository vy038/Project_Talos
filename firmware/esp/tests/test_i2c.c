/**
 * @file test_i2c.c
 * @brief I2C bus scan - discovers all devices on the bus
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c.h"

#define TAG "TEST_I2C"

void test_i2c_scan(void) {
    printf("I2C Bus Scan (0x08 - 0x77)\n\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    int found = 0;

    for (int i = 0; i < 128; i += 16) {
        printf("%02X: ", i);
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
                continue;
            }

            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                printf("%02X ", addr);
                found++;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }

    printf("\nFound %d device(s) on I2C bus.\n", found);
}
