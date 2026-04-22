/**
 * @file test_i2c_scan.c
 * @brief I2C address scanner
 *
 * Scans all I2C addresses 0x00-0x7F and reports which devices respond.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "i2c_scan";

void test_i2c_scan(uint8_t scl, uint8_t sda) {
    printf("\n========================================\n");
    printf("  I2C Address Scanner\n");
    printf("  SDA=GPIO%d, SCL=GPIO%d\n", sda, scl);
    printf("========================================\n\n");

    // Enable GPIO pull-ups
    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);

    // Configure I2C (match working ESP project)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = false,  // Disable internal; use external 4.7k resistors
        .scl_pullup_en = false,
        .master.clk_speed = 100000,  // Standard I2C speed (was 10kHz, that was the problem!)
    };

    printf("Installing I2C driver on port 0...\n");
    esp_err_t err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err && err != ESP_ERR_INVALID_STATE) {
        printf("ERROR: i2c_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    }
    if (err == ESP_ERR_INVALID_STATE) {
        printf("WARNING: I2C port already installed, deleting first...\n");
        i2c_driver_delete(I2C_NUM_0);
        err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        if (err) {
            printf("ERROR: i2c_driver_install (retry) failed: %s\n", esp_err_to_name(err));
            return;
        }
    }

    printf("Configuring I2C parameters (SDA=%d, SCL=%d, speed=10kHz)...\n", sda, scl);
    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err) {
        printf("ERROR: i2c_param_config failed: %s\n", esp_err_to_name(err));
        i2c_driver_delete(I2C_NUM_0);
        return;
    }

    printf("Scanning I2C addresses 0x00-0x7F (full range):\n");
    printf("---\n");

    int found = 0;
    for (uint8_t addr = 0x00; addr < 0x80; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));  // Match vision_processor
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("  Found device at 0x%02X\n", addr);
            found++;
        }
    }

    printf("---\n");
    i2c_driver_delete(I2C_NUM_0);

    printf("---\n");
    printf("Total devices found: %d\n", found);

    if (found == 0) {
        printf("ERROR: No I2C devices found. Check wiring and pull-ups.\n");
    }
}
