#include "i2c.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "I2C";

static bool i2c_initialized = false;

/**
 * @brief Bit-bang 9 clock pulses on SCL to release a stuck SDA line
 *
 * If a slave held SDA low during a previous transaction (power glitch, noise),
 * the bus stays locked. Clocking SCL 9 times per the I2C spec lets the slave
 * finish its byte and release SDA. Pins must NOT be owned by the I2C driver.
 */
static void i2c_bus_bitbang_recovery(void) {
    gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(I2C_MASTER_SDA_IO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(I2C_MASTER_SCL_IO, GPIO_PULLUP_ONLY);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(5);
        if (gpio_get_level(I2C_MASTER_SDA_IO)) break;
    }

    // send STOP condition (SDA low -> high while SCL high)
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    esp_rom_delay_us(5);
}

/**
 * @brief Runtime I2C bus recovery. Tears down the driver, bit-bangs recovery,
 *        then reinitializes. Call when a transaction returns ESP_ERR_TIMEOUT.
 */
esp_err_t xI2cBusRecovery(void) {
    ESP_LOGW(TAG, "Runtime I2C bus recovery...");

    // release pins from I2C driver
    i2c_driver_delete(I2C_MASTER_NUM);
    i2c_initialized = false;

    // bit-bang recovery while pins are free
    i2c_bus_bitbang_recovery();

    ESP_LOGI(TAG, "Bus recovery done, SDA=%d. Reinitializing...",
             gpio_get_level(I2C_MASTER_SDA_IO));

    // reinit the driver
    return xI2cMasterInit();
}

esp_err_t xI2cMasterInit(void) {
    // check if initialized already
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    // recover bus in case a slave is holding SDA low from a previous crash
    i2c_bus_bitbang_recovery();

    // set config
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // 21
        .scl_io_num = I2C_MASTER_SCL_IO, // 22
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // using external pullups
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    // apply config parameters + check
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    // install driver + check
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C initialized on port %d (SDA:%d SCL:%d @ %dHz)",
             I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

esp_err_t xI2cWriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write to 0x%02X reg 0x%02X failed: %s",
                 dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t xI2cWriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write %d bytes to 0x%02X reg 0x%02X failed: %s",
                 (int)len, dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t xI2cReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // repeated start to switch to read mode
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read from 0x%02X reg 0x%02X failed: %s",
                 dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t xI2cReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // repeated start to switch to read mode
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read %d bytes from 0x%02X reg 0x%02X failed: %s",
                 (int)len, dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}