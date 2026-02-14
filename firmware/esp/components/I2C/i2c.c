#include "i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "I2C";

static bool i2c_initialized = false;

esp_err_t xI2cMasterInit(void) {
    // check if initialized already
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    // set config
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // 21
        .scl_io_num = I2C_MASTER_SCL_IO, // 22
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
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
    /* CREATING WRITE COMMAND BLOCK */

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // bitshift device address (7 bits) to the left and keep LSB as 0 for read bit
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    // indicate register to write to
    i2c_master_write_byte(cmd, reg_addr, true);

    // write data
    i2c_master_write_byte(cmd, data, true);

    i2c_master_stop(cmd);

    /* ENDED WRITE COMMAND BLOCK */

    // excecute the finalized command
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    // free memory from cmd link
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write to 0x%02X reg 0x%02X failed: %s",
                 dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t xI2cWriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_OK;

    // standard write
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // write multiple bytes at once
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write %d bytes to 0x%02X reg 0x%02X failed: %s",
                 (int)len, dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t xI2cReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    /* CREATING READ COMMAND BLOCK */

    // set register pointer
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // REPEATED START (to read data)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    /* ENDED READ COMMAND BLOCK */

    // excecute the finalized command
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    // free memory from cmd link
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read from 0x%02X reg 0x%02X failed: %s",
                 dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t xI2cReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_OK;

    // standard read
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // REPEATED START (to read data)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    // if multiple bytes then read them all with ACK
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    // read last byte with NACK (signals end of read to slave)
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read %d bytes from 0x%02X reg 0x%02X failed: %s",
                 (int)len, dev_addr, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}