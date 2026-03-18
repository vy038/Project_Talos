/**
 * @file i2c_stub.c
 * @brief Virtual I2C bus replacing components/I2C/i2c.c
 *
 * Logs all transactions and tracks state. Returns ESP_OK for all operations.
 */

#include "i2c.h"
#include "esp_log.h"
#include "sim_state.h"

static const char *TAG = "I2C_SIM";

esp_err_t xI2cMasterInit(void) {
    ESP_LOGI(TAG, "Virtual I2C bus initialized (SDA=21, SCL=22, 100kHz)");
    return ESP_OK;
}

esp_err_t xI2cBusRecovery(void) {
    ESP_LOGW(TAG, "Virtual I2C bus recovery (no-op in sim)");
    sim_log_i2c("RECOVERY", 0x00, 0x00, NULL, 0);
    return ESP_OK;
}

esp_err_t xI2cWriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    sim_log_i2c("W", dev_addr, reg_addr, &data, 1);
    return ESP_OK;
}

esp_err_t xI2cWriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    sim_log_i2c("W", dev_addr, reg_addr, data, len);
    return ESP_OK;
}

esp_err_t xI2cReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    if (data) *data = 0;
    sim_log_i2c("R", dev_addr, reg_addr, NULL, 1);
    return ESP_OK;
}

esp_err_t xI2cReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (data) memset(data, 0, len);
    sim_log_i2c("R", dev_addr, reg_addr, NULL, len);
    return ESP_OK;
}
