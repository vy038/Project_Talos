#include "servo.h"
#include "i2c.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "PCA9685";

esp_err_t xPCA9685Init(i2c_port_t port, uint8_t addr, uint16_t pwm_freq_hz) {
    // sleep mode
    esp_err_t ret = xI2cWriteByte(addr, PCA9685_REG_MODE1, MODE1_SLEEP);
    if (ret != ESP_OK) return ret;

    // config cycles
    uint8_t prescale = (uint8_t)(round(25000000.0 / (4096.0 * pwm_freq_hz)) - 1);
    ESP_LOGI(TAG, "Setting PWM to %d Hz (prescale = %d)", pwm_freq_hz, prescale);

    ret = xI2cWriteByte(addr, PCA9685_REG_PRESCALE, prescale);
    if (ret != ESP_OK) return ret;

    // auto increment for easier writing to registers
    ret = xI2cWriteByte(addr, PCA9685_REG_MODE1, MODE1_AI | MODE1_RESTART);
    if (ret != ESP_OK) return ret;

    // wait for oscillator to stabilize after restart
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "PCA9685 at 0x%02X initialized successfully", addr);
    return ESP_OK;
}

esp_err_t xPCA9685SetPwm(i2c_port_t port, uint8_t addr, uint8_t channel, uint16_t pulse_us) {
    // parameter validation
    if (channel > 15) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // setting up pwm
    uint16_t pwm_on = 0;
    // period = 20000us (50Hz), 12 bit counter = 4096 ticks, pulse_us out of 4096 (~voltage)
    uint16_t pwm_off = (pulse_us * 4096) / 20000;

    // find start of array of registers for channel (every channel has 4 registers, starting from 0x06)
    uint8_t reg = PCA9685_REG_LED0_ON_L + (4 * channel);

    // create data to write
    uint8_t data[4] = {
        (uint8_t)(pwm_on),                  // low 8 bits of pwm on (0)
        (uint8_t)((pwm_on >> 8) & 0x0F),    // high 4 bits of pwm on (0)
        (uint8_t)(pwm_off),                 // low 8 bits of pwm off
        (uint8_t)((pwm_off >> 8) & 0x0F)    // high 4 bits of pwm off
    };

    // write to all 4 channel registers
    return xI2cWriteBytes(addr, reg, data, 4);
}

esp_err_t xPCA9685SetAngle(i2c_port_t port, uint8_t addr, uint8_t channel, uint8_t angle) {
    if (angle > 180) angle = 180;

    // generic mapping (replace with calibration for min/max later)
    // find space between min and max pulses, and map each angle to a pulse between
    uint16_t pulse_us = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180);

    return xPCA9685SetPwm(port, addr, channel, pulse_us);
}

esp_err_t xPCA9685SetPwmBurst(i2c_port_t port, uint8_t addr, uint8_t start_channel, uint8_t num_channels, uint16_t pulse_us[]) {
    // parameter validation
    if (start_channel + num_channels > 16) {
        ESP_LOGE(TAG, "Invalid channels from %d to %d", start_channel, start_channel + num_channels);
        return ESP_ERR_INVALID_ARG;
    }

    // setting up pwm
    uint16_t pwm_on = 0;

    // find start of array of registers for channels (every channel has 4 registers, starting from 0x06)
    uint8_t reg = PCA9685_REG_LED0_ON_L + (4 * start_channel);

    // allocate buffer
    uint8_t *data = malloc(num_channels * 4);
    if (!data) {
        ESP_LOGE(TAG, "Malloc failed");
        return ESP_ERR_NO_MEM;
    }

    // create data packets to write
    for (int i = 0; i < num_channels; i++) {
        // period = 20000us (50Hz), 12 bit counter = 4096 ticks, pulse_us out of 4096 (~voltage)
        uint16_t pwm_off = (pulse_us[i] * 4096) / 20000;

        data[i*4 + 0] = (uint8_t)(pwm_on);                  // low 8 bits of pwm on (0)
        data[i*4 + 1] = (uint8_t)((pwm_on >> 8) & 0x0F);    // high 4 bits of pwm on (0)
        data[i*4 + 2] = (uint8_t)(pwm_off);                 // low 8 bits of pwm off
        data[i*4 + 3] = (uint8_t)((pwm_off >> 8) & 0x0F);   // high 4 bits of pwm off
    }

    size_t num_bytes = num_channels * 4;
    esp_err_t ret = xI2cWriteBytes(addr, reg, data, num_bytes);
    free(data);

    return ret;
}

esp_err_t xPCA9685SetPwmMulti(i2c_port_t port, uint8_t addr, servo_command_t commands[], uint8_t num_commands) {
    // run pwm individually and set each servo according to array
    for (int i = 0; i < num_commands; i++) {
        esp_err_t ret = xPCA9685SetPwm(port, addr, commands[i].channel, commands[i].pulse_us);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

#define PCA9685_RETRIES 3

esp_err_t xPCA9685SetAngleWithRetry(i2c_port_t port, uint8_t addr, uint8_t channel,
                                     uint8_t angle, uint16_t pwm_freq_hz) {
    esp_err_t ret;
    for (int attempt = 0; attempt < PCA9685_RETRIES; attempt++) {
        ret = xPCA9685SetAngle(port, addr, channel, angle);
        if (ret == ESP_OK) return ESP_OK;
        ESP_LOGW(TAG, "I2C retry %d for 0x%02X ch%d", attempt + 1, addr, channel);
        if (ret == ESP_ERR_TIMEOUT) {
            // bus is stuck, recover before retrying
            xI2cBusRecovery();
            // board needs reinit after bus recovery
            xPCA9685Init(port, addr, pwm_freq_hz);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ret;
}