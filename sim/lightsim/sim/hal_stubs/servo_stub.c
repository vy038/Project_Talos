/**
 * @file servo_stub.c
 * @brief Virtual PCA9685 servo driver replacing components/servo/servo.c
 *
 * Captures servo angles into sim_state arrays for visualization.
 * Implements the same angle-to-pulse conversion as the real driver.
 */

#include "servo.h"
#include "i2c.h"
#include "esp_log.h"
#include "sim_state.h"
#include <math.h>

static const char *TAG = "PCA9685_SIM";

esp_err_t xPCA9685Init(i2c_port_t port, uint8_t addr, uint16_t pwm_freq_hz) {
    uint8_t prescale = (uint8_t)(round(25000000.0 / (4096.0 * pwm_freq_hz)) - 1);
    ESP_LOGI(TAG, "Virtual PCA9685 at 0x%02X initialized (%d Hz, prescale=%d)",
             addr, pwm_freq_hz, prescale);
    return ESP_OK;
}

esp_err_t xPCA9685SetPwm(i2c_port_t port, uint8_t addr, uint8_t channel, uint16_t pulse_us) {
    if (channel > 15) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    /* Convert pulse to approximate angle for state tracking */
    float angle = (float)(pulse_us - SERVO_MIN_PULSE_US) * 180.0f /
                  (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    int board = (addr == PCA9685_BODY_ADDR) ? 0 : 1;
    sim_set_servo(board, channel, angle, pulse_us);

    return ESP_OK;
}

esp_err_t xPCA9685SetAngle(i2c_port_t port, uint8_t addr, uint8_t channel, uint8_t angle) {
    if (angle > 180) angle = 180;
    uint16_t pulse_us = SERVO_MIN_PULSE_US +
                        (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180);
    return xPCA9685SetPwm(port, addr, channel, pulse_us);
}

esp_err_t xPCA9685SetPwmBurst(i2c_port_t port, uint8_t addr, uint8_t start_channel,
                               uint8_t num_channels, uint16_t pulse_us[]) {
    if (start_channel + num_channels > 16) {
        ESP_LOGE(TAG, "Invalid channels from %d to %d", start_channel, start_channel + num_channels);
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < num_channels; i++) {
        esp_err_t ret = xPCA9685SetPwm(port, addr, start_channel + i, pulse_us[i]);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

esp_err_t xPCA9685SetPwmMulti(i2c_port_t port, uint8_t addr, servo_command_t commands[],
                               uint8_t num_commands) {
    for (int i = 0; i < num_commands; i++) {
        esp_err_t ret = xPCA9685SetPwm(port, addr, commands[i].channel, commands[i].pulse_us);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}
