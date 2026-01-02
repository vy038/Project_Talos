#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include "driver/i2c.h"
#include "esp_err.h"

// I2C addresses
#define PCA9685_BODY_ADDR       0x40
#define PCA9685_ARM_ADDR        0x41

// segister addresses
#define PCA9685_REG_MODE1       0x00
#define PCA9685_REG_PRESCALE    0xFE
#define PCA9685_REG_LED0_ON_L   0x06

// MODE1 register bits
#define MODE1_SLEEP             0x10
#define MODE1_AI                0x20
#define MODE1_RESTART           0x80

// standard servo pulse range (sg90 standard)
#define SERVO_MIN_PULSE_US      1000
#define SERVO_MAX_PULSE_US      2000

// servo command structure
typedef struct {
    uint8_t channel;
    uint16_t pulse_us;
} servo_command_t;

/**
 * @brief Initialize PCA9685
 *
 * Sets PWM frequency and enables auto-increment
 * (Make sure I2C init() is called first!)
 * 
 * @param port I2C port
 * @param addr Address of board to initialize
 * @param pwm_freq_hz PWM frequency (typically 50Hz for servos)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xPCA9685Init(i2c_port_t port, uint8_t addr, uint16_t pwm_freq_hz);

/**
 * @brief Set single servo pulse width
 *
 * Use for testing/calibration; for walking use burst writes
 * 
 * @param port I2C port
 * @param addr Address of board
 * @param channel Servo channel (0-15)
 * @param pulse_us Pulse width in microseconds
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xPCA9685SetPwm(i2c_port_t port, uint8_t addr, uint8_t channel, uint16_t pulse_us);

/**
 * @brief Set servo angle
 *
 * Converts angle to pulse width and sets servo position
 * 
 * @param port I2C port
 * @param addr Address of board
 * @param channel Servo channel (0-15)
 * @param angle Angle in degrees (0-180)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xPCA9685SetAngle(i2c_port_t port, uint8_t addr, uint8_t channel, uint8_t angle);

/**
 * @brief Set multiple consecutive servos
 *
 * Updates consecutive channels in one I2C transaction (~10x faster)
 * 
 * @param port I2C port
 * @param addr Address of board
 * @param start_channel First channel to update
 * @param num_channels Number of consecutive channels
 * @param pulse_us Array of pulse widths in microseconds
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xPCA9685SetPwmBurst(i2c_port_t port, uint8_t addr, uint8_t start_channel, 
                                 uint8_t num_channels, uint16_t pulse_us[]);

/**
 * @brief Set multiple non-consecutive servos
 *
 * Updates scattered channels when legs stop individually
 * 
 * @param port I2C port
 * @param addr Address of board
 * @param commands Array of servo commands
 * @param num_commands Number of servos to update
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t xPCA9685SetPwmMulti(i2c_port_t port, uint8_t addr, servo_command_t commands[], 
                                 uint8_t num_commands);


#endif