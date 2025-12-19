#include "servo_control.h"
#include "driver/i2c.h"
#include <stdio.h>

void servo_init(void) {
    printf("TODO: Initialize servos\n");
    // TODO: Configure I2C on GPIO 21/22
    // TODO: Initialize PCA9685 boards
    // TODO: Set PWM to 50Hz
}

void servo_set_angle(uint8_t board, uint8_t channel, uint8_t angle) {
    printf("TODO: Move servo[%d][%d] to %d°\n", board, channel, angle);
    // TODO: Convert angle to PWM pulse width
    // TODO: Write to PCA9685 via I2C
}
