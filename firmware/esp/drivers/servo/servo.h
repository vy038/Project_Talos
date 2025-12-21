#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define PCA9685_ADDR_BODY   0x40
#define PCA9685_ADDR_ARM    0x41

// TODO: Initialize servos
void servo_init(void);

// TODO: Set servo angle (0-180 degrees)
void servo_set_angle(uint8_t board, uint8_t channel, uint8_t angle);

#endif
