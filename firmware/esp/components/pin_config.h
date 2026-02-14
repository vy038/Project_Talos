/**
 * @file pin_config.h
 * @brief Centralized pin assignments for Project Talos - ESP32-WROOM (Motor Control)
 *
 * All GPIO assignments in one place. If you move a wire, change it here and
 * nowhere else. Organized by peripheral/function.
 *
 * NOTE: Update this file if you change any hardware connections.
 *       Do NOT hardcode pin numbers anywhere else in the codebase.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/* ========================================================================== */
/*  I2C Bus Configuration                                                     */
/* ========================================================================== */

/** @brief Primary I2C bus - PCA9685 servo drivers, MPU6050, VL53L0X */
#define PIN_I2C0_SDA            GPIO_NUM_21
#define PIN_I2C0_SCL            GPIO_NUM_22
#define I2C0_FREQ_HZ            400000      /* 400kHz fast mode */
#define I2C0_PORT               I2C_NUM_0

/**
 * @brief Secondary I2C bus (optional)
 * Use if you need to isolate sensors from servo drivers to avoid
 * bus contention or if a PCA9685 is pulling the bus down.
 */
#define PIN_I2C1_SDA            GPIO_NUM_18
#define PIN_I2C1_SCL            GPIO_NUM_19
#define I2C1_FREQ_HZ            400000
#define I2C1_PORT               I2C_NUM_1

/* ========================================================================== */
/*  I2C Device Addresses                                                      */
/* ========================================================================== */

/** PCA9685 servo driver boards (7-bit addresses) */
#define PCA9685_ADDR_LEGS_0     0x40    /* Legs 1-2 (channels 0-15)  */
#define PCA9685_ADDR_LEGS_1     0x41    /* Legs 3-4 (channels 0-15)  */
#define PCA9685_ADDR_LEGS_2     0x42    /* Legs 5-6 (channels 0-11)  */
#define PCA9685_ADDR_ARM        0x43    /* Arm servos (channels 0-5) */

/** Sensors */
#define MPU6050_ADDR            0x68    /* AD0 pin low */
#define VL53L0X_ADDR            0x29    /* Default address */

/* ========================================================================== */
/*  UART - Inter-ESP32 Communication (WROOM <-> S3)                           */
/* ========================================================================== */

#define PIN_UART_TX             GPIO_NUM_17
#define PIN_UART_RX             GPIO_NUM_16
#define UART_INTER_ESP_PORT     UART_NUM_2
#define UART_INTER_ESP_BAUD     115200

/* ========================================================================== */
/*  ADC - Analog Sensors                                                      */
/* ========================================================================== */

/** Battery voltage monitoring via voltage divider */
#define PIN_VBAT_ADC            GPIO_NUM_34     /* ADC1_CH6, input only */
#define VBAT_ADC_CHANNEL        ADC1_CHANNEL_6
#define VBAT_DIVIDER_RATIO      3.0f            /* Adjust to your divider R1/R2 */

/** ACS712 current sensor analog output */
#define PIN_CURRENT_ADC         GPIO_NUM_35     /* ADC1_CH7, input only */
#define CURRENT_ADC_CHANNEL     ADC1_CHANNEL_7
#define ACS712_SENSITIVITY      0.185f          /* V/A for 5A module, 0.1 for 20A */
#define ACS712_ZERO_CURRENT_V   2.5f            /* Output at 0A (Vcc/2) */

/* ========================================================================== */
/*  Status LEDs                                                               */
/* ========================================================================== */

#define PIN_LED_STATUS          GPIO_NUM_2      /* Onboard LED */
#define PIN_LED_ERROR           GPIO_NUM_4      /* External error indicator */

/* ========================================================================== */
/*  Power Control                                                             */
/* ========================================================================== */

/** Enable pin for servo power rail (via MOSFET gate) */
#define PIN_SERVO_POWER_EN      GPIO_NUM_25

/** Enable pin for sensor power rail */
#define PIN_SENSOR_POWER_EN     GPIO_NUM_26

/* ========================================================================== */
/*  Spare / Expansion                                                         */
/* ========================================================================== */

/**
 * Available GPIOs on ESP32-WROOM-32:
 *   GPIO 0  - Boot button (use with caution)
 *   GPIO 5  - Available (has pullup at boot)
 *   GPIO 12 - Available (must be low at boot for flash voltage)
 *   GPIO 13 - Available
 *   GPIO 14 - Available
 *   GPIO 15 - Available (pullup at boot, debug output)
 *   GPIO 23 - Available
 *   GPIO 27 - Available
 *   GPIO 32 - Available (ADC1_CH4)
 *   GPIO 33 - Available (ADC1_CH5)
 *
 * DO NOT USE:
 *   GPIO 6-11  - Connected to SPI flash
 *   GPIO 36,39 - Input only, no pullup (usable for ADC if needed)
 */

/* ========================================================================== */
/*  Servo Configuration Constants                                             */
/* ========================================================================== */

/** PCA9685 PWM parameters */
#define SERVO_PWM_FREQ_HZ       50      /* Standard servo frequency */
#define SERVO_PULSE_MIN_US      500     /* 0 degrees (tune per servo) */
#define SERVO_PULSE_MAX_US      2500    /* 180 degrees (tune per servo) */
#define SERVO_ANGLE_MIN         0
#define SERVO_ANGLE_MAX         180

/* ========================================================================== */
/*  Hexapod Leg Servo Channel Mapping                                         */
/*                                                                            */
/*  Each leg has 3 servos: coxa (hip), femur (thigh), tibia (shin)            */
/*  Format: LEG[n]_[JOINT] = {PCA9685_BOARD, CHANNEL}                        */
/* ========================================================================== */

/* Leg 1 - Front Right */
#define LEG1_COXA_BOARD         PCA9685_ADDR_LEGS_0
#define LEG1_COXA_CH            0
#define LEG1_FEMUR_BOARD        PCA9685_ADDR_LEGS_0
#define LEG1_FEMUR_CH           1
#define LEG1_TIBIA_BOARD        PCA9685_ADDR_LEGS_0
#define LEG1_TIBIA_CH           2

/* Leg 2 - Front Left */
#define LEG2_COXA_BOARD         PCA9685_ADDR_LEGS_0
#define LEG2_COXA_CH            4
#define LEG2_FEMUR_BOARD        PCA9685_ADDR_LEGS_0
#define LEG2_FEMUR_CH           5
#define LEG2_TIBIA_BOARD        PCA9685_ADDR_LEGS_0
#define LEG2_TIBIA_CH           6

/* Leg 3 - Mid Right */
#define LEG3_COXA_BOARD         PCA9685_ADDR_LEGS_1
#define LEG3_COXA_CH            0
#define LEG3_FEMUR_BOARD        PCA9685_ADDR_LEGS_1
#define LEG3_FEMUR_CH           1
#define LEG3_TIBIA_BOARD        PCA9685_ADDR_LEGS_1
#define LEG3_TIBIA_CH           2

/* Leg 4 - Mid Left */
#define LEG4_COXA_BOARD         PCA9685_ADDR_LEGS_1
#define LEG4_COXA_CH            4
#define LEG4_FEMUR_BOARD        PCA9685_ADDR_LEGS_1
#define LEG4_FEMUR_CH           5
#define LEG4_TIBIA_BOARD        PCA9685_ADDR_LEGS_1
#define LEG4_TIBIA_CH           6

/* Leg 5 - Rear Right */
#define LEG5_COXA_BOARD         PCA9685_ADDR_LEGS_2
#define LEG5_COXA_CH            0
#define LEG5_FEMUR_BOARD        PCA9685_ADDR_LEGS_2
#define LEG5_FEMUR_CH           1
#define LEG5_TIBIA_BOARD        PCA9685_ADDR_LEGS_2
#define LEG5_TIBIA_CH           2

/* Leg 6 - Rear Left */
#define LEG6_COXA_BOARD         PCA9685_ADDR_LEGS_2
#define LEG6_COXA_CH            4
#define LEG6_FEMUR_BOARD        PCA9685_ADDR_LEGS_2
#define LEG6_FEMUR_CH           5
#define LEG6_TIBIA_BOARD        PCA9685_ADDR_LEGS_2
#define LEG6_TIBIA_CH           6

/* ========================================================================== */
/*  Arm Servo Channel Mapping                                                 */
/*                                                                            */
/*  6-DOF: base, shoulder, elbow, wrist_pitch, wrist_roll, gripper            */
/* ========================================================================== */

#define ARM_BASE_BOARD          PCA9685_ADDR_ARM
#define ARM_BASE_CH             0
#define ARM_SHOULDER_BOARD      PCA9685_ADDR_ARM
#define ARM_SHOULDER_CH         1
#define ARM_ELBOW_BOARD         PCA9685_ADDR_ARM
#define ARM_ELBOW_CH            2
#define ARM_WRIST_PITCH_BOARD   PCA9685_ADDR_ARM
#define ARM_WRIST_PITCH_CH      3
#define ARM_WRIST_ROLL_BOARD    PCA9685_ADDR_ARM
#define ARM_WRIST_ROLL_CH       4
#define ARM_GRIPPER_BOARD       PCA9685_ADDR_ARM
#define ARM_GRIPPER_CH          5

#endif /* PIN_CONFIG_H */