idf_component_register(
    SRCS "main.c"
         "../tests/test_servo.c"
         "../tests/test_i2c.c"
         "../tests/test_mpu.c"
         "../tests/test_adc.c"
         "../tests/test_uart.c"
         "../tests/test_power.c"
         "../tests/test_gait.c"
         "locomotion/gait_generator.c"
         "locomotion/balance_control.c"
    INCLUDE_DIRS "." "../tests" "locomotion"
    REQUIRES I2C servo MPU6050 ADC power_monitor UART
)
