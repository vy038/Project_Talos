idf_component_register(
    SRCS "main.c"
         "../tests/test_servo.c"
         "../tests/test_i2c.c"
         "../tests/test_mpu.c"
         "../tests/test_adc.c"
         "../tests/test_uart.c"
         "../tests/test_power.c"
         "../tests/test_gait.c"
         "../tests/test_arm.c"
         "locomotion/gait_generator.c"
         "locomotion/balance_control.c"
         "arm/arm_control.c"
         "arm/ik_solver.c"
    INCLUDE_DIRS "." "../tests" "locomotion" "arm"
    REQUIRES I2C servo MPU6050 ADC power_monitor UART
)
