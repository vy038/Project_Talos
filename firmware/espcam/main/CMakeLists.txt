idf_component_register(
    SRCS "main.c"
         "vision/vision_processor.c"
         "uart_protocol/uart_protocol.c"
         "tasks/vision_task.c"
         "../tests/test_red_ball.c"
         "../tests/test_vl53l0x.c"
         "../tests/test_i2c_scan.c"
         "../tests/test_gpio.c"
    INCLUDE_DIRS "." "vision" "uart_protocol" "tasks" "../tests"
    REQUIRES camera UART VL53L0X I2C
)
