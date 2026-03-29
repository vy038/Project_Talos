idf_component_register(
    SRCS "main.c"
         "vision/vision_processor.c"
         "uart_protocol/uart_protocol.c"
         "tasks/vision_task.c"
         "../tests/test_red_ball.c"
    INCLUDE_DIRS "." "vision" "uart_protocol" "tasks" "../tests"
    REQUIRES camera UART VL53L0X I2C
)
