idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES camera UART VL53L0X
)
