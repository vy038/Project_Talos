idf_component_register(
    SRCS "main.c"
         "vision/vision_processor.c"
    INCLUDE_DIRS "." "vision"
    REQUIRES camera UART VL53L0X
)
