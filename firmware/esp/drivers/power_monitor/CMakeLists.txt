idf_component_register(
    SRCS "power_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES driver ADC I2C
)
