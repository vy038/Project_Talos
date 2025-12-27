# Project Talos - Top Level CMakeLists.txt
# ESP-IDF v5.2 compatible

cmake_minimum_required(VERSION 3.16)

# Set project name
set(PROJECT_NAME "Project_Talos")

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define project
project(${PROJECT_NAME})

idf_component_register(SRCS "main.c"
                            "drivers/I2C/i2c.c"
                            "drivers/servo/servo.c"
                            "drivers/mpu6050/mpu6050.c"
                            "drivers/ADC/adc_helpers.c"
                            "drivers/power_monitor/power_monitor.c"
                            "drivers/UART/uart.c"
                            "tasks/arm/grab/grab_task.c"
                            "tasks/arm/position/position_task.c"
                            "tasks/locomotion/mpu/mpu_task.c"
                            "tasks/locomotion/neutral/neutral_task.c"
                            "tasks/locomotion/step/step_task.c"
                            "tasks/locomotion/turn/turn_task.c"
                            "tasks/power/power_task.c"
                            "tasks/uart/uart_task.c"
                            "test/test_i2c.c"
                    INCLUDE_DIRS "." "drivers" "tasks" "test")