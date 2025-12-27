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
                            "drivers/http_client.c"
                            "drivers/spi.c"
                            "drivers/mpu6050.c"
                            "drivers/adc_helpers.c"
                            "drivers/gpio_helpers.c"
                            "drivers/wifi_helpers.c"
                            "tasks/mpu_task.c"
                            "tasks/mic_task.c"
                            "tasks/ls_task.c"
                            "tasks/spi_task.c"
                            "tasks/http_client_task.c"
                    INCLUDE_DIRS "." "drivers" "tasks")