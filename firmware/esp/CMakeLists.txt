# Project Talos - Top Level CMakeLists.txt
# ESP-IDF v5.2 compatible

cmake_minimum_required(VERSION 3.16)

# Set project name
set(PROJECT_NAME "Project_Talos")

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define project
project(${PROJECT_NAME})
