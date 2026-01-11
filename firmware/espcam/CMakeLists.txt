cmake_minimum_required(VERSION 3.16)

# Tell ESP-IDF to look for components in drivers/ directory
set(EXTRA_COMPONENT_DIRS "drivers")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espcam)
