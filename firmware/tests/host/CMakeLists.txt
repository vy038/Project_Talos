cmake_minimum_required(VERSION 3.16)
project(talos_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -g")

# Include paths: stubs first (so esp_stubs.h shadows esp_err.h etc.)
set(STUBS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(ESP_MAIN ${CMAKE_SOURCE_DIR}/../../esp/main)
set(ESPCAM_MAIN ${CMAKE_SOURCE_DIR}/../../espcam/main)

# --- UART protocol parser test ---
add_executable(test_uart_protocol test_uart_protocol.c)
target_include_directories(test_uart_protocol PRIVATE
    ${STUBS_DIR}
    ${ESP_MAIN}/state_machine
)
# bStateMachineParseUART is compiled from the .c via include in the test
# We only need the header for types/constants

# --- IK solver test ---
add_executable(test_ik_solver test_ik_solver.c)
target_include_directories(test_ik_solver PRIVATE
    ${STUBS_DIR}
    ${ESP_MAIN}/arm
)
target_link_libraries(test_ik_solver m)

# --- Vision math test ---
add_executable(test_vision_math test_vision_math.c)
target_include_directories(test_vision_math PRIVATE
    ${STUBS_DIR}
    ${ESPCAM_MAIN}/vision
)
target_link_libraries(test_vision_math m)

# CTest integration
enable_testing()
add_test(NAME uart_protocol COMMAND test_uart_protocol)
add_test(NAME ik_solver COMMAND test_ik_solver)
add_test(NAME vision_math COMMAND test_vision_math)
