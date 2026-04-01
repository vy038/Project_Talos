cmake_minimum_required(VERSION 3.16)
project(lightsim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Path to the Project Talos firmware (sim lives at Project_Talos/sim/lightsim/)
set(TALOS_FW "${CMAKE_SOURCE_DIR}/../../firmware/esp")

# ============================================================================
# Source files
# ============================================================================

# HAL stubs (replace real hardware drivers)
set(STUB_SOURCES
    sim/hal_stubs/i2c_stub.c
    sim/hal_stubs/servo_stub.c
    sim/hal_stubs/mpu6050_stub.c
    sim/hal_stubs/adc_stub.c
    sim/hal_stubs/uart_stub.c
    sim/hal_stubs/power_monitor_stub.c
    sim/hal_stubs/freertos_sim.c
    sim/hal_stubs/vl53l0x_stub.c
)

# Simulation engine
set(ENGINE_SOURCES
    sim/engine/sim_state.c
)

# Unmodified firmware logic files (compiled from Project Talos)
set(FW_LOGIC_SOURCES
    ${TALOS_FW}/main/main.c
    ${TALOS_FW}/main/locomotion/gait_generator.c
    ${TALOS_FW}/main/locomotion/balance_control.c
    ${TALOS_FW}/main/arm/arm_control.c
    ${TALOS_FW}/main/arm/ik_solver.c
    ${TALOS_FW}/main/state_machine/state_machine.c
    ${TALOS_FW}/main/power/power_management.c
    ${TALOS_FW}/main/uart_protocol/uart_protocol.c
    ${TALOS_FW}/main/tasks/state_machine_task.c
    ${TALOS_FW}/main/tasks/gait_task.c
    ${TALOS_FW}/main/tasks/balance_task.c
    ${TALOS_FW}/main/tasks/arm_ctrl_task.c
    ${TALOS_FW}/main/tasks/power_mon_task.c
    ${TALOS_FW}/main/tasks/uart_cam_task.c
)

# Firmware test files (compiled from Project Talos)
set(FW_TEST_SOURCES
    ${TALOS_FW}/tests/test_servo.c
    ${TALOS_FW}/tests/test_i2c.c
    ${TALOS_FW}/tests/test_mpu.c
    ${TALOS_FW}/tests/test_adc.c
    ${TALOS_FW}/tests/test_uart.c
    ${TALOS_FW}/tests/test_power.c
    ${TALOS_FW}/tests/test_gait.c
    ${TALOS_FW}/tests/test_arm.c
)

# Simulator entry point
set(SIM_SOURCES
    sim/main_sim.c
)

# ============================================================================
# Build target
# ============================================================================

add_executable(talos_sim
    ${SIM_SOURCES}
    ${STUB_SOURCES}
    ${ENGINE_SOURCES}
    ${FW_LOGIC_SOURCES}
    ${FW_TEST_SOURCES}
)

# ============================================================================
# Include paths (ORDER MATTERS - shims must come FIRST)
# ============================================================================

target_include_directories(talos_sim PRIVATE
    # 1. ESP-IDF shim headers (must override real ESP-IDF includes)
    ${CMAKE_SOURCE_DIR}/sim/hal_stubs/include

    # 2. Master shim header
    ${CMAKE_SOURCE_DIR}/sim/hal_stubs

    # 3. Simulation engine headers
    ${CMAKE_SOURCE_DIR}/sim/engine

    # 4. Firmware component headers (i2c.h, servo.h, mpu6050.h, etc.)
    ${TALOS_FW}/components/I2C
    ${TALOS_FW}/components/servo
    ${TALOS_FW}/components/MPU6050
    ${TALOS_FW}/components/ADC
    ${TALOS_FW}/components/UART
    ${TALOS_FW}/components/power_monitor
    ${CMAKE_SOURCE_DIR}/../../firmware/espcam/components/VL53L0X

    # 5. Firmware application headers
    ${TALOS_FW}/main
    ${TALOS_FW}/main/locomotion
    ${TALOS_FW}/main/arm
    ${TALOS_FW}/main/state_machine
    ${TALOS_FW}/main/power
    ${TALOS_FW}/main/tasks
    ${TALOS_FW}/main/uart_protocol
    ${TALOS_FW}/tests
)

# ============================================================================
# Compiler flags
# ============================================================================

target_compile_options(talos_sim PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-sign-compare
    -Wno-missing-field-initializers
    -g
    -O2
)

# Link math and pthread libraries
target_link_libraries(talos_sim PRIVATE m pthread)
