cmake_minimum_required(VERSION 3.10)
project(embedded_driver C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files for the driver library
set(DRIVER_SOURCES
    src/device_registers.c
    src/motor_controller.c
    src/sensor_array.c
    src/shared_mem.c
    src/interrupt_handler.c
)

# Create static library
add_library(driver_lib STATIC ${DRIVER_SOURCES})

# Link pthread
find_package(Threads REQUIRED)
target_link_libraries(driver_lib Threads::Threads)

# On Linux, link rt for shared memory
if(UNIX AND NOT APPLE)
    target_link_libraries(driver_lib rt)
endif()

# Main executable
add_executable(motor_driver src/main.c)
target_link_libraries(motor_driver driver_lib)

# Enable testing
enable_testing()

# Test executable
add_executable(test_driver tests/test_driver.c)
target_link_libraries(test_driver driver_lib)

# Add tests - each test function is a separate CTest test
# Register tests
add_test(NAME test_reg_init COMMAND test_driver test_reg_init)
add_test(NAME test_reg_read_write COMMAND test_driver test_reg_read_write)
add_test(NAME test_reg_set_clear_bits COMMAND test_driver test_reg_set_clear_bits)
add_test(NAME test_reg_invalid_offset COMMAND test_driver test_reg_invalid_offset)

add_test(NAME test_motor_init COMMAND test_driver test_motor_init)
add_test(NAME test_motor_start COMMAND test_driver test_motor_start)
add_test(NAME test_motor_stop COMMAND test_driver test_motor_stop)
add_test(NAME test_motor_brake COMMAND test_driver test_motor_brake)
add_test(NAME test_motor_speed_ramp COMMAND test_driver test_motor_speed_ramp)
add_test(NAME test_motor_direction COMMAND test_driver test_motor_direction)
add_test(NAME test_motor_position_update COMMAND test_driver test_motor_position_update)
add_test(NAME test_motor_fault_stall COMMAND test_driver test_motor_fault_stall)
add_test(NAME test_motor_fault_overheat COMMAND test_driver test_motor_fault_overheat)
add_test(NAME test_motor_fault_recovery COMMAND test_driver test_motor_fault_recovery)
add_test(NAME test_motor_reset COMMAND test_driver test_motor_reset)
add_test(NAME test_motor_max_speed COMMAND test_driver test_motor_max_speed)

add_test(NAME test_sensor_init COMMAND test_driver test_sensor_init)
add_test(NAME test_sensor_enable_disable COMMAND test_driver test_sensor_enable_disable)
add_test(NAME test_sensor_trigger COMMAND test_driver test_sensor_trigger)
add_test(NAME test_sensor_read COMMAND test_driver test_sensor_read)
add_test(NAME test_sensor_read_all COMMAND test_driver test_sensor_read_all)
add_test(NAME test_sensor_continuous_mode COMMAND test_driver test_sensor_continuous_mode)
add_test(NAME test_sensor_buffer_push_pop COMMAND test_driver test_sensor_buffer_push_pop)
add_test(NAME test_sensor_buffer_overflow COMMAND test_driver test_sensor_buffer_overflow)
add_test(NAME test_sensor_buffer_clear COMMAND test_driver test_sensor_buffer_clear)
add_test(NAME test_sensor_value_clamping COMMAND test_driver test_sensor_value_clamping)

add_test(NAME test_irq_init COMMAND test_driver test_irq_init)
add_test(NAME test_irq_enable_disable COMMAND test_driver test_irq_enable_disable)
add_test(NAME test_irq_trigger COMMAND test_driver test_irq_trigger)
add_test(NAME test_irq_handler_callback COMMAND test_driver test_irq_handler_callback)
add_test(NAME test_irq_pending_mask COMMAND test_driver test_irq_pending_mask)
add_test(NAME test_irq_clear COMMAND test_driver test_irq_clear)

add_test(NAME test_shm_create_destroy COMMAND test_driver test_shm_create_destroy)
add_test(NAME test_shm_status_update COMMAND test_driver test_shm_status_update)
add_test(NAME test_shm_shutdown COMMAND test_driver test_shm_shutdown)

add_test(NAME test_integration_motor_sensor COMMAND test_driver test_integration_motor_sensor)
add_test(NAME test_integration_fault_irq COMMAND test_driver test_integration_fault_irq)
