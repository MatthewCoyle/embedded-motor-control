cmake_minimum_required(VERSION 3.16)
project(EmbeddedMotorControl C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(motorcore STATIC
    src/motor_controller.c
    src/motor_hal.c
    src/motor_plant.c
    src/uart_protocol.c
    src/console.c
)
target_include_directories(motorcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_options(motorcore PRIVATE /W4)
else()
    target_compile_options(motorcore PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(motorcore PUBLIC m)
endif()
add_executable(motor_sim src/main.c)
target_link_libraries(motor_sim PRIVATE motorcore)
add_executable(motor_trace tools/trace_export.c)
target_link_libraries(motor_trace PRIVATE motorcore)
add_executable(motor_tests tests/test_motor_controller.c)
target_link_libraries(motor_tests PRIVATE motorcore)
enable_testing()
add_test(NAME motor_unit_tests COMMAND motor_tests)
add_test(NAME normal_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/normal_operation.txt)
add_test(NAME overtemp_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/overtemperature.txt)
add_test(NAME overcurrent_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/overcurrent.txt)
add_test(NAME stall_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/stall.txt)
add_test(NAME sensor_failure_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/sensor_failure.txt)
add_test(NAME estop_recovery_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/estop_recovery.txt)
add_test(NAME undervoltage_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/undervoltage.txt)

add_test(NAME motor_trace_export COMMAND motor_trace)
add_test(NAME overspeed_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/overspeed.txt)
add_test(NAME stale_sensor_script COMMAND motor_sim --script ${CMAKE_SOURCE_DIR}/examples/stale_sensor.txt)
