cmake_minimum_required(VERSION 3.16)
project(esp32_industrial_iot_telemetry_node VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wshadow -ffunction-sections -fdata-sections)
endif()

add_library(telemetry_core STATIC
    src/sensor_filter.c
    src/telemetry_fsm.c
)

target_include_directories(telemetry_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

enable_testing()

add_executable(test_telemetry_node tests/test_telemetry_node.c)
target_link_libraries(test_telemetry_node PRIVATE telemetry_core m)
add_test(NAME TelemetryNodeTestSuite COMMAND test_telemetry_node)
