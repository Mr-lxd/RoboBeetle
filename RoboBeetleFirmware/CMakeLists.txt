cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RoboBeetleFirmware)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(ROBOBEETLE_CPG_TARGET_BENCHMARK
       "Enable the STM32F407 CPG DWT benchmark"
       OFF)

option(ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS
       "Enable RAM-only STM32F407 motion timing diagnostics"
       OFF)

option(ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY
       "Suppress optional telemetry in a diagnostics image"
       OFF)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/App/app_main.c
    Core/Src/rb_protocol_v2.c
    Core/Communication/ring_buffer.c
    Core/Communication/uart_tx_queue.c
    Core/Communication/uart_transport_stm32.c
    Core/Communication/jy901s_transport_stm32.c
    Core/Communication/depth_transport_stm32.c
    Core/Communication/depth_telemetry.c
    Core/Communication/imu_telemetry_policy.c
    Core/Communication/telemetry_scheduler.c
    Core/Communication/protocol_dispatcher.c
    Core/Motion/cpg_core.c
    Core/Motion/cpg_gait_generator.c
    Core/Motion/simple_gait_generator.c
    Core/Motion/motion_manager.c
    Core/Sensors/leak_sensor.c
    Core/Sensors/leak_sensor_stm32.c
    Core/Sensors/leak_telemetry_policy.c
    Core/Sensors/jy901s_parser.c
    Core/Sensors/depth_parser.c
    Core/Sensors/jy901s_telemetry.c
    Core/Servo/servo_calibration.c
    Core/Servo/servo_descriptor.c
    Core/Servo/servo_service.c
    Core/Servo/servo_driver_stm32.c
    Core/Servo/servo_pwm_stop_policy.c
    Core/Safety/safety_supervisor.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/App
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Communication
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Servo
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Motion
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

if(ROBOBEETLE_CPG_TARGET_BENCHMARK)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/App/cpg_target_benchmark.c
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ROBOBEETLE_CPG_TARGET_BENCHMARK=1
    )
endif()

if(ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Diagnostics/motion_timing_diagnostics.c
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ROBOBEETLE_MOTION_TIMING_DIAGNOSTICS=1
    )

    if(ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
            ROBOBEETLE_MOTION_TIMING_REDUCED_TELEMETRY=1
        )
    endif()
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)
