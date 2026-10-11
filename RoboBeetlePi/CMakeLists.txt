cmake_minimum_required(VERSION 3.19)

project(RoboBeetlePiProtocol VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(rbp2_protocol STATIC
    protocol/src/crc16.cpp
    protocol/src/cobs.cpp
    protocol/src/codec.cpp
    protocol/src/stream_decoder.cpp
)
target_include_directories(rbp2_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
)
target_compile_features(rbp2_protocol PUBLIC cxx_std_17)

add_library(rbp2_link_core STATIC
    link_core/src/link_core.cpp
)
target_include_directories(rbp2_link_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/link_core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
)
target_compile_features(rbp2_link_core PUBLIC cxx_std_17)
target_link_libraries(rbp2_link_core PUBLIC rbp2_protocol)

add_library(rbp2_transport STATIC
    transport/src/frame_tx_queue.cpp
)
target_include_directories(rbp2_transport PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
)
target_compile_features(rbp2_transport PUBLIC cxx_std_17)

add_library(rbp2_robot_codec STATIC application/src/robot_codec.cpp)
target_include_directories(rbp2_robot_codec PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/application/include
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
)
target_compile_features(rbp2_robot_codec PUBLIC cxx_std_17)

add_library(rbp2_rbrp_codec STATIC
    gateway/src/rbrp_codec.cpp
)
target_include_directories(rbp2_rbrp_codec PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
)
target_compile_features(rbp2_rbrp_codec PUBLIC cxx_std_17)

add_library(rbp2_gateway_core STATIC
    gateway/src/control_gateway_core.cpp
)
target_include_directories(rbp2_gateway_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
)
target_compile_features(rbp2_gateway_core PUBLIC cxx_std_17)
target_link_libraries(rbp2_gateway_core PUBLIC rbp2_protocol)

# Keep Linux headers and syscalls out of all portable targets.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(Threads REQUIRED)

    add_library(rbp2_posix_serial_transport STATIC
        transport/src/posix_serial_transport.cpp
    )
    target_link_libraries(rbp2_posix_serial_transport PUBLIC rbp2_transport)
    target_compile_features(rbp2_posix_serial_transport PUBLIC cxx_std_17)

    add_library(rbp2_serial_session STATIC session/src/serial_session.cpp)
    target_include_directories(rbp2_serial_session PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/session/include
    )
    target_link_libraries(rbp2_serial_session PUBLIC rbp2_link_core rbp2_posix_serial_transport)
    target_compile_features(rbp2_serial_session PUBLIC cxx_std_17)

    add_library(rbp2_link_runtime STATIC runtime/src/link_runtime.cpp)
    target_include_directories(rbp2_link_runtime PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/runtime/include
    )
    target_link_libraries(rbp2_link_runtime PUBLIC rbp2_serial_session)
    target_compile_features(rbp2_link_runtime PUBLIC cxx_std_17)

    add_library(rbp2_onboard_application STATIC application/src/onboard_application.cpp)
    target_link_libraries(rbp2_onboard_application PUBLIC rbp2_link_runtime rbp2_robot_codec)
    target_compile_features(rbp2_onboard_application PUBLIC cxx_std_17)

    add_library(rbp2_gateway_linux_application_port STATIC
        gateway/linux/src/linux_onboard_application_port.cpp
    )
    target_include_directories(rbp2_gateway_linux_application_port PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/linux/include
    )
    target_link_libraries(rbp2_gateway_linux_application_port PUBLIC
        rbp2_gateway_core rbp2_onboard_application
    )
    target_compile_features(rbp2_gateway_linux_application_port PUBLIC cxx_std_17)

    add_library(rbp2_gateway_tcp STATIC
        gateway/linux/src/tcp_adapter.cpp
    )
    target_include_directories(rbp2_gateway_tcp PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/linux/include
    )
    target_link_libraries(rbp2_gateway_tcp PUBLIC
        rbp2_rbrp_codec Threads::Threads
    )
    target_compile_features(rbp2_gateway_tcp PUBLIC cxx_std_17)

    add_library(rbp2_gateway_owner STATIC
        gateway/linux/src/gateway_owner.cpp
    )
    target_include_directories(rbp2_gateway_owner PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/linux/include
    )
    target_link_libraries(rbp2_gateway_owner PUBLIC
        rbp2_gateway_core
        rbp2_gateway_linux_application_port
        rbp2_gateway_tcp
        Threads::Threads
    )
    target_compile_features(rbp2_gateway_owner PUBLIC cxx_std_17)

    add_executable(robobeetle_pi_smoke tools/robobeetle_pi_smoke.cpp)
    target_link_libraries(robobeetle_pi_smoke PRIVATE rbp2_onboard_application)

    add_executable(robobeetle_pi_gateway tools/robobeetle_pi_gateway.cpp)
    target_link_libraries(robobeetle_pi_gateway PRIVATE rbp2_gateway_owner)
endif()

if(BUILD_TESTING)
    add_executable(rbp2_cpg_parameters_codec_tests tests/cpg_parameters_codec_tests.cpp)
    target_link_libraries(rbp2_cpg_parameters_codec_tests PRIVATE rbp2_protocol rbp2_rbrp_codec)
    add_test(NAME rbp2_cpg_parameters_codec_tests COMMAND rbp2_cpg_parameters_codec_tests)
    enable_language(C)
    add_executable(rbp2_motion_state_codec_tests
        tests/motion_state_codec_tests.cpp
        ../RoboBeetleFirmware/Core/Communication/motion_state_codec.c
        ../RoboBeetleFirmware/Core/Src/rb_protocol_v2.c
    )
    target_include_directories(rbp2_motion_state_codec_tests PRIVATE
        ../RoboBeetleFirmware/Core/Communication
        ../RoboBeetleFirmware/Core/Inc
    )
    target_link_libraries(rbp2_motion_state_codec_tests PRIVATE rbp2_protocol rbp2_rbrp_codec)
    add_test(NAME rbp2_motion_state_codec_tests COMMAND rbp2_motion_state_codec_tests)

    add_executable(rbp2_rbrp_codec_tests tests/rbrp_codec_tests.cpp)
    target_include_directories(rbp2_rbrp_codec_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_compile_features(rbp2_rbrp_codec_tests PRIVATE cxx_std_17)
    target_link_libraries(rbp2_rbrp_codec_tests PRIVATE rbp2_rbrp_codec)
    add_test(NAME rbp2_rbrp_codec_tests COMMAND rbp2_rbrp_codec_tests)

    add_executable(rbp2_gateway_core_tests tests/control_gateway_core_tests.cpp)
    target_include_directories(rbp2_gateway_core_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_compile_features(rbp2_gateway_core_tests PRIVATE cxx_std_17)
    target_link_libraries(rbp2_gateway_core_tests PRIVATE rbp2_gateway_core)
    add_test(NAME rbp2_gateway_core_tests COMMAND rbp2_gateway_core_tests)

    add_executable(rbp2_protocol_tests
        tests/protocol_test_main.cpp
        tests/protocol_golden_tests.cpp
        tests/crc_tests.cpp
        tests/cobs_tests.cpp
        tests/codec_tests.cpp
        tests/stream_decoder_tests.cpp
        tests/link_core_tests.cpp
        tests/frame_tx_queue_tests.cpp
    )
    target_include_directories(rbp2_protocol_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
        ${CMAKE_CURRENT_SOURCE_DIR}/link_core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_link_libraries(rbp2_protocol_tests PRIVATE rbp2_protocol rbp2_link_core rbp2_transport)
    add_test(NAME rbp2_protocol_tests COMMAND rbp2_protocol_tests)

    add_executable(rbp2_motion_telemetry_fifo_tests tests/motion_telemetry_fifo_tests.cpp)
    target_link_libraries(rbp2_motion_telemetry_fifo_tests PRIVATE rbp2_rbrp_codec)
    add_test(NAME rbp2_motion_telemetry_fifo_tests COMMAND rbp2_motion_telemetry_fifo_tests)

    add_executable(rbp2_robot_codec_tests tests/robot_codec_tests.cpp)
    target_include_directories(rbp2_robot_codec_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/application/include
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_compile_features(rbp2_robot_codec_tests PRIVATE cxx_std_17)
    target_link_libraries(rbp2_robot_codec_tests PRIVATE rbp2_robot_codec)
    add_test(NAME rbp2_robot_codec_tests COMMAND rbp2_robot_codec_tests)

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_executable(rbp2_posix_serial_transport_tests
            tests/posix_serial_transport_tests.cpp
        )
        target_link_libraries(rbp2_posix_serial_transport_tests PRIVATE rbp2_posix_serial_transport)
        add_test(NAME rbp2_posix_serial_transport_tests COMMAND rbp2_posix_serial_transport_tests)

        add_executable(rbp2_serial_session_tests tests/serial_session_tests.cpp)
        target_link_libraries(rbp2_serial_session_tests PRIVATE rbp2_serial_session)
        add_test(NAME rbp2_serial_session_tests COMMAND rbp2_serial_session_tests)

        add_executable(rbp2_link_runtime_tests tests/link_runtime_tests.cpp)
        target_link_libraries(rbp2_link_runtime_tests PRIVATE rbp2_link_runtime)
        add_test(NAME rbp2_link_runtime_tests COMMAND rbp2_link_runtime_tests)
        set_tests_properties(rbp2_link_runtime_tests PROPERTIES TIMEOUT 15)

        add_executable(rbp2_onboard_application_tests tests/onboard_application_tests.cpp)
        target_include_directories(rbp2_onboard_application_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/application/include
        )
        target_link_libraries(rbp2_onboard_application_tests PRIVATE rbp2_onboard_application)
        add_test(NAME rbp2_onboard_application_tests COMMAND rbp2_onboard_application_tests)
        set_tests_properties(rbp2_onboard_application_tests PROPERTIES TIMEOUT 15)

        add_executable(rbp2_gateway_linux_application_port_tests
            tests/linux_onboard_application_port_tests.cpp
        )
        target_include_directories(rbp2_gateway_linux_application_port_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
            ${CMAKE_CURRENT_SOURCE_DIR}/gateway/linux/include
            ${CMAKE_CURRENT_SOURCE_DIR}/application/include
            ${CMAKE_CURRENT_SOURCE_DIR}/runtime/include
            ${CMAKE_CURRENT_SOURCE_DIR}/session/include
            ${CMAKE_CURRENT_SOURCE_DIR}/link_core/include
            ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
            ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
            ${CMAKE_CURRENT_SOURCE_DIR}/tests
        )
        target_compile_features(rbp2_gateway_linux_application_port_tests PRIVATE cxx_std_17)
        target_link_libraries(rbp2_gateway_linux_application_port_tests
            PRIVATE rbp2_gateway_linux_application_port)
        add_test(NAME rbp2_gateway_linux_application_port_tests
            COMMAND rbp2_gateway_linux_application_port_tests)
        set_tests_properties(rbp2_gateway_linux_application_port_tests PROPERTIES TIMEOUT 20)

        add_executable(rbp2_tcp_gateway_integration_tests
            tests/tcp_gateway_integration_tests.cpp
        )
        target_include_directories(rbp2_tcp_gateway_integration_tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/gateway/include
            ${CMAKE_CURRENT_SOURCE_DIR}/gateway/linux/include
            ${CMAKE_CURRENT_SOURCE_DIR}/tests
        )
        target_compile_features(rbp2_tcp_gateway_integration_tests PRIVATE cxx_std_17)
        target_link_libraries(rbp2_tcp_gateway_integration_tests
            PRIVATE rbp2_gateway_tcp rbp2_gateway_owner)
        add_test(NAME rbp2_tcp_gateway_integration_tests
            COMMAND rbp2_tcp_gateway_integration_tests)
        set_tests_properties(rbp2_tcp_gateway_integration_tests PROPERTIES TIMEOUT 20)
    endif()

    # MinGW test executables need the runtime beside the selected compiler
    # first in the child-process PATH. This is test-process setup only; no
    # runtime DLLs are copied into the repository or product targets.
    if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        get_filename_component(rbp2_test_compiler_bin
            ${CMAKE_CXX_COMPILER} DIRECTORY)
        set_tests_properties(
            rbp2_rbrp_codec_tests
            rbp2_gateway_core_tests
            rbp2_protocol_tests
            rbp2_robot_codec_tests
            rbp2_motion_telemetry_fifo_tests
            PROPERTIES ENVIRONMENT "PATH=${rbp2_test_compiler_bin};$ENV{PATH}"
        )
    endif()
endif()
