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

# Keep Linux headers and syscalls out of all portable targets.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
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
endif()

if(BUILD_TESTING)
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

    # The guarded RED test intentionally builds without a codec library until
    # the portable Slice 6 implementation is added.
    add_executable(rbp2_robot_codec_tests tests/robot_codec_tests.cpp)
    target_include_directories(rbp2_robot_codec_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/application/include
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_compile_features(rbp2_robot_codec_tests PRIVATE cxx_std_17)
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
    endif()
endif()
