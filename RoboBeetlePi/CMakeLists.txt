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

if(BUILD_TESTING)
    add_executable(rbp2_protocol_tests
        tests/protocol_test_main.cpp
        tests/protocol_golden_tests.cpp
        tests/crc_tests.cpp
        tests/cobs_tests.cpp
        tests/codec_tests.cpp
        tests/stream_decoder_tests.cpp
        tests/link_core_tests.cpp
    )
    target_include_directories(rbp2_protocol_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/include
        ${CMAKE_CURRENT_SOURCE_DIR}/link_core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_link_libraries(rbp2_protocol_tests PRIVATE rbp2_protocol rbp2_link_core)
    add_test(NAME rbp2_protocol_tests COMMAND rbp2_protocol_tests)
endif()
