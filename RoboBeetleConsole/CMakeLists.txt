cmake_minimum_required(VERSION 3.19)

project(RoboBeetleConsole VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 6.2 REQUIRED COMPONENTS Core Widgets SerialPort)

add_library(RoboBeetleCore
    src/protocol/Crc16.h
    src/protocol/Crc16.cpp
    src/protocol/Packet.h
    src/protocol/PacketCodec.h
    src/protocol/PacketCodec.cpp
    src/protocol/StreamDecoder.h
    src/protocol/StreamDecoder.cpp
    src/transport/ITransport.h
    src/transport/FakeTransport.h
    src/transport/FakeTransport.cpp
    src/transport/SerialTransport.h
    src/transport/SerialTransport.cpp
    src/robot/RobotCommand.h
    src/robot/RobotController.h
    src/robot/RobotController.cpp
)

target_include_directories(RoboBeetleCore PUBLIC src)
target_link_libraries(RoboBeetleCore PUBLIC Qt6::Core Qt6::SerialPort)

add_executable(RoboBeetleConsole
    src/main.cpp
    src/ui/MainWindow.h
    src/ui/MainWindow.cpp
)

target_link_libraries(RoboBeetleConsole PRIVATE RoboBeetleCore Qt6::Widgets)

include(CTest)
if(BUILD_TESTING)
    add_executable(protocol_tests tests/protocol_tests.cpp)
    target_link_libraries(protocol_tests PRIVATE RoboBeetleCore)
    add_test(NAME protocol_tests COMMAND protocol_tests)

    add_executable(robot_controller_tests tests/robot_controller_tests.cpp)
    target_link_libraries(robot_controller_tests PRIVATE RoboBeetleCore)
    add_test(NAME robot_controller_tests COMMAND robot_controller_tests)
endif()
