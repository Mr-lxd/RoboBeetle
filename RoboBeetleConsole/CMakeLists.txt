cmake_minimum_required(VERSION 3.19)

project(RoboBeetleConsole VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 6.2 REQUIRED COMPONENTS Core Gui Widgets Network SerialPort)

add_library(RoboBeetleCore
    src/controller/IConsoleController.h
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
    src/remote/RbrpClientSession.h
    src/remote/RbrpClientSession.cpp
    src/remote/RemoteRobotController.h
    src/remote/RemoteRobotController.cpp
    src/vision/VisionProtocol.h
    src/vision/VisionProtocol.cpp
    src/vision/VisionStreamDecoder.h
    src/vision/VisionStreamDecoder.cpp
    src/vision/VisionFrameDecoder.h
    src/vision/VisionFrameDecoder.cpp
    src/vision/VisionClient.h
    src/vision/VisionClient.cpp
    src/vision/VisionControlClient.h
    src/vision/VisionControlClient.cpp
    src/vision/DetectionMetadata.h
    src/vision/DetectionMetadata.cpp
    src/vision/TargetState.h
    src/vision/TargetState.cpp
    src/vision/VisualPolicyConfig.h
    src/vision/VisualTargetStateMachine.h
    src/vision/VisualTargetStateMachine.cpp
    src/vision/VisualCommandPolicy.h
    src/vision/VisualCommandPolicy.cpp
    src/vision/VisualDiagnosticSession.h
    src/vision/VisualDiagnosticSession.cpp
    src/vision/DetectionStreamDecoder.h
    src/vision/DetectionStreamDecoder.cpp
    src/vision/DetectionClient.h
    src/vision/DetectionClient.cpp
    src/vision/InferenceUiState.h
    src/vision/InferenceUiState.cpp
    ../RoboBeetlePi/gateway/src/rbrp_codec.cpp
    src/robot/RobotCommand.h
    src/robot/LeakStatus.h
    src/robot/ImuSnapshot.h
    src/robot/ImuSnapshot.cpp
    src/robot/ImuMonitor.h
    src/robot/ImuMonitor.cpp
    src/robot/DepthSnapshot.h
    src/robot/DepthSnapshot.cpp
    src/robot/DepthMonitor.h
    src/robot/DepthMonitor.cpp
    src/robot/ServoDescriptor.h
    src/robot/ServoDescriptor.cpp
    src/robot/RobotController.h
    src/robot/RobotController.cpp
)

target_include_directories(RoboBeetleCore PUBLIC
    src
    ../RoboBeetlePi/gateway/include
)
target_link_libraries(RoboBeetleCore PUBLIC Qt6::Core Qt6::Gui Qt6::Network Qt6::SerialPort)

set(RB_CONSOLE_UI_SOURCES
    src/ui/MainWindow.h
    src/ui/MainWindow.cpp
    src/vision/VideoView.h
    src/vision/VideoView.cpp
    src/ui/ElidedLabel.h
    src/ui/ElidedLabel.cpp
)

add_executable(RoboBeetleConsole
    src/main.cpp
    ${RB_CONSOLE_UI_SOURCES}
)

target_link_libraries(RoboBeetleConsole PRIVATE RoboBeetleCore Qt6::Widgets)

include(CTest)
if(BUILD_TESTING)
    add_executable(protocol_tests tests/protocol_tests.cpp)
    target_link_libraries(protocol_tests PRIVATE RoboBeetleCore)
    add_test(NAME protocol_tests COMMAND protocol_tests)

    add_executable(vision_protocol_tests tests/vision_protocol_tests.cpp)
    target_link_libraries(vision_protocol_tests PRIVATE RoboBeetleCore)
    add_test(NAME vision_protocol_tests COMMAND vision_protocol_tests)

    add_executable(vision_stream_decoder_tests tests/vision_stream_decoder_tests.cpp)
    target_link_libraries(vision_stream_decoder_tests PRIVATE RoboBeetleCore)
    add_test(NAME vision_stream_decoder_tests COMMAND vision_stream_decoder_tests)

    add_executable(vision_frame_decoder_tests tests/vision_frame_decoder_tests.cpp)
    target_link_libraries(vision_frame_decoder_tests PRIVATE RoboBeetleCore)
    add_test(NAME vision_frame_decoder_tests COMMAND vision_frame_decoder_tests)

    add_executable(vision_client_tests tests/vision_client_tests.cpp)
    target_link_libraries(vision_client_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME vision_client_tests COMMAND vision_client_tests)

    add_executable(vision_control_client_tests
        tests/vision_control_client_tests.cpp
    )
    target_link_libraries(vision_control_client_tests PRIVATE RoboBeetleCore)
    add_test(NAME vision_control_client_tests COMMAND vision_control_client_tests)

    add_executable(detection_stream_decoder_tests
        tests/detection_stream_decoder_tests.cpp
    )
    target_link_libraries(detection_stream_decoder_tests PRIVATE RoboBeetleCore)
    add_test(NAME detection_stream_decoder_tests COMMAND detection_stream_decoder_tests)

    add_executable(detection_client_tests
        tests/detection_client_tests.cpp
    )
    target_link_libraries(detection_client_tests PRIVATE RoboBeetleCore)
    add_test(NAME detection_client_tests COMMAND detection_client_tests)

    add_executable(inference_ui_state_tests
        tests/inference_ui_state_tests.cpp
    )
    target_link_libraries(inference_ui_state_tests PRIVATE RoboBeetleCore)
    add_test(NAME inference_ui_state_tests COMMAND inference_ui_state_tests)

    add_executable(video_view_tests
        tests/video_view_tests.cpp
        src/vision/VideoView.h
        src/vision/VideoView.cpp
    )
    target_include_directories(video_view_tests PRIVATE src)
    target_link_libraries(video_view_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME video_view_tests COMMAND video_view_tests)

    add_executable(target_state_tests tests/target_state_tests.cpp)
    target_link_libraries(target_state_tests PRIVATE RoboBeetleCore)
    add_test(NAME target_state_tests COMMAND target_state_tests)

    add_executable(visual_policy_config_tests tests/visual_policy_config_tests.cpp)
    target_include_directories(visual_policy_config_tests PRIVATE src)
    add_test(NAME visual_policy_config_tests COMMAND visual_policy_config_tests)
    add_executable(visual_target_state_machine_tests tests/visual_target_state_machine_tests.cpp)
    target_link_libraries(visual_target_state_machine_tests PRIVATE RoboBeetleCore)
    add_test(NAME visual_target_state_machine_tests COMMAND visual_target_state_machine_tests)
    add_executable(visual_command_policy_tests tests/visual_command_policy_tests.cpp)
    target_include_directories(visual_command_policy_tests PRIVATE src)
    target_sources(visual_command_policy_tests PRIVATE src/vision/VisualCommandPolicy.cpp)
    add_test(NAME visual_command_policy_tests COMMAND visual_command_policy_tests)
    add_executable(visual_diagnostic_session_tests tests/visual_diagnostic_session_tests.cpp)
    target_link_libraries(visual_diagnostic_session_tests PRIVATE RoboBeetleCore)
    add_test(NAME visual_diagnostic_session_tests COMMAND visual_diagnostic_session_tests)

    add_executable(robot_controller_tests tests/robot_controller_tests.cpp)
    target_link_libraries(robot_controller_tests PRIVATE RoboBeetleCore)
    add_test(NAME robot_controller_tests COMMAND robot_controller_tests)

    add_executable(servo_descriptor_tests tests/servo_descriptor_tests.cpp)
    target_link_libraries(servo_descriptor_tests PRIVATE RoboBeetleCore)
    add_test(NAME servo_descriptor_tests COMMAND servo_descriptor_tests)

    add_executable(imu_monitor_tests tests/imu_monitor_tests.cpp)
    target_link_libraries(imu_monitor_tests PRIVATE RoboBeetleCore)
    add_test(NAME imu_monitor_tests COMMAND imu_monitor_tests)

    add_executable(depth_monitor_tests tests/depth_monitor_tests.cpp)
    target_link_libraries(depth_monitor_tests PRIVATE RoboBeetleCore)
    add_test(NAME depth_monitor_tests COMMAND depth_monitor_tests)

    add_executable(main_window_tests
        tests/main_window_tests.cpp
        ${RB_CONSOLE_UI_SOURCES}
    )
    target_link_libraries(main_window_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME main_window_tests COMMAND main_window_tests)

    add_executable(main_window_inference_controls_tests
        tests/main_window_inference_controls_tests.cpp
        ${RB_CONSOLE_UI_SOURCES}
    )
    target_link_libraries(main_window_inference_controls_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME main_window_inference_controls_tests COMMAND main_window_inference_controls_tests)

    add_executable(main_window_visual_error_tests
        tests/main_window_visual_error_tests.cpp
        ${RB_CONSOLE_UI_SOURCES}
    )
    target_link_libraries(main_window_visual_error_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME main_window_visual_error_tests COMMAND main_window_visual_error_tests)

    add_executable(rbrp_client_session_tests
        tests/rbrp_client_session_tests.cpp
    )
    target_link_libraries(rbrp_client_session_tests PRIVATE RoboBeetleCore)
    add_test(NAME rbrp_client_session_tests COMMAND rbrp_client_session_tests)

    add_executable(remote_robot_controller_tests
        tests/remote_robot_controller_tests.cpp
        ${RB_CONSOLE_UI_SOURCES}
    )
    target_link_libraries(remote_robot_controller_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME remote_robot_controller_tests COMMAND remote_robot_controller_tests)

    add_executable(main_window_layout_tests
        tests/main_window_layout_tests.cpp
        tests/helpers/OperatorConsoleFixture.h
        ${RB_CONSOLE_UI_SOURCES}
    )
    target_include_directories(main_window_layout_tests PRIVATE tests src)
    target_link_libraries(main_window_layout_tests PRIVATE RoboBeetleCore Qt6::Widgets)
    add_test(NAME main_window_layout_tests COMMAND main_window_layout_tests)

    add_executable(operator_console_preview
        tests/operator_console_preview.cpp
        tests/helpers/OperatorConsoleFixture.h
        ${RB_CONSOLE_UI_SOURCES}
    )
    target_include_directories(operator_console_preview PRIVATE tests src)
    target_link_libraries(operator_console_preview PRIVATE RoboBeetleCore Qt6::Widgets)
endif()
