# Slice 4 Task 03 — Operator Console UX Final Implementation Record

## Purpose

This file records the actual implementation and acceptance path for the final
Task 03 state. Earlier implementation steps were refined during repeated native
Qt review; the final accepted implementation is commit
`c3b2fc8f38d8e8c38d7b866269c8b449c65e8b5b`.

## Final implementation sequence

1. Reorganized MainWindow around one vertical workspace splitter with Realtime
   Video as the dominant upper content.
2. Kept one shared Pi Host and existing Robot / Video / Vision endpoints.
3. Moved Vision Status beside the video and Vision Controls below it.
4. Consolidated Inference and Recording into authoritative state-driven operator
   toggles without changing HTTP endpoints or reconciliation rules.
5. Converted the right side into one compact Leak / IMU / Depth / Protocol /
   Actuator status rail.
6. Converted Actuator Control into five status-only rows.
7. Centralized editable servo controls in the Servo Fine Control tab.
8. Integrated Log into Motion/Gait and kept Motion Stop directly reachable.
9. Preserved Vision Details, Telemetry Details, Protocol Details and Data Plots as
   dedicated tabs.
10. Added exact source-aspect VideoView geometry to eliminate the narrow black
    letterbox edge.
11. Tuned startup geometry to a 1420-wide default while preserving 1100x720,
    1420x880 and 1600x1000 operation.
12. Added distinct presentation for Video FPS, inference FPS, Inference Running,
    and enabled/ACKed servos.
13. Renamed the pending Backward-facing operator label to Brake while keeping the
    existing underlying motion action disabled.
14. Updated MainWindow tests and Task 03 layout tests to lock the final reviewed
    UI ownership and behavior.

## Final automated gates

The final branch was rebuilt with:

- Qt 6.11.2
- MinGW GCC 13.1.0
- `QT_QPA_PLATFORM=offscreen`

Required gate:

- protocol_tests
- vision_protocol_tests
- vision_stream_decoder_tests
- vision_frame_decoder_tests
- vision_client_tests
- vision_control_client_tests
- inference_ui_state_tests
- video_view_tests
- robot_controller_tests
- servo_descriptor_tests
- imu_monitor_tests
- depth_monitor_tests
- main_window_tests
- main_window_inference_controls_tests
- rbrp_client_session_tests
- remote_robot_controller_tests
- main_window_layout_tests

Final result: **17/17 PASS**.

`git diff --check` passes; Windows line-ending warnings are non-functional.

## Native / hardware acceptance

After the automated gate, the final UI was exercised natively on Windows with the
real Pi:

- Robot TCP 47000
- RBVS 47010
- Vision HTTP 47011
- USB camera 640x480
- inference Running diagnostics
- real detection counts
- Snapshot
- manual inference Stop/Start
- servo enabled/ACKed status

The Pi Vision backend was updated to the Slice 4 manual-inference implementation.
A persistent `robobeetle-vision-live.service` was created and user linger enabled.

## Windows deployment acceptance

The raw CMake build executable still depends on Qt/MinGW runtime DLL discovery.

A portable directory was therefore created using `windeployqt --compiler-runtime`.
The deployed `RoboBeetleConsole.exe` was then started without manually setting
the Qt/MinGW PATH, confirming the DLL deployment issue was resolved for the
portable bundle.

Generated deployment DLLs are kept outside the Git worktree and are not committed.

## Release boundary

Task 03 ends with the operator console UI/manual-inference workflow accepted.

Not part of Task 03:

- detection text overlay transport/rendering
- target tracking/selection/lock
- visual servo
- PID
- robot motion from inference
- RBRP redesign
- STM32 changes
- model retraining
- confidence threshold tuning

Those belong to later Vision slices.
