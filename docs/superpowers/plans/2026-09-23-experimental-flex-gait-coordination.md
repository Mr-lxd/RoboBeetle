# Experimental Flex Gait and Front/Rear Coordination Implementation Plan

> **For agentic workers:** Work through these tasks in order in the isolated `codex/motion-gait-flex-coordination` worktree. Use test-first changes for each behavior; preserve the fixed wire values and physical semantics in the approved design.

**Goal:** Add a third Experimental Flex generator, common Front/Rear coordination and neutral-hold turn policy, and end-to-end selectors in Firmware, Pi/RBRP, and Qt.

**Architecture:** Generators continue to emit `joint_targets_t`. MotionManager applies Rear sign coordination and then the inactive-side logical-zero turn policy after each successful sample, before the existing 750 ms blend and envelope guard. Protocol V2 carries one-byte gait and coordination values; RBRP carries the coordination selector as command kind `0x09` plus one value byte. Qt confirms requested selector values only after the existing direct ACK or remote command outcome.

**Tech Stack:** STM32 C11 host/target CMake, Pi C++20 CMake/CTest, Qt 6/C++20, Protocol V2, RBRP.

---

## File map

### Firmware

- Create `RoboBeetleFirmware/Core/Motion/experimental_flex_gait_generator.h` and `.c` for the standalone 2000 ms waveform.
- Modify `RoboBeetleFirmware/Core/Motion/motion_types.h`, `motion_config.h`, `motion_manager.h`, `motion_manager.c`, `simple_gait_generator.c`, and `cpg_gait_generator.h/.c` for backend value 2, coordination, common turn handling, and removal of generator-side turn scaling.
- Modify `RoboBeetleFirmware/Core/App/app_main.c`, `Core/Inc/rb_protocol_v2.h`, `Core/Communication/protocol_dispatcher.c`, and `Core/Diagnostics/motion_timing_diagnostics.h` to construct/register the generator and accept Protocol V2 selector `0x17`.
- Modify `RoboBeetleFirmware/CMakeLists.txt` and `RoboBeetleFirmware/tests/run_host_tests.ps1` so the new source and test participate in target and full host gates.
- Extend `RoboBeetleFirmware/tests/simple_gait_generator_tests.c`, `tests/test_cpg_gait_generator.c`, `tests/motion_manager_tests.c`, `tests/protocol_dispatcher_tests.c`, and `tests/app_main_backend_tests.c`; create `tests/experimental_flex_gait_generator_tests.c`.

### Pi application and gateway

- Modify `RoboBeetlePi/protocol/include/robobeetle/protocol/message_types.hpp` for Protocol V2 `0x17`.
- Modify `RoboBeetlePi/application/include/robobeetle/application/robot_types.hpp`, `robot_codec.hpp`, `onboard_application.hpp`, `application/src/robot_codec.cpp`, and `application/src/onboard_application.cpp` for the typed selector and one-byte encoder/submission method.
- Modify `RoboBeetlePi/gateway/include/robobeetle/gateway/gateway_types.hpp`, `gateway/src/rbrp_codec.cpp`, `gateway/src/control_gateway_core.cpp`, and `gateway/linux/src/linux_onboard_application_port.cpp` for RBRP kind `0x09`, validation, and forwarding.
- Extend `RoboBeetlePi/tests/protocol_golden_tests.cpp`, `robot_codec_tests.cpp`, `onboard_application_tests.cpp`, `rbrp_codec_tests.cpp`, `control_gateway_core_tests.cpp`, and `linux_onboard_application_port_tests.cpp`.

### Qt console

- Modify `RoboBeetleConsole/src/robot/RobotCommand.h`, `src/protocol/Packet.h`, and `src/controller/IConsoleController.h` for the public enums, message type, and symmetric coordination interface.
- Modify `src/robot/RobotController.h/.cpp` and `src/remote/RemoteRobotController.h/.cpp` for direct and RBRP selector requests, ACK-confirmed state, cross-selector pending exclusion, and local active-Motion rejection.
- Modify `src/ui/MainWindow.h/.cpp` for the same-row Gait selector, independent Front/Rear selector, confirmed/unknown labels, and shared enabled-state predicate.
- Extend `RoboBeetleConsole/tests/protocol_tests.cpp`, `robot_controller_tests.cpp`, `remote_robot_controller_tests.cpp`, `main_window_tests.cpp`, and `main_window_layout_tests.cpp`.

## Task 1: Experimental Flex generator

**Files:** Create the two generator files and `tests/experimental_flex_gait_generator_tests.c`; register them in Firmware CMake and `tests/run_host_tests.ps1`.

- [ ] **Step 1: Add failing generator tests.** Assert that reset samples `q = 0`; advancing from reset by 1350 ms reaches the power start (`-1000`), another 1300 ms reaches power end (`+1000`), and another 700 ms returns to `-1000`. Assert continuity around the 1300 ms segment boundary, pair mapping `FR=FL=q`, `RR=RL=-q`, ±1000 cdeg bounds, STOP zero, FrontAxis ASCEND `+1000`, DESCEND `-1000`, and BACKWARD rejection.
- [ ] **Step 2: Register the test and source.** Add the test executable/source pair to the CMake firmware test configuration and host-runner target list; add the generator source to the production firmware source list.
- [ ] **Step 3: Run the new test and verify RED.** From `RoboBeetleFirmware`, run `powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1`; before implementation, the new target must fail because its API/source is absent.
- [ ] **Step 4: Implement the generator.** Store phase in `[0, 2000)`, initialize/reset to 650 ms, advance using `dt_ms % 2000` to avoid unsigned addition overflow, use `q=-A*cos(pi*s)` for `[0,1300)` and `q=+A*cos(pi*s)` for `[1300,2000)`, and keep amplitude/bias scaling in the generator inputs.
- [ ] **Step 5: Run the generator test.** Re-run the host runner and confirm the new executable passes.

## Task 2: MotionManager backend and common policies

**Files:** Modify `motion_types.h`, `motion_manager.h/.c`, `app_main.c`, and call sites in firmware tests/wrappers. Extend `motion_manager_tests.c`.

- [ ] **Step 1: Add failing deterministic fake-target tests.** Start from `SameDirection`; assert sampled Front/Rear targets are unchanged. Set `OppositeDirection`; assert both Front targets are unchanged and both Rear signs flip. Assert invalid values reject, STOPPED accepts, RUNNING/STOPPING/FAULTED return BUSY, and selecting coordination causes zero Servo writes and leaves ownership and `last_targets` unchanged.
- [ ] **Step 2: Add failing turning and transition tests.** With a fake generator returning distinct nonzero joints, assert settled TurnLeft zeros FL/RL but leaves FR/RR active; TurnRight zeros FR/RR but leaves FL/RL active. Repeat the active-side relation with both coordination values. Assert all four paddles remain in `write_mask` and Motion owns them. Drive Forward→TurnLeft across `MOTION_TRANSITION_DURATION_MS` and assert the left pair blends to zero and settles after exactly 750 ms.
- [ ] **Step 3: Extend the registered backend fixture.** Assert backend value 2 is valid/selectable while STOPPED, value 3 is invalid, active selection is BUSY, selected generator reset behavior follows the existing backend contract, and selection itself performs no Servo write.
- [ ] **Step 4: Run `motion_manager_tests` and verify RED.** Use the Firmware host runner and confirm the new coordination/turn/backend assertions fail before policy implementation.
- [ ] **Step 5: Implement the common policy.** Add `motion_front_rear_coordination_t`, default it to SameDirection, add the getter/setter, register all three generators, and apply coordination then turn neutral-hold inside the shared successful-sample path before transition interpolation completes and before sanitize/write. Keep required paddle masks and Servo ownership unchanged.
- [ ] **Step 6: Run `motion_manager_tests`.** Confirm the new deterministic policy, state gates, transition, and no-write assertions pass.

## Task 3: Remove generator-side turn scaling and wire the third backend

**Files:** Modify `simple_gait_generator.c`, `motion_config.h`, `cpg_gait_generator.h/.c`, `Core/App/app_main.c`, `Core/Diagnostics/motion_timing_diagnostics.h`, `tests/simple_gait_generator_tests.c`, `tests/test_cpg_gait_generator.c`, and all `motion_manager_init_with_backends` call sites in the host tests.

- [ ] **Step 1: Rewrite SimpleGait turn assertions.** At one fixed phase, assert TurnLeft and TurnRight generator samples exactly equal Forward; keep Front=`q`, Rear=`-q`, Ascend/Descend FrontAxis, and Backward-invalid assertions.
- [ ] **Step 2: Rewrite CPG adapter assertions.** Remove `turn_reduced_side_scale` profile setup/assertions. For Forward, TurnLeft, and TurnRight samples, assert the adapter installs the same full four-node target-amplitude vector and does not alter `cpg_core.c` state ordering or equations.
- [ ] **Step 3: Run the focused generator tests and verify RED.** Run `simple_gait_generator_tests` and `cpg_gait_generator_tests` through the Firmware host runner; old turn-scaling behavior must fail the new assertions.
- [ ] **Step 4: Remove scaling and register backend 2.** Remove SimpleGait side scales and the unused shared turn-scale constant; remove the CPG profile field and all related code. Add ExperimentalFlex value 2/static assertions, instantiate it in `app_main.c`, pass all three generators to MotionManager, and update timing-diagnostic backend values.
- [ ] **Step 5: Run focused generator and MotionManager tests.** Confirm exact SimpleGait/CPG turn parity and backend selection behavior; verify `cpg_core.c` is unchanged.

## Task 4: Firmware Protocol V2 coordination command

**Files:** Modify `Core/Inc/rb_protocol_v2.h`, `Core/Communication/protocol_dispatcher.c`, and `tests/protocol_dispatcher_tests.c`.

- [ ] **Step 1: Add failing dispatcher tests.** For `0x17`, cover SameDirection=0, OppositeDirection=1, out-of-range value, zero/two-byte payload, and BUSY while Motion is active. Verify accepted selection has no Servo write. Extend `0x16` tests to accept backend 2 and reject backend 3; preserve existing one-byte backend payload and BUSY precedence.
- [ ] **Step 2: Run `protocol_dispatcher_tests` and verify RED.** Run it through `tests/run_host_tests.ps1`.
- [ ] **Step 3: Implement `0x17`.** Add only the new message enum and exact one-byte dispatch path; map invalid coordination to `INVALID_PAYLOAD` and non-STOPPED state to `BUSY`. Do not change version, ACK bytes, `0x15`, or `0x16` payload shape.
- [ ] **Step 4: Run `protocol_dispatcher_tests`.** Confirm new selectors pass and existing motion/Backward behavior is unchanged.

## Task 5: Pi Protocol V2 application and RBRP gateway

**Files:** Modify the Pi protocol/application/gateway files and six listed Pi test files.

- [ ] **Step 1: Add failing codec and application tests.** Assert the new protocol type is `0x17`; SameDirection/OppositeDirection encode as one-byte payloads `0`/`1`; invalid values reject; `OnboardApplication::set_front_rear_coordination` submits the `0x17` frame. Assert ExperimentalFlex backend encodes as `2` and backend `3` rejects.
- [ ] **Step 2: Add failing RBRP/gateway/port tests.** Assert RobotCommandKind `0x09` decodes exactly two payload bytes (kind plus value), accepts values 0/1, rejects invalid value/length, validates the typed command, forwards it as `0x17` with one payload byte, and preserves command outcome correlation. Assert backend 2 is valid through the same path. Leave all Backward qualification tests untouched.
- [ ] **Step 3: Run Pi RED tests.** Configure `RoboBeetlePi` in an external build directory, build `robot_codec_tests`, `onboard_application_tests`, `rbrp_codec_tests`, `control_gateway_core_tests`, `linux_onboard_application_port_tests`, and `protocol_golden_tests`, then run those CTest names; the new assertions must fail before code changes.
- [ ] **Step 4: Implement Pi types/codecs/submission.** Add matching application and gateway enums, `encode_front_rear_coordination`, and `OnboardApplication::set_front_rear_coordination`; preserve `SetServoPwm=0x08`, assign coordination `0x09`, decode/validate one value, and forward through `LinuxOnboardApplicationPort`.
- [ ] **Step 5: Run Pi focused tests.** Confirm all six focused test executables pass and existing Backward qualification assertions remain unchanged.

## Task 6: Qt protocol and controller selector lifecycle

**Files:** Modify Qt command/message enums, `IConsoleController`, direct and remote controller headers/sources, and controller/protocol tests.

- [ ] **Step 1: Add failing protocol/controller tests.** Verify ExperimentalFlex wire value 2, `0x17`, coordination wire bytes 0/1, invalid enum rejection, ACK-confirmed/requested/pending state, BUSY/Error/timeout/disconnect behavior, and direct/RBRP forwarding.
- [ ] **Step 2: Add cross-gating tests.** While a gait selector is pending, a coordination selector request must be locally rejected; while coordination is pending, gait selection must be locally rejected. During active Motion or pending Motion commands both controller methods reject without writing/submitting commands. Selector requests must not emit StartMotion.
- [ ] **Step 3: Run protocol/controller RED tests.** Run the Qt `protocol_tests`, `robot_controller_tests`, and `remote_robot_controller_tests` CTest entries before implementation.
- [ ] **Step 4: Implement symmetric selector lifecycle.** Add `FrontRearCoordination`, request/confirmed/pending interface methods, and state-change signal. Correlate direct `0x17` ACKs and remote kind-`0x09` command outcomes independently. Clear pending/confirmed state through the same disconnect/fail-close paths as gait selection. Gate either selector on active Motion and on either pending selector.
- [ ] **Step 5: Run protocol/controller tests.** Confirm the direct and remote state transitions, rejection paths, and no-motion-start assertions pass.

## Task 7: Qt Gait panel

**Files:** Modify `RoboBeetleConsole/src/ui/MainWindow.h/.cpp` and `tests/main_window_tests.cpp`, `tests/main_window_layout_tests.cpp`.

- [ ] **Step 1: Add failing panel assertions.** Verify the Gait label and combo share one row; options are Unknown, SimpleGait, CPG, Experimental Flex; the second row is labelled Front / Rear with Unknown, Same Direction, Opposite Direction; no Phase labels or Chinese explanatory text appear. Check Confirmed — labels display only ACK-confirmed values and otherwise Unknown.
- [ ] **Step 2: Add failing enable/side-effect assertions.** With inactive Motion, active control, and no pending selectors, both combos enable. During running/stopping/pending Motion or either pending selector, both disable. Selecting either sends its own selector command, leaves confirmed display unchanged until ACK, and emits zero StartMotion/Backward/Brake commands.
- [ ] **Step 3: Run the two Qt UI RED tests.** Run the `main_window_tests` and `main_window_layout_tests` CTest entries.
- [ ] **Step 4: Implement only the Gait panel changes.** Keep the Gait combo on the Gait label row. Add the Front / Rear row, independent confirmation labels, and one shared enabled-state predicate using controller control state, `isMotionActive()`, and both pending flags. Use English labels exactly as specified; selecting a value must call only its selector method.
- [ ] **Step 5: Run both Qt UI tests.** Confirm row geometry, option names, ACK behavior, enabled-state gating, Brake disabled state, and no unexpected motion commands.

## Task 8: Full verification and delivery

- [ ] **Step 1: Run Firmware host gate.** From `RoboBeetleFirmware`, run `powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1`; require all listed host executables and compile contracts to pass.
- [ ] **Step 2: Run Pi full CMake/CTest gate.** Use `C:\Users\laixindong\.codex\worktrees\motion-gait-flex-coordination-build\pi` as the external build directory; run `cmake -S RoboBeetlePi -B C:\Users\laixindong\.codex\worktrees\motion-gait-flex-coordination-build\pi -DCMAKE_BUILD_TYPE=Release`, `cmake --build C:\Users\laixindong\.codex\worktrees\motion-gait-flex-coordination-build\pi --parallel`, and `ctest --test-dir C:\Users\laixindong\.codex\worktrees\motion-gait-flex-coordination-build\pi --output-on-failure`; require zero failures.
- [ ] **Step 3: Run Qt full configure/build/CTest gate.** Configure `RoboBeetleConsole` with the installed Qt 6 (`D:\Qt\6.11.2\mingw_64`) and matching MinGW (`D:\Qt\Tools\mingw1310_64\bin`) in `C:\Users\laixindong\.codex\worktrees\motion-gait-flex-coordination-build\qt`, build all targets, then run all CTest tests with `D:\Qt\Tools\mingw1310_64\bin` and `D:\Qt\6.11.2\mingw_64\bin` at the front of PATH; require zero failures.
- [ ] **Step 4: Attempt ARM build when available.** Check for `arm-none-eabi-gcc`, configure with `RoboBeetleFirmware/cmake/gcc-arm-none-eabi.cmake`, and build the firmware target. Report unavailable toolchain separately; do not infer target or physical verification from host tests.
- [ ] **Step 5: Audit scope and cleanliness.** Run `git diff --check`, verify only the planned Firmware/Pi/Qt sources/tests plus the design and plan documents changed, and verify `git status --short` is empty after commits.
- [ ] **Step 6: Commit and push.** Commit the implementation on `codex/motion-gait-flex-coordination`, push that branch, create no PR, merge nothing, then report branch, base SHA, HEAD, files, architecture, Firmware/Pi/Qt test commands and outcomes, ARM build availability/result, diff check, and clean status. Stop for ChatGPT Review.
