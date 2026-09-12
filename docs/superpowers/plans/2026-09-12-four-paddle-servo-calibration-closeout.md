# Four-Paddle Servo Calibration Closeout Implementation Plan

> For agentic workers: use the subagent-driven-development workflow when delegating independent tasks. Execute each task in order, preserve the red/green checkpoints, and do not combine unrelated scope.

**Goal:** Replace the provisional paddle-servo descriptors in PR #13 with the user-accepted four-paddle logical-angle calibration contract, keep FrontAxis unchanged, and prove Firmware/Console parity without touching APC, gait, CPG, resource sources, or Protocol V2 wire behavior.

**Architecture:** Keep the existing descriptor-driven design and signed-delta interpolation. Calibration endpoints describe angle-to-pulse direction, while command PWM bounds are independent ascending numeric limits. Firmware and Console tables remain explicit parallel descriptors. No reverse flag, servo-ID special case, UI layout change, or protocol change is permitted.

**Evidence boundary:** The four paddle measurements supplied by the user are Bench Hardware Verified for RearRight/RearLeft, and Bench Measured plus User Accepted symmetry-derived endpoints for FrontRight/FrontLeft as specified below. ARM build and external hardware evidence for this code revision remain Pending unless independently available.

## Frozen contract

| Servo | -45 deg / -4500 | 0 deg / neutral | +45 deg / +4500 | raw PWM bounds | angle bounds |
|---|---:|---:|---:|---:|---:|
| FrontRight | 1000 us | 1450 us | 1900 us | 1000..1900 us | -4500..+4500 |
| FrontLeft | 2020 us | 1580 us | 1140 us | 1140..2020 us | -4500..+4500 |
| RearRight | 1110 us | 1570 us | 2030 us | 1110..2030 us | -4500..+4500 |
| RearLeft | 1940 us | 1450 us | 960 us | 960..1940 us | -4500..+4500 |
| FrontAxis | 1060 us at -90 deg | 1745 us | 2430 us at +90 deg | 1060..2430 us | -9000..+9000 |

FrontLeft and RearLeft intentionally have decreasing pulse values as logical angle increases. Their raw PWM bounds must still be validated as ascending numeric intervals.

## Scope and write ownership

- Firmware tests: RoboBeetleFirmware/tests/servo_calibration_tests.c, servo_descriptor_tests.c, and servo_service_tests.c.
- Firmware production: RoboBeetleFirmware/Core/Servo/servo_descriptor.c only.
- Console tests: RoboBeetleConsole/tests/servo_descriptor_tests.cpp and robot_controller_tests.cpp.
- Console production: RoboBeetleConsole/src/robot/ServoDescriptor.cpp only.
- Documentation: the active actuator sections in RoboBeetleFirmware/README.md, RoboBeetleConsole/README.md, ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md, and docs/engineering-lessons.md.
- Do not edit RobotController production, Protocol V2 codecs or payloads, UI layout/source, APC/USART/Safety/Leak/IMU/Depth transport, CPG/gait code, resource sources, CMake source lists, or generated MCU files.

## Task 1: Confirm baseline and preserve test-first checkpoints

1. Fetch origin and verify the current branch is feature/servo-calibration-depth-limits, PR #13 is OPEN against main, local HEAD equals the remote PR branch, and the worktree is clean. Scan changed paths for APC, gait, CPG, and resource-source scope drift; stop if any unrelated file is present.
2. Run all existing Firmware host regressions and all available Console tests before edits. Record the baseline counts and do not narrow the existing regression set.
3. Audit servo_calibration_angle_to_pulse, Firmware service validation, Console controller validation, and descriptor consumers. Confirm the shared signed-delta implementation already supports decreasing pulse mappings and that raw validation uses command_min_pulse_us/command_max_pulse_us.
4. Keep the design and implementation plan as separate reviewable checkpoints. Do not modify production code before a focused test change has produced a deterministic red result.

## Task 2: Add failing Firmware tests

1. Update servo_descriptor_tests.c expectations for exact FrontRight, FrontLeft, RearRight, RearLeft descriptors and the unchanged FrontAxis descriptor. Assert all calibration endpoints, neutral values, raw numeric bounds, and angle bounds.
2. Update servo_calibration_tests.c to call the existing servo_calibration_angle_to_pulse function for every exact endpoint and for -2250 and +2250 cdeg on all four paddles. Expected midpoint values are FR 1225/1675, FL 1800/1360, RR 1340/1800, and RL 1695/1205. Keep FrontAxis endpoint/interpolation assertions passing.
3. Update servo_service_tests.c for every requested raw PWM boundary and angle boundary. Verify enable and neutral writes use FR 1450, FL 1580, FrontAxis 1745, RR 1570, and RL 1450. Do not change service semantics.
4. Compile and run only the affected Firmware tests against the unchanged production descriptor table. Capture the expected failures as the red checkpoint.
5. Commit only the Firmware test changes with a test checkpoint commit.

## Task 3: Implement and verify Firmware descriptors

1. Change only the five descriptor rows in servo_descriptor.c: apply the four final paddle calibration triples and ascending raw PWM bounds, and preserve the exact FrontAxis contract. Do not add reverse flags or servo-ID branches.
2. Run the focused calibration, descriptor, and service tests. Confirm exact mappings, decreasing left-side mappings, all boundary cases, neutral writes, and unchanged FrontAxis behavior.
3. Run the complete existing Firmware regression suite, including Leak sensor and telemetry-policy tests, and any current API compile-contract checks.
4. Commit only the Firmware production descriptor change.

## Task 4: Add failing Console parity tests

1. Update servo_descriptor_tests.cpp to assert the exact same five descriptors and independent ascending raw PWM bounds as Firmware.
2. Update robot_controller_tests.cpp to cover the four paddle raw and angle boundaries, exact FrontAxis regression payloads, descriptor-driven neutral behavior, and left-side reversed calibration metadata without changing wire construction. Preserve the existing ACK sequencing and one-active-exchange test harness.
3. Run the focused Console tests against the unchanged Console descriptor table and capture deterministic red failures when Qt6 is available. If Qt6 is unavailable, record configure/build/CTest as Pending and retain source-level test evidence without claiming a pass.
4. Commit only the Console test changes.

## Task 5: Implement and verify Console descriptors

1. Mirror the Firmware descriptor values in ServoDescriptor.cpp. Keep raw PWM bounds ascending even where calibration endpoint pulses decrease.
2. Run focused and then full Console CTest/build with the known-good MinGW/Qt runtime ordering when Qt6 is available. Verify descriptor parity and controller boundary/payload assertions.
3. Confirm MainWindow and layout files are unchanged and descriptor-driven controls derive the new bounds without UI edits.
4. Commit only the Console descriptor change.

## Task 6: Synchronize documentation

1. Record the common logical convention, exact five-servo table, neutral values, and the distinction between directional calibration endpoints and ascending raw PWM bounds in the active Firmware, Console, and hardware handoff documentation.
2. Label evidence exactly: FR 1450/1900 Bench Measured and 1000 Symmetry-Derived / User Accepted; FL 1580/1140 Bench Measured and 2020 Symmetry-Derived / User Accepted; RR and RL full triples Bench Hardware Verified; FrontAxis retains the PR #13 evidence. Do not relabel symmetry-derived endpoints as hardware verified.
3. Add only architecture documentation for the future data flow Motion Command -> Gait/CPG Generator -> Logical Joint Target -> ServoService set_angle -> Servo Calibration -> PWM. Do not implement any of those future components.
4. Preserve historical PR #8 references that describe the old five-servo bring-up and do not alter unrelated handoff sections.
5. Commit the documentation-only checkpoint.

## Task 7: Final gates and independent review

1. Run Firmware full regressions, focused servo tests, Console CTest/build if available, Firmware/Console parity checks, header self-sufficiency and standard-header audits, warnings checks, git diff --check, and forbidden-scope scans.
2. Check for arm-none-eabi-gcc. Run ARM syntax/build verification only if the toolchain is actually available; otherwise report ARM Build as Pending and never infer hardware verification.
3. Request an independent read-only code review of the final diff. If it finds an actionable issue, fix it in scope and rerun the affected gates before proceeding.
4. Verify the final changed-file list contains no APC, gait, CPG, resource-source, UI-layout, protocol-wire, or unrelated production changes.

## Task 8: Push PR #13 and stop for external review

1. Commit any final in-scope verification/documentation adjustment required by review, then push normally to feature/servo-calibration-depth-limits. Do not create a new PR, merge, rebase, reset, or force-push.
2. Verify local HEAD equals the remote branch HEAD, the worktree is clean, and PR #13 remains OPEN and unmerged against main.
3. Report the previous HEAD, new commit SHA, remote SHA, exact calibration and neutral tables, midpoint and boundary test evidence, regression results, evidence labels, ARM status, and PR #13 state.
4. End with READY FOR EXTERNAL GITHUB REVIEW and stop.

