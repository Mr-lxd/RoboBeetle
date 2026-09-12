# Servo Calibration and FrontAxis Limits Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Update the Firmware and Console five-servo descriptors so FrontAxis uses its bench-calibrated 1060/1745/2430 us, -90/+90 degree contract and both rear servos expose only the approved temporary 820–2220 us PWM exploration window, with tests and documentation proving the boundaries and scope.

**Architecture:** Keep the existing independent Firmware and Console descriptor tables as the source of command limits and calibration metadata. Reuse the existing pure-C angle-to-pulse interpolation and descriptor-driven Console behavior; do not add a profile layer, UI layout changes, new protocol fields, or any transport/control changes. Implement in small test-first commits: focused Firmware tests, Firmware table, focused Console tests, Console table, documentation, then full verification and review.

**Tech Stack:** C11 Firmware host tests with GCC, STM32CubeMX-generated C/HAL sources, C++20/Qt6 Console tests when Qt6 is available, CMake/Ninja, GitHub PR workflow, Markdown documentation.

---

## File map and write ownership

- Modify `RoboBeetleFirmware/tests/servo_descriptor_tests.c`: Firmware descriptor contract assertions.
- Modify `RoboBeetleFirmware/tests/servo_calibration_tests.c`: FrontAxis interpolation and endpoint/rounding assertions.
- Modify `RoboBeetleFirmware/tests/servo_service_tests.c`: Firmware PWM/angle boundary and neutral-write assertions.
- Modify `RoboBeetleFirmware/Core/Servo/servo_descriptor.c`: only the FrontAxis and rear command-table values.
- Modify `RoboBeetleConsole/tests/servo_descriptor_tests.cpp`: Console descriptor parity assertions.
- Modify `RoboBeetleConsole/tests/robot_controller_tests.cpp`: descriptor-driven FrontAxis/rear command acceptance and payload assertions.
- Modify `RoboBeetleConsole/src/robot/ServoDescriptor.cpp`: only the matching Console descriptor values.
- Modify the active actuator-calibration sections of `RoboBeetleFirmware/README.md`, `RoboBeetleConsole/README.md`, `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`, and `docs/engineering-lessons.md`: record the bench calibration, temporary rear window, pending hardware status, and distinction from the separate ROVMAKER depth sensor.
- Do not modify any APC feasibility/timing file, `LinkProfile`, USART, Protocol V2 codec or payload, Safety, Leak, JY901S, RPi, ROS2, depth-sensor transport, Qt widget/layout source, `.ioc`, generated HAL, or CMake source list unless a verification result proves a directly required Servo-only change.

### Task 1: Add failing Firmware contract tests

**Files:**
- Modify: `RoboBeetleFirmware/tests/servo_descriptor_tests.c`
- Modify: `RoboBeetleFirmware/tests/servo_calibration_tests.c`
- Modify: `RoboBeetleFirmware/tests/servo_service_tests.c`

- [ ] **Step 1: Change descriptor expectations to the frozen contract.** In `servo_descriptor_tests.c`, keep all IDs, masks, timers, channels, FrontRight, and FrontLeft expectations unchanged. Replace only the FrontAxis expected row with:

```c
expect(table[SERVO_ID_FRONT_AXIS].angle_supported,
       "FrontAxis angle must be supported");
expect_calibration(&table[SERVO_ID_FRONT_AXIS],
                   1060U, 1745U, 2430U, -9000, 9000,
                   1060U, 2430U, -9000, 9000);
```

Replace the command-range arguments for both rear rows with `820U, 2220U, -4500, 4500`; leave their `520U, 1520U, 2520U, -9000, 9000` calibration arguments intact.

- [ ] **Step 2: Add FrontAxis interpolation assertions.** In `servo_calibration_tests.c`, after the existing FrontRight checks, select `&table[SERVO_ID_FRONT_AXIS].calibration` and assert the frozen fields and exact mappings:

```c
calibration = &table[SERVO_ID_FRONT_AXIS].calibration;
expect(calibration->min_pulse_us == 1060U,
       "FrontAxis minimum pulse calibration differs");
expect(calibration->neutral_pulse_us == 1745U,
       "FrontAxis neutral pulse calibration differs");
expect(calibration->max_pulse_us == 2430U,
       "FrontAxis maximum pulse calibration differs");
expect(calibration->min_angle_cdeg == -9000,
       "FrontAxis minimum angle calibration differs");
expect(calibration->max_angle_cdeg == 9000,
       "FrontAxis maximum angle calibration differs");
expect(servo_calibration_angle_to_pulse(calibration, -9000) == 1060U,
       "FrontAxis -90 degree mapping differs");
expect(servo_calibration_angle_to_pulse(calibration, -4500) == 1403U,
       "FrontAxis -45 degree mapping differs");
expect(servo_calibration_angle_to_pulse(calibration, 0) == 1745U,
       "FrontAxis zero degree mapping differs");
expect(servo_calibration_angle_to_pulse(calibration, 4500) == 2087U,
       "FrontAxis +45 degree mapping differs");
expect(servo_calibration_angle_to_pulse(calibration, 9000) == 2430U,
       "FrontAxis +90 degree mapping differs");
```

- [ ] **Step 3: Replace Firmware service boundaries with exact edge cases.** In `test_provisional_calibration_and_bounds()` in `servo_service_tests.c`, retain the existing FrontRight checks and change the rear cases to `819/reject`, `820/accept`, `2220/accept`, and `2221/reject`. Replace the FrontAxis PWM cases with `1059/reject`, `1060/accept`, `1745/accept`, `2430/accept`, and `2431/reject`. Replace the old unsupported-angle assertion with these calls and expected results:

```c
expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, -9001) ==
           SERVO_SERVICE_RESULT_OUT_OF_RANGE,
       "FrontAxis below -90 degrees must be rejected");
expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, -9000) ==
           SERVO_SERVICE_RESULT_OK,
       "FrontAxis -90 degrees must be accepted");
expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, 0) ==
           SERVO_SERVICE_RESULT_OK,
       "FrontAxis zero degrees must be accepted");
expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, 9000) ==
           SERVO_SERVICE_RESULT_OK,
       "FrontAxis +90 degrees must be accepted");
expect(servo_service_set_angle(&service, SERVO_ID_FRONT_AXIS, 9001) ==
           SERVO_SERVICE_RESULT_OUT_OF_RANGE,
       "FrontAxis above +90 degrees must be rejected");
```

Update the expected rear success/failure messages to describe the temporary PWM window and keep rear angle checks at ±4500 cdeg. Add a direct assertion that the FrontAxis enable and neutral events write `1745U`, while leaving the existing descriptor-driven checks for the other four servos intact.

- [ ] **Step 4: Run only the affected Firmware tests against the unchanged descriptor implementation.** Use the repository's established host GCC command with `-std=c11 -Wall -Wextra -Werror`, the production Servo sources, and the three test sources. Run `servo_descriptor_tests`, `servo_calibration_tests`, and `servo_service_tests`.

Expected result: all three executables fail because the baseline tables still expose FrontAxis `500–2500`/disabled/`1500` and rear `1020–2020`. Do not change production code before capturing this red result.

- [ ] **Step 5: Commit the red test checkpoint.**

```powershell
git add RoboBeetleFirmware/tests/servo_descriptor_tests.c RoboBeetleFirmware/tests/servo_calibration_tests.c RoboBeetleFirmware/tests/servo_service_tests.c
git commit -m "test: specify servo calibration and limit boundaries"
```

### Task 2: Implement the minimal Firmware descriptor update

**Files:**
- Modify: `RoboBeetleFirmware/Core/Servo/servo_descriptor.c`

- [ ] **Step 1: Change only the two approved descriptor rows.** Set the FrontAxis calibration and command fields to:

```c
.calibration = {
    .min_pulse_us = 1060U,
    .neutral_pulse_us = 1745U,
    .max_pulse_us = 2430U,
    .min_angle_cdeg = -9000,
    .max_angle_cdeg = 9000,
},
.command_min_pulse_us = 1060U,
.command_max_pulse_us = 2430U,
.command_min_angle_cdeg = -9000,
.command_max_angle_cdeg = 9000,
.angle_supported = true,
```

Set both rear rows' `.command_min_pulse_us` to `820U` and `.command_max_pulse_us` to `2220U`. Do not change rear calibration fields or angle command fields. Do not change FrontRight/FrontLeft.

- [ ] **Step 2: Run the three focused Firmware tests.** Expected: PASS, including exact `1403U`/`2087U` truncation-preserving mappings and FrontAxis neutral writes.

- [ ] **Step 3: Run the complete Firmware host regression gate.** Compile and run all 17 current executables listed in `RoboBeetleFirmware/README.md`, plus the separate `app_main_jy901s_api_tests.c` compile-contract check. Expected: PASS for every existing regression; no test may be removed or narrowed.

- [ ] **Step 4: Commit the Firmware implementation checkpoint.**

```powershell
git add RoboBeetleFirmware/Core/Servo/servo_descriptor.c
git commit -m "feat: apply bench FrontAxis and temporary rear limits"
```

### Task 3: Add failing Console parity and controller tests

**Files:**
- Modify: `RoboBeetleConsole/tests/servo_descriptor_tests.cpp`
- Modify: `RoboBeetleConsole/tests/robot_controller_tests.cpp`

- [ ] **Step 1: Update the Console descriptor expectation helper calls.** Replace only FrontAxis expected values with `angleSupported=true`, `calibrationPending=false`, electrical `1060/1745/2430`, calibration angles `-9000/9000`, command PWM `1060/2430`, and command angles `-9000/9000`. Change both rear command PWM expectations to `820/2220`; retain their `520/1520/2520` and ±4500 angle expectations. Keep FrontRight/FrontLeft values unchanged.

- [ ] **Step 2: Replace the PWM-only FrontAxis controller test.** Rename the test to describe calibrated FrontAxis behavior and assert:

```cpp
const rb::ServoDescriptor *descriptor = rb::servoDescriptor(rb::ServoId::FrontAxis);
expect(descriptor != nullptr && descriptor->angleSupported,
       "FrontAxis must support calibrated angles");
expect(descriptor != nullptr && !descriptor->calibrationPending,
       "FrontAxis calibration must not remain pending");
expect(!controller.setServoPwm(rb::ServoId::FrontAxis, 1059),
       "FrontAxis PWM below 1060 us must be rejected");
expect(controller.setServoPwm(rb::ServoId::FrontAxis, 1060),
       "FrontAxis PWM 1060 us must be accepted");
expect(controller.setServoPwm(rb::ServoId::FrontAxis, 2430),
       "FrontAxis PWM 2430 us must be accepted");
expect(!controller.setServoPwm(rb::ServoId::FrontAxis, 2431),
       "FrontAxis PWM above 2430 us must be rejected");
expect(!controller.setServoAngle(rb::ServoId::FrontAxis, -9001),
       "FrontAxis angle below -90 degrees must be rejected");
expect(controller.setServoAngle(rb::ServoId::FrontAxis, -9000),
       "FrontAxis -90 degrees must be accepted");
expect(controller.setServoAngle(rb::ServoId::FrontAxis, 0),
       "FrontAxis zero degrees must be accepted");
expect(controller.setServoAngle(rb::ServoId::FrontAxis, 9000),
       "FrontAxis +90 degrees must be accepted");
expect(!controller.setServoAngle(rb::ServoId::FrontAxis, 9001),
       "FrontAxis angle above +90 degrees must be rejected");
```

Assert the endpoint payloads are `02002404` for `-9000` mapped to `1060` us and `02007a09` for `+9000` mapped to `2430` us (semantic ServoId byte `2`, little-endian PWM). Use the existing ACK helper between accepted commands so the one-active-exchange rule in the test harness is preserved. Add a neutral request assertion that the wire command remains the existing mask-based `Neutral` message; the `1745 us` value is verified by Firmware service tests, since Protocol V2 Neutral carries a mask rather than a pulse.

- [ ] **Step 3: Update the shared semantic boundary table.** Set FrontAxis to `{1060, 2430, -9000, 9000, true}` and both rear rows to `{820, 2220, -4500, 4500, true}`. Adjust the accepted-write count for FrontAxis from the old PWM-only two writes to five accepted commands (two PWM endpoints plus three accepted angles), matching the existing controller test structure.

- [ ] **Step 4: Run the affected Console tests against the unchanged Console descriptor table.** If Qt6 is unavailable, retain the exact configure failure as evidence and do not label the Console gate as a test failure or pass. If a Qt6 toolchain is present, run only the descriptor and controller executables.

Expected with an available Qt6 build: red failures for the old FrontAxis and rear descriptor values. No production Console source is changed before this red checkpoint.

- [ ] **Step 5: Commit the red Console test checkpoint.**

```powershell
git add RoboBeetleConsole/tests/servo_descriptor_tests.cpp RoboBeetleConsole/tests/robot_controller_tests.cpp
git commit -m "test: specify Console servo calibration contract"
```

### Task 4: Implement the matching Console descriptor update

**Files:**
- Modify: `RoboBeetleConsole/src/robot/ServoDescriptor.cpp`

- [ ] **Step 1: Mirror the Firmware descriptor values.** Change only the FrontAxis entry to `1060, 1745, 2430`, calibration angles `-9000, 9000`, command PWM `1060, 2430`, command angles `-9000, 9000`, `angleSupported=true`, and `calibrationPending=false`. Change only the rear command PWM values to `820, 2220`; leave rear electrical/calibration values and ±4500 command angles unchanged.

- [ ] **Step 2: Run the Console descriptor/controller tests if Qt6 is available.** Expected: PASS, including the descriptor parity and endpoint command payload assertions. If Qt6 remains unavailable, record Console configure/build/CTest as Pending and use source-level parity plus Firmware tests as the available evidence; do not fabricate a Console PASS.

- [ ] **Step 3: Check for unintended UI changes.** Confirm `RoboBeetleConsole/src/ui/MainWindow.cpp` and layout files are unchanged. The existing descriptor-driven controls must supply the new range/angle availability without a widget or layout edit.

- [ ] **Step 4: Commit the Console implementation checkpoint.**

```powershell
git add RoboBeetleConsole/src/robot/ServoDescriptor.cpp
git commit -m "feat: mirror calibrated servo descriptors in Console"
```

### Task 5: Synchronize active documentation without rewriting history

**Files:**
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `RoboBeetleConsole/README.md`
- Modify: `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`
- Modify: `docs/engineering-lessons.md`

- [ ] **Step 1: Record the current actuator contract in the active Firmware and Console sections.** State that FrontAxis/Depth actuator bench calibration is `1060 us = -90°`, `1745 us = 0°`, `2430 us = +90°`, command PWM `1060–2430 us`, angle command `-90°…+90°`, and neutral `1745 us`. State that RearRight/RearLeft expose temporary command PWM `820–2220 us` while retaining electrical calibration `520/1520/2520 us` and software angle range ±45°; this window is not final rear ±45° endpoint calibration.

- [ ] **Step 2: Preserve evidence labels.** Mark the supplied actuator measurement as `Bench Hardware Calibrated`. Mark the new Firmware image, target ARM build/programming, and physical regression as Pending unless independently evidenced. Keep the separate ROVMAKER depth-sensor communication/calibration wording distinct from the FrontAxis actuator.

- [ ] **Step 3: Add the concise engineering lesson.** Include the rule that host/compiler success does not establish target portability or hardware verification, and that direct descriptor values must be kept in parity across Firmware and Console. Do not alter unrelated APC, Protocol, Leak, IMU, Depth transport, or historical evidence sections.

- [ ] **Step 4: Commit documentation separately.**

```powershell
git add RoboBeetleFirmware/README.md RoboBeetleConsole/README.md ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md docs/engineering-lessons.md
git commit -m "docs: record servo bench calibration boundaries"
```

### Task 6: Run quality audits and the complete verification matrix

**Files:**
- Read-only audit: all changed C/C++ translation units and public/internal headers.
- Read-only audit: `git diff --name-only`, `.ioc`, CMake, generated HAL, and APC paths.

- [ ] **Step 1: Run the focused Firmware Servo tests and all 17 Firmware regressions again.** Use the documented `-Wall -Wextra -Werror` host build. Confirm the Leak, IMU, Depth, transport, scheduler, and app integration tests remain included. Run the separate `app_main_jy901s_api_tests.c` compile-contract check.

- [ ] **Step 2: Run the Console configure/build/CTest gate.** Configure with the environment's Qt6 CMake prefix when available, build, then run CTest. If Qt6 is not available, record `Console configure/build/CTest: Pending — Qt6 package not found` and retain the exact configure output.

- [ ] **Step 3: Run generated-style/HAL and ARM availability checks.** Verify the `.ioc`, generated HAL, MSP/IRQ, and CMake source lists are unchanged and consistent. Probe `arm-none-eabi-gcc --version`; if absent, record `ARM Build: Pending — toolchain unavailable` and do not claim a target build.

- [ ] **Step 4: Audit direct standard-header dependencies.** For each changed C/C++ source/header, confirm direct includes for every used standard symbol: `<stdbool.h>`, `<stdint.h>`, `<stddef.h>`, `<string.h>`, `<stdio.h>`, `<stdlib.h>`, and `<math.h>` as applicable. Fix only a definite missing direct include in a changed Servo file, then rerun the affected gate and record the extra file in the report.

- [ ] **Step 5: Check scope and whitespace.** Run:

```powershell
git diff --check
git diff --name-only origin/main...HEAD
git status --short
```

The changed-file list must contain only approved Servo tests/descriptors/docs plus the committed spec/plan. It must contain no APC feasibility/timing file and no APC source change. `daplink.cfg` and unrelated pre-existing files outside this worktree are not to be modified.

- [ ] **Step 6: Commit any narrowly required audit repair separately.** Use a message naming the exact portability or test correction, rerun its gate, and do not fold unrelated changes into the feature.

### Task 7: Independent internal review, push, and PR

**Files:**
- Read-only review of the complete branch diff and verification evidence.

- [ ] **Step 1: Run an independent spec-compliance review.** The reviewer must verify every frozen value, exact inclusive/exclusive boundary, FrontAxis sign, rear temporary-window semantics, unchanged FrontRight/FrontLeft behavior, no UI layout change, no APC/transport/protocol changes, and evidence labels. Resolve every actionable finding before proceeding.

- [ ] **Step 2: Run an independent code-quality review.** Check integer mapping reuse/rounding preservation, descriptor parity, test isolation, direct headers, warning cleanliness, documentation chronology, and accidental scope changes. Resolve findings and rerun affected tests.

- [ ] **Step 3: Confirm branch and remote state.** Verify branch is `feature/servo-calibration-depth-limits`, base is current `origin/main`, and the only pre-existing worktree state is preserved. Push with:

```powershell
git push -u origin feature/servo-calibration-depth-limits
```

- [ ] **Step 4: Open exactly one PR against `main` without merging.** Use the repository's existing GitHub CLI/remote workflow. The PR body must list Firmware/Console changes, tests, `Console configure/build: Pending` and `ARM Build: Pending` when applicable, `Program Verify: Pending`, `Hardware Verified: Pending`, rear final calibration Pending, and External GitHub Review Pending. Do not include APC feasibility work.

- [ ] **Step 5: Verify the PR remotely.** Query the PR head/base, changed-file list, and open state after push. Do not treat a successful push message alone as proof that the PR exists.

Final state: implementation reviewed internally, feature branch pushed, one PR open against `main`, not merged, waiting for ChatGPT's External GitHub Review.
