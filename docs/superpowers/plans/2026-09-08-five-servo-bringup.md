# Five-Servo Bring-Up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring all five semantic servo channels through the existing Protocol V2, STM32 Firmware service/driver, and Qt Console while preserving the current APC220, safety, and wire behavior.

**Architecture:** Firmware will use a pure-C, HAL-independent five-entry descriptor table containing IDs, masks, capabilities, calibration, mechanical envelopes, and abstract timer/channel selectors. A HAL-specific driver binding table will map those selectors to TIM3/TIM4 handles and HAL channels. Qt will maintain an independent five-entry semantic descriptor collection and generate five UI cards from it; the two tables will be checked by explicit tests rather than coupled at build time.

**Tech Stack:** C11 STM32 HAL/CubeMX/CMake; C++20 Qt 6 Core/Widgets/SerialPort; existing manual Firmware pure-C regressions; Qt CTest targets.

---

## Files and responsibilities

Firmware files to create or modify:

- Create `RoboBeetleFirmware/Core/Servo/servo_descriptor.h` and `.c`: five semantic IDs, masks, abstract timer/channel enums, capabilities, electrical calibration, mechanical command envelopes, compile-time constants, and bounded lookup APIs. No HAL include or HAL constant is allowed.
- Modify `RoboBeetleFirmware/Core/Servo/servo_calibration.h/.c`: make angle-to-pulse conversion operate on a supplied calibration record; remove Servo1-only global mapping while retaining the pure integer arithmetic contract.
- Modify `RoboBeetleFirmware/Core/Servo/servo_service.h/.c`: consume the descriptor table, validate five-bit masks/IDs, enforce mechanical envelopes, implement all-or-nothing multi-bit Enable rollback, best-effort Disable/Disable All, and per-servo Neutral/PWM/Angle behavior.
- Modify `RoboBeetleFirmware/Core/Servo/servo_driver_stm32.h/.c`: bind abstract descriptor timer/channel selectors to TIM3/TIM4 handles and HAL channels; perform lookup-based write/start/stop operations and reject invalid/unbound IDs.
- Modify `RoboBeetleFirmware/Core/App/app_main.h/.c`: accept TIM3 and TIM4 handles and pass the runtime bindings to the existing service/dispatcher path.
- Modify `RoboBeetleFirmware/Core/Src/main.c`, `Core/Src/stm32f4xx_hal_msp.c`, `Core/Inc/main.h` only as required for TIM3 CH1/2/3 and TIM4 CH1/2 initialization and GPIO alternate-function setup.
- Modify `RoboBeetleFirmware/RoboBeetleFirmware.ioc`: record the same five pins/channels and timer parameters without changing USART1, SWD, DBG_LED, or safety-related configuration.
- Modify `RoboBeetleFirmware/CMakeLists.txt`: add only the descriptor source/header entries and any required include path; leave generated CubeMX CMake untouched.
- Create `RoboBeetleFirmware/tests/servo_descriptor_tests.c`; modify `servo_calibration_tests.c`, `servo_service_tests.c`, and `protocol_dispatcher_tests.c` for the five-ID contract and failure cases.

Console files to create or modify:

- Create `RoboBeetleConsole/src/robot/ServoDescriptor.h/.cpp`: independent semantic enum/table with labels, capability flags, electrical calibration, and mechanical command ranges.
- Modify `RoboBeetleConsole/src/robot/RobotCommand.h`: define the five frozen enum values, `Servo1` deprecated alias to `FrontRight`, no `Servo2`, mask/count constants, and compile-time assertions.
- Modify `RoboBeetleConsole/src/robot/RobotController.h/.cpp`: replace single provisional ranges and two-index assumptions with descriptor lookup/count logic while preserving APC220 scheduler behavior and protocol payload encoding.
- Modify `RoboBeetleConsole/src/ui/MainWindow.h/.cpp`: generate five semantic cards from the descriptor collection, maintain independent state/status widgets, keep FrontAxis angle disabled, and retain only global Disable All.
- Modify `RoboBeetleConsole/CMakeLists.txt` to include the descriptor source/header.
- Modify `RoboBeetleConsole/tests/robot_controller_tests.cpp` and, if needed, `tests/protocol_tests.cpp` for all five IDs, ranges, payloads, and no regressions.

Documentation files:

- Modify `RoboBeetleFirmware/README.md`, `RoboBeetleConsole/README.md`, `RoboBeetleConsole/docs/protocol.md`, `docs/engineering-lessons.md`, and `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md` only in relevant sections.

## Task 1: Add failing Firmware descriptor and calibration tests

**Files:** Create `RoboBeetleFirmware/tests/servo_descriptor_tests.c`; modify `RoboBeetleFirmware/tests/servo_calibration_tests.c`.

- [ ] **Step 1: Write descriptor table assertions.** Add a pure-C test that includes `servo_descriptor.h`, uses `_Static_assert` for IDs `0..4` and mask `0x001F`, checks `servo_descriptor_count()==5`, and checks each row’s semantic ID, mask, abstract timer (`TIM3` for IDs 0–2, `TIM4` for IDs 3–4), abstract channel (1/2/3 and 1/2), support/angle capability, and exact calibration/mechanical values.
- [ ] **Step 2: Write calibration boundary assertions.** Extend the calibration test to call the generic mapping helper with the descriptor’s calibration records and assert SAVOX `-4500/0/+4500` maps to `1050/1500/1950`, GDW maps to `1020/1520/2020`, and FrontAxis is never passed to angle mapping because its descriptor marks `angle_supported=false`.
- [ ] **Step 3: Run the new tests before production changes.** Compile the pure modules and tests with the repository’s existing host-GCC flags (`-std=c11 -Wall -Wextra -Werror`) and record the expected failure because the descriptor API and generic mapping do not yet exist.

## Task 2: Implement the pure-C descriptor and calibration layer

**Files:** Create `RoboBeetleFirmware/Core/Servo/servo_descriptor.h/.c`; modify `servo_calibration.h/.c`; modify `RoboBeetleFirmware/CMakeLists.txt`.

- [ ] **Step 1: Define HAL-independent types and table.** Define `servo_id_t` values `SERVO_ID_FRONT_RIGHT=0U`, `SERVO_ID_FRONT_LEFT=1U`, `SERVO_ID_FRONT_AXIS=2U`, `SERVO_ID_REAR_RIGHT=3U`, `SERVO_ID_REAR_LEFT=4U`, `SERVO_ID_COUNT=5U`; define `servo_timer_id_t` and `servo_channel_id_t` abstract enums; define `servo_descriptor_t` with calibration, mechanical limits, support flags, timer selector, and abstract channel. Do not include `stm32f4xx_hal.h` or use `TIM_CHANNEL_*`.
- [ ] **Step 2: Populate immutable metadata.** Add five descriptors with SAVOX electrical `1000/1500/2000 us`, angle `-5000/+5000 cdeg`, mechanical `1050..1950 us` and `-4500..+4500 cdeg`; GDW electrical `520/1520/2520 us`, angle `-9000/+9000 cdeg`, mechanical `1020..2020 us` and `-4500..+4500 cdeg`; FrontAxis electrical/command `1450..1550 us`, neutral candidate `1500 us`, `angle_supported=false`, TIM3 abstract channel 3. Set every row supported and aggregate mask `0x001F`.
- [ ] **Step 3: Add bounded lookup APIs and compile-time checks.** Implement `servo_descriptor_table`, `servo_descriptor_count`, `servo_descriptor_for_id`, and `servo_descriptor_supported_mask`; reject IDs outside `0..4` by returning `NULL`; add `_Static_assert` checks for count/mask relationship.
- [ ] **Step 4: Generalize calibration arithmetic.** Change the mapping signature to accept `const servo_calibration_t *` and use that record’s negative/positive angle limits and pulse endpoints; guard invalid/null/zero-span inputs by returning the neutral pulse. Remove the Servo1-only static calibration and one-argument mapping implementation.
- [ ] **Step 5: Build the pure layer and rerun Task 1 tests.** Use the exact host-GCC command from Step 1 with `servo_descriptor.c` and `servo_calibration.c`; expect all descriptor/calibration tests to pass.
- [ ] **Step 6: Commit the pure layer.** Run `git diff --check`, then commit `feat(firmware): add semantic five-servo descriptors`.

## Task 3: Make Firmware service table-driven with atomic Enable

**Files:** Modify `RoboBeetleFirmware/Core/Servo/servo_service.h/.c`; modify `tests/servo_service_tests.c`.

- [ ] **Step 1: Extend fake-driver tests for five IDs.** Add fake-driver event arrays indexed by semantic ID, configure an optional `start_fail_mask`, and write tests that enable each ID independently, observe the correct ID/pulse, verify independent enabled bits, reject zero/high/unsupported masks, and reject invalid IDs without invoking fake driver operations.
- [ ] **Step 2: Add range and capability tests.** Assert SAVOX PWM `1049/1951` rejects and `1050/1950` accepts; SAVOX angles `-4500/+4500` accept and `-4501/+4501` reject. Assert GDW PWM `1019/2021` rejects and `1020/2020` accepts; GDW angles `-4500/+4500` accept and `-4501/+4501` reject. Assert FrontAxis PWM `1449/1551` rejects and `1450/1550` accepts and every SetAngle returns UnsupportedServo.
- [ ] **Step 3: Add all-or-nothing Enable regression.** Request a multi-bit Enable with one fake start failure; assert the result is HardwareFailure, enabled mask equals the pre-call mask, all channels started earlier in this call were stopped, and no partial arm remains. Add a success case for all five bits and verify neutral writes precede starts.
- [ ] **Step 4: Add Disable/Neutral/Disable All tests.** Enable all five, verify individual Disable clears only its bit, Neutral requires all requested bits enabled and writes every requested neutral, and Disable All stops all five and clears the mask.
- [ ] **Step 5: Implement descriptor lookup and validation.** Make `servo_service` use `servo_descriptor_for_id`/`servo_descriptor_table` instead of Servo1 constants. Validate masks against the descriptor supported mask and reject invalid IDs before driver calls.
- [ ] **Step 6: Implement atomic multi-bit Enable.** Save the original enabled mask, write neutral/start each requested descriptor, track only newly started bits, stop those bits on any start failure, restore the original mask, and return HardwareFailure. On success OR the requested mask into enabled state.
- [ ] **Step 7: Implement per-servo command policy.** Require enabled state, enforce mechanical PWM/angle limits, use the descriptor calibration for mapping, and return UnsupportedServo for FrontAxis SetAngle. Keep Disable/Disable All best-effort stop semantics and iterate all five descriptors.
- [ ] **Step 8: Run service/calibration/descriptor tests and commit.** Compile with `-std=c11 -Wall -Wextra -Werror`; require zero failures; commit `feat(firmware): make servo service descriptor-driven`.

## Task 4: Bind abstract channels to STM32 timers

**Files:** Modify `RoboBeetleFirmware/Core/Servo/servo_driver_stm32.h/.c`; modify `Core/App/app_main.h/.c`; modify `Core/Src/main.c`; modify `Core/Src/stm32f4xx_hal_msp.c`; modify `Core/Inc/main.h`; modify `RoboBeetleFirmware.ioc`.

- [ ] **Step 1: Add driver binding tests at the abstraction boundary.** Extend descriptor tests (or add a small driver binding test that builds against a HAL stub) to assert ID 0/1/2 resolve to TIM3 abstract channels 1/2/3 and ID 3/4 resolve to TIM4 abstract channels 1/2; invalid IDs return no binding. The test must not require a live MCU.
- [ ] **Step 2: Define runtime binding storage.** Add a five-entry `servo_driver_stm32_binding_t` array containing the pure descriptor pointer, bound `TIM_HandleTypeDef *`, and mapped HAL channel. `servo_driver_stm32_init` accepts TIM3/TIM4 handles and fills bindings by iterating the descriptor table.
- [ ] **Step 3: Map enums to HAL constants in one adapter.** Implement bounded timer/channel mapping in `servo_driver_stm32.c`; use `TIM_CHANNEL_1/2/3` only there. `write`, `start`, and `stop` look up the binding by ID, reject missing/unsupported/null handles, and never contain five-ID if/else policy.
- [ ] **Step 4: Extend App/Main handle plumbing.** Change `app_main_init` to accept `TIM_HandleTypeDef *tim3` and `TIM_HandleTypeDef *tim4`; initialize the driver with both and leave dispatcher, safety, UART, and protocol code unchanged.
- [ ] **Step 5: Add TIM3/TIM4 CubeMX initialization.** Add `htim4` and `MX_TIM4_Init` to `main.c`; configure TIM3 CH1/CH2/CH3 and TIM4 CH1/CH2 with prescaler 15, period 3002, 1 us ticks, and startup pulses matching descriptor neutrals (1500 for front channels, 1520 for rear channels). Pass both handles to `app_main_init`.
- [ ] **Step 6: Add required MSP/pin setup.** Keep USART1 PA9/PA10 and SWD/DBG_LED untouched. Add TIM3 PA7/PB0 AF2 and TIM4 PD12/PD13 AF2 alongside existing PA6; enable the corresponding GPIO/TIM clocks in MSP post-init and de-init paths.
- [ ] **Step 7: Update the `.ioc` consistently.** Add TIM4, PA7, PB0, PD12, and PD13 signal entries and both timers’ channel/prescaler/period/pulse settings; ensure the IOC still records USART1 9600 8-N-1, SWD, and DBG_LED. Do not regenerate unrelated files.
- [ ] **Step 8: Run Firmware configure/build and all pure-C regressions.** Use `cmake --preset Debug` and `cmake --build --preset Debug`; compile/run protocol golden vectors, ring buffer, safety, descriptor, calibration, service, and dispatcher tests. Commit `feat(firmware): bind five servo channels to TIM3 and TIM4` after `git diff --check`.

## Task 5: Add failing Qt semantic descriptor/controller tests

**Files:** Create `RoboBeetleConsole/src/robot/ServoDescriptor.h/.cpp`; modify `RobotCommand.h`; modify `tests/robot_controller_tests.cpp`.

- [ ] **Step 1: Add compile-time semantic assertions.** Assert `FrontRight=0`, `FrontLeft=1`, `FrontAxis=2`, `RearRight=3`, `RearLeft=4`, supported mask `0x001F`, and `Servo1` aliases `FrontRight`; do not define `Servo2`.
- [ ] **Step 2: Add descriptor table tests.** Assert five rows, semantic labels, support/capability flags, SAVOX/GDW/FrontAxis calibration and mechanical limits, and FrontAxis `angleSupported=false`.
- [ ] **Step 3: Add controller behavior tests.** Exercise all five supported IDs, per-servo Enable and payload IDs, independent enabled bits, SAVOX/GDW boundary acceptance/rejection, FrontAxis PWM clamp and Angle rejection, Disable/Disable All, and absence of writes for invalid IDs. Keep existing APC220/DirectUart tests unchanged except for semantic IDs and ranges.
- [ ] **Step 4: Run the Qt tests before implementation.** Configure/build the Console test target and run `robot_controller_tests`; record the expected compile/test failures for the missing descriptor APIs and five-servo behavior.

## Task 6: Implement Qt semantic descriptors and controller integration

**Files:** Modify `RoboBeetleConsole/src/robot/RobotCommand.h`; create/modify `ServoDescriptor.h/.cpp`; modify `RobotController.h/.cpp`; modify `RoboBeetleConsole/CMakeLists.txt`.

- [ ] **Step 1: Define semantic enum and compatibility policy.** Add the five enum values and mark only `Servo1 = FrontRight` as a deprecated historical alias. Use `kServoCount=5`, `kSupportedServoMask=0x001F`, and compile-time assertions; remove all `Servo2` declarations.
- [ ] **Step 2: Populate the independent Qt table.** Store semantic/display names, hardware labels, support/angle flags, electrical calibration, and mechanical command limits matching the Firmware table. Mark FrontAxis as PWM Bring-up Only/Calibration Pending with no angle capability.
- [ ] **Step 3: Refactor configuration and lookup helpers.** Make the default controller supported mask five bits; add bounded descriptor lookup helpers and use them for support checks, range checks, logging, and masks. Remove use of one global PWM/angle range.
- [ ] **Step 4: Preserve Protocol V2 payloads.** Keep Enable/Disable/Neutral uint16 masks and Set PWM/Set Angle count=1, semantic ID, little-endian value; do not alter sequence, retry, ACK, duplicate-cache, or APC220 scheduler code.
- [ ] **Step 5: Expand state loops and logs.** Change enabled/pending mask signal loops and any queue/disable-mask iteration from two entries to five and use descriptor semantic names in user-visible logs.
- [ ] **Step 6: Run controller/protocol tests and commit.** Configure/build Qt, run `protocol_tests`, `robot_controller_tests`, and `ctest`; commit `feat(console): add five-servo semantic controller support` after all pass.

## Task 7: Generate the five-card Qt UI

**Files:** Modify `RoboBeetleConsole/src/ui/MainWindow.h/.cpp`.

- [ ] **Step 1: Replace fixed arrays with five-entry collections.** Size widget storage from `kServoCount`; iterate `servoDescriptors()` rather than casting arbitrary indexes to IDs.
- [ ] **Step 2: Build semantic cards.** Create five cards with display labels and independent Enable/Disable, Neutral, PWM, Apply PWM, angle input, Set Angle, and status label. Use each descriptor’s mechanical PWM/angle limits. Arrange cards in a compact grid without introducing a new UI framework.
- [ ] **Step 3: Handle FrontAxis explicitly.** Show `Calibration Pending — PWM Bring-up Only`, configure 1450–1550 us PWM, disable angle input/button permanently, and never claim a calibrated neutral. All other cards expose their descriptor angle capability.
- [ ] **Step 4: Preserve safety gates.** Enable motion controls only when connected and logically enabled/ACKed; continue to honor disable-pending and APC fail-closed state. Keep global Disable All and do not add Enable All.
- [ ] **Step 5: Verify UI build and controller tests.** Build the Console and run all CTest tests; inspect the diff for any remaining two-card/index<2 assumptions; commit `feat(console): generate five semantic servo cards`.

## Task 8: Update documentation and run integrated verification

**Files:** Modify the five documentation files listed above.

- [ ] **Step 1: Update Firmware README.** Document the pure-C descriptor plus HAL binding architecture, all five IDs/channels, shared ~333 Hz timer base, per-servo calibration/mechanical limits, FrontAxis calibration pending, and the layout compatibility break. Preserve historical Hardware Verified Servo1/Set Angle status while marking PR #8 five-servo hardware regression pending.
- [ ] **Step 2: Update Console README/protocol.** Document five semantic cards, exact Protocol V2 payloads/masks, FrontAxis PWM-only behavior, mechanical envelope precedence, no Servo2 alias, and the prohibition on mixing the v0.4 PA6/RearLeft layout with the PR #8 wiring.
- [ ] **Step 3: Update engineering lessons and handoff.** Record the final mapping, TIM3/TIM4 paths, historical PA6 mapping, SAVOX 333 Hz datasheet context, S3150D uncertainty, and required unloaded FrontAxis hardware sequence `1500→1450→1500→1550→1500`; do not claim that sequence is verified.
- [ ] **Step 4: Run fresh integrated verification.** From clean build directories run Firmware configure/build, all Firmware pure-C regressions and Protocol golden vectors, Console configure/build, `protocol_tests`, `robot_controller_tests`, and `ctest`. Capture exact outputs and inspect `git diff --stat`/`git status` for unrelated changes.
- [ ] **Step 5: Commit documentation.** Commit `docs: document five-servo bring-up and layout compatibility` only after tests pass and `git diff --check` is clean.

## Task 9: Self-review, push, and open PR #8

**Files:** No new source files; review all branch changes.

- [ ] **Step 1: Audit scope and behavior.** Confirm no Protocol V2 wire/CRC/COBS/message-ID changes, no Safety timeout/APC scheduler changes, no CPG/Pi/sensor work, no `.ioc` unrelated churn, and no Firmware/Console functional changes outside five-servo bring-up.
- [ ] **Step 2: Check branch and history.** Confirm branch is `feature/five-servo-bringup`, base is `54a8631194645a83e65afa21660be0adb38e27de`, all commits are reviewable, and no unrelated working-tree files are present.
- [ ] **Step 3: Push the branch.** Run `git push -u origin feature/five-servo-bringup`; do not push or merge `main`.
- [ ] **Step 4: Create PR #8.** Use base `main`, head `feature/five-servo-bringup`, title `feat: bring up five semantic servo channels`, and describe mapping, timer wiring, calibration/mechanical limits, tests, unchanged Protocol/Safety/APC behavior, and hardware checks still required. Do not merge.
- [ ] **Step 5: Final report and stop.** Report branch, commits, files, architecture, build/test results, PR URL/number, clean status, and remaining hardware risks; stop for Review and do not begin PR #9.
