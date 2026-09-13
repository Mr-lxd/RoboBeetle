# Motion / Simple Gait Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add the first deterministic, desktop-verifiable Motion / Gait foundation with logical joint targets, provisional simple profiles, explicit actuator ownership, graceful STOP, immediate Safety takeover, Protocol V2 control, and a minimal Qt bench panel.

**Architecture:** Keep the existing cooperative `app_main_process()` and `SafetySupervisor` as the only foreground safety authority. `MotionManager` owns the motion state machine and 10 ms tick, `SimpleGaitGenerator` emits HAL-independent logical centidegree targets, and `ServoService` remains the sole calibration/PWM boundary. The Qt controller sends one start/stop command and renders firmware state transitions; it never streams targets or owns PWM.

**Tech Stack:** C11 firmware, existing STM32 HAL/CMake layout, pure-C host tests compiled with GCC, C++20/Qt6 console, Qt Test, CMake/Ninja.

---

## Task 1: Add motion types, target contract, and centralized provisional constants

**Files:**
- Create `RoboBeetleFirmware/Core/Motion/motion_types.h`.
- Create `RoboBeetleFirmware/Core/Motion/joint_targets.h`.
- Create `RoboBeetleFirmware/Core/Motion/gait_generator.h`.
- Create `RoboBeetleFirmware/Core/Motion/motion_config.h`.

**Steps:**

1. Define `motion_mode_t` with the stable wire-facing order `STOP`, `FORWARD`, `BACKWARD`, `TURN_LEFT`, `TURN_RIGHT`, `ASCEND`, `DESCEND`, `COUNT` and `motion_state_t` with `STOPPED`, `RUNNING`, `STOPPING`, `FAULTED`.
2. Define `joint_targets_t` as five signed `int32_t` logical centidegree fields in Servo ID order: FrontRight, FrontLeft, FrontAxis, RearRight, RearLeft. Keep this header independent of HAL and ServoService.
3. Define the replaceable generator interface: explicit `advance(dt_ms)`, `sample(mode, amplitude_scale, bias_scale, output)`, and a mode-validity operation. The interface must carry only logical motion data and no allocation, PWM, HAL, or ISR dependency.
4. Put the bench-provisional values in one header: `MOTION_GAIT_TICK_MS = 10`, `MOTION_TRANSITION_DURATION_MS = 750`, `0.5 Hz`, `10 degree` paddle amplitude, `+/-10 degree` FrontAxis candidate bias, `50%` turn-side scale, and rear operational bounds `-3000..+4500 cdeg`.
5. Add compile-time/static assertions or test-visible constants that make the five mode IDs, tick, transition duration, and rear limits unambiguous.
6. Do not change ServoDescriptor calibration endpoints or manual command ranges.

**Verification:** `rg` confirms no duplicate `750` transition literal outside the centralized motion configuration and no Servo calibration values changed; headers compile under `-std=c11 -Wall -Wextra -Werror` in the next test target.

## Task 2: Write failing deterministic generator tests, then implement SimpleGaitGenerator

**Files:**
- Create `RoboBeetleFirmware/tests/simple_gait_generator_tests.c`.
- Create `RoboBeetleFirmware/Core/Motion/simple_gait_generator.h`.
- Create `RoboBeetleFirmware/Core/Motion/simple_gait_generator.c`.

**Steps:**

1. Add tests first for every supported mode being valid, STOP producing zero targets, default forward profile amplitudes/biases, front/rear phase relation, same logical sign on both sides, backward stroke inversion candidate, 10 ms phase advancement at 0.5 Hz, and deterministic repeated sampling.
2. Add tests for turn-left/right side scales and ascend/descend FrontAxis bias while paddles remain forward-like. Test that profile values are sourced from the centralized configuration rather than hidden per-mode literals.
3. Add boundary tests for rear operational limiting: `-3001 -> -3000`, `-3000` unchanged, `+4500` unchanged, and `+4501 -> +4500`; assert a diagnostic counter changes only when a clamp occurs.
4. Run the new test before implementation and record the expected compile/undefined-symbol failure.
5. Implement a fixed-size generator context with phase in radians. Advance by `2*pi*frequency_hz*dt_ms/1000` using explicit deterministic input; do not use wall clock, malloc, delays, or HAL.
6. Sample `bias + amplitude*sin(phase + offset)` into integer centidegrees with the profile table. Apply the installed rear operational clamp and expose diagnostics. Keep ServoService absolute calibration validation as the subsequent safety boundary.
7. Keep the interface replaceable so a future CPG generator can emit the same `joint_targets_t` without changing MotionManager or ServoService.

**Verification:** Compile and run `simple_gait_generator_tests` with `-Wall -Wextra -Werror`; all profile, phase, determinism, and clamp assertions pass. `git diff --check` is clean.

## Task 3: Add ServoService Motion ownership with failing arbitration tests first

**Files:**
- Modify `RoboBeetleFirmware/Core/Servo/servo_service.h`.
- Modify `RoboBeetleFirmware/Core/Servo/servo_service.c`.
- Modify `RoboBeetleFirmware/tests/servo_service_tests.c`.

**Steps:**

1. Add a `BUSY` result after the existing result values without renumbering any existing result code, plus an explicit owner state in `servo_service_t`.
2. Add tests that an enabled Motion owner can apply logical angles, manual SetAngle/SetPWM/Neutral/Enable return `BUSY`, and ownership remains held after a motion write.
3. Add tests that `servo_service_motion_begin(mask)` rejects missing enable bits, rejects a second owner, and `servo_service_motion_end()` releases ownership. Test `servo_service_motion_abort()` releases it for immediate takeover.
4. Add tests that Disable and Disable All remain allowed while Motion owns channels, clear enabled state as appropriate, and do not allow a later Motion write.
5. Run the modified ServoService test before implementing the ownership changes and confirm the new assertions fail for the expected missing API/behavior.
6. Implement `motion_begin`, motion-only `set_angle_from_motion`, `motion_end`, `motion_abort`, and an ownership query. Gate all manual actuator-writing paths with `BUSY` while Motion owns any actuator. Keep Disable/Disable All unconditional and preserve existing calibration mapping and absolute bounds.
7. Ensure the motion angle API uses logical centidegrees and passes through the existing descriptor/calibration conversion; it must not introduce raw PWM or a second mapping table.

**Verification:** Existing calibration, enable/disable, and rollback tests still pass, and the new ownership tests pass. The frozen five-servo descriptors and PWM endpoints are byte-for-byte unchanged.

## Task 4: Write failing MotionManager state/STOP/Safety tests, then implement the manager

**Files:**
- Create `RoboBeetleFirmware/Core/Motion/motion_manager.h`.
- Create `RoboBeetleFirmware/Core/Motion/motion_manager.c`.
- Create `RoboBeetleFirmware/tests/motion_manager_tests.c`.

**Steps:**

1. Build a deterministic fake Servo driver in the test fixture and initialize SafetySupervisor, ServoService, SimpleGaitGenerator, and MotionManager with all five channels enabled where needed.
2. Add failing tests for `STOPPED -> RUNNING`, missing required Servo enable rejection, running mode change cross-fade, and phase continuity across mode changes.
3. Add failing STOP contract tests: request STOP returns `OK` immediately, state becomes `STOPPING`, ownership remains Motion, and the manager reaches `STOPPED` after approximately 750 ms of 10 ms ticks. Assert all logical targets converge monotonically toward neutral and the final write is zero.
4. Add tests for STOP idempotency in `STOPPED`, `BUSY` for a Start while `STOPPING`, and no manual Servo operation during the ramp.
5. Add tests for immediate abort on watchdog/liveness loss during STOPPING, immediate Disable All takeover, owner release on abort, fault/stopped state handling, and explicit reconnect-equivalent reinitialization not resuming a prior mode.
6. Run the new manager test before implementation and confirm it fails for the expected missing manager APIs.
7. Implement a non-blocking manager with one fixed 10 ms tick per `motion_manager_process(now_ms)` call. Start acquires required Servo ownership and ramps profile scales from zero to one over the centralized duration. Running mode changes cross-fade old/new profiles while preserving generator phase.
8. Implement ordinary `request_stop()` as an acceptance-time state transition. Hold generator phase, linearly scale the retained target vector to zero over 750 ms, apply each step through the Motion-only Servo API, release ownership only at the final neutral write, and enter `STOPPED`.
9. Implement `motion_manager_abort()` for Safety/Disable All/internal application failure. Cancel any ramp, prevent further gait writes, release ownership, and mark the appropriate stopped/faulted state. Never wait for the graceful deadline.
10. Use wrap-safe elapsed-time comparisons and no busy wait, dynamic allocation, or ISR work. Expose state, active mode, last targets, and clamp diagnostics for deterministic tests and integration.

**Verification:** `motion_manager_tests` passes with exact state assertions, an elapsed range centered on 750 ms (within one tick), monotonic convergence, ownership arbitration, immediate safety interruption, and no auto-resume.

## Task 5: Extend Protocol V2 and dispatcher with acceptance-time Motion ACKs

**Files:**
- Modify `RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h`.
- Modify `RoboBeetleFirmware/Core/Communication/protocol_dispatcher.h`.
- Modify `RoboBeetleFirmware/Core/Communication/protocol_dispatcher.c`.
- Modify `RoboBeetleFirmware/tests/protocol_dispatcher_tests.c`.

**Steps:**

1. Add the unused command ID `RBP2_MSG_SET_MOTION_MODE = 0x15` and append `RBP2_RESULT_BUSY = 7` without changing existing IDs/results.
2. Add failing dispatcher tests for strict three-byte payload validation (`schema=1, mode, action`), valid START/STOP dispatch, invalid STOP mode, invalid START STOP-mode, invalid action, invalid schema, and unsupported mode.
3. Add tests proving STOP ACK is returned when the manager enters STOPPING, before the 750 ms ramp completes. Add successful duplicate replay tests proving no second start/stop side effect, and failed commands not cached under the existing Protocol V2 contract.
4. Add tests proving explicit Servo Disable first aborts Motion and is accepted during STOPPING; manual angle/PWM packets return `BUSY` during ownership.
5. Run the modified dispatcher test before implementation to establish the expected failure.
6. Wire a MotionManager pointer into the dispatcher. Decode only the exact payload, gate START on heartbeat and manager Servo readiness, allow safety STOP without requiring host liveness, and map manager outcomes including `BUSY` to Protocol V2 results.
7. Before Servo Disable/Disable All side effects, call the manager immediate-abort hook. Preserve the existing duplicate cache semantics, heartbeat handling, ACK framing, and all legacy commands.

**Verification:** Firmware protocol dispatcher tests and all existing protocol golden vectors pass; `0x15` is accepted only with the documented schema/action and STOP ACK timing is acceptance-based.

## Task 6: Wire the cooperative application and firmware build/test targets

**Files:**
- Modify `RoboBeetleFirmware/Core/App/app_main.c`.
- Modify `RoboBeetleFirmware/CMakeLists.txt`.
- Modify `RoboBeetleFirmware/README.md` and firmware test/build instructions as needed.

**Steps:**

1. Instantiate the generator and MotionManager beside existing SafetySupervisor/ServoService objects; initialize them before dispatcher use and pass the manager pointer to the dispatcher.
2. Keep `main.c` thin and keep `app_main_process()` as the only scheduler. In each pass, use one captured `HAL_GetTick()` value, process Safety first, immediately abort/disable/invalidate on a Safety event, otherwise process at most one MotionManager tick. If target application fails, take the same immediate fail-safe path.
3. Do not change heartbeat timeout, leak telemetry/monitoring semantics, generated CubeMX files, interrupt handlers, or DAPLink configuration.
4. Add a firmware host-test CMake target or documented reproducible compile commands for all new pure-C tests and link `libm` where required; keep target-specific STM32 sources out of host tests.
5. Build the firmware host tests and, if `arm-none-eabi-gcc` is available, build the ARM target with the existing preset. Record ARM unavailability separately rather than treating host success as ARM success.

**Verification:** All baseline firmware executables plus the new generator/manager/dispatcher coverage pass. `cmake --build --preset Debug` is run if configured tools permit; no safety path contains a 750 ms delay.

## Task 7: Add Qt Protocol/Controller Motion state and arbitration tests first

**Files:**
- Modify `RoboBeetleConsole/src/protocol/Packet.h`.
- Modify `RoboBeetleConsole/src/robot/RobotCommand.h`.
- Modify `RoboBeetleConsole/src/robot/RobotController.h`.
- Modify `RoboBeetleConsole/src/robot/RobotController.cpp`.
- Modify `RoboBeetleConsole/tests/protocol_tests.cpp`.
- Modify `RoboBeetleConsole/tests/robot_controller_tests.cpp`.

**Steps:**

1. Add host enums/mappings for MotionMode, MotionAction, MotionState, the `0x15` message type, and ACK result `BUSY=7`; retain all existing message/result values.
2. Add failing codec/controller tests for exact three-byte payloads, start/stop commands, ACK-driven `Running`, ACK-driven `Stopping`, and the provisional single-shot transition to `Stopped`.
3. Add tests that manual SetAngle/SetPWM/Neutral are rejected locally while Running or Stopping, firmware-side BUSY ACKs are surfaced, Disable All remains available, disconnect/liveness loss clears/faults motion, and reconnect never auto-resumes.
4. Run these tests before implementation and capture the expected missing symbol/behavior failures.
5. Add controller APIs/signals for `startMotion`, `stopMotion`, selected mode, and MotionState. Attach the 750 ms single-shot timer only after a successful STOP ACK; do not claim the timer is actuator confirmation.
6. Include Motion commands in the existing APC220 queue/disable-preemption bookkeeping while preserving heartbeat priority and the DirectUart behavior. A successful Disable All immediately changes local UI state to a safe fault/stopped representation; timeout/liveness fail-closed stops any pending graceful timer.
7. Update local manual-command guards and UI-facing state without changing the existing transport retry/duplicate sequence policy.

**Verification:** Console protocol/controller tests pass in both DirectUart and APC220 configurations with Qt runtime DLLs on `PATH`; existing leak/IMU/depth/servo tests remain green.

## Task 8: Add the minimal Motion / Gait bench panel and UI tests

**Files:**
- Modify `RoboBeetleConsole/src/ui/MainWindow.h`.
- Modify `RoboBeetleConsole/src/ui/MainWindow.cpp`.
- Modify `RoboBeetleConsole/tests/main_window_tests.cpp`.

**Steps:**

1. Add a `QGroupBox` titled `Motion / Gait — Bench`, a mode combo with the seven documented labels, Start/Stop buttons, and a status label with `Stopped`, `Running`, `Stopping`, and `Faulted` states.
2. Add failing UI tests that find the group and controls, verify all modes are present, verify Start/Stop enablement, verify servo manual controls disable while Motion is active, and verify global Disable All remains enabled.
3. Connect controls to RobotController and refresh the panel from controller signals. Do not disable the global Disable All path.
4. Keep the first panel explicit that profiles are Bench Provisional / Pending Water Verification; do not add tuning fields, raw PWM controls, or hidden auto-enable behavior.
5. Run the UI tests with the existing Qt test environment, including required Qt DLL path setup.

**Verification:** `main_window_tests` passes and the panel remains usable when disconnected; manual controls recover after a completed stop or fail-safe reset.

## Task 9: Update canonical documentation and run the full evidence matrix

**Files:**
- Modify `RoboBeetleFirmware/README.md`.
- Modify `RoboBeetleConsole/README.md`.
- Modify `RoboBeetleConsole/docs/protocol.md`.
- Modify `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`.
- Add or update the canonical Motion/Gait verification note under `docs/`.

**Steps:**

1. Document the mode/state contract, exact `0x15` payload, ACK acceptance semantics, 750 ms centralized provisional duration, ownership/BUSY behavior, and immediate Safety/Disable All precedence.
2. Document the immutable calibration mapping and gait-only rear envelope separately; state that no manual UI endpoints or ServoDescriptor limits changed.
3. Mark all motion profiles and hydrodynamic claims `Bench Provisional / Pending Water Verification`; distinguish host tests, ARM build, programming, desktop bench, and water evidence.
4. Run `git diff --check`, all firmware host executables, the ARM build if available, all Qt CTest targets, and a final `rg` audit for accidental raw PWM/gait-in-ISR/second-scheduler/auto-resume claims.
5. Verify the isolated worktree contains only intended changes and that `RoboBeetleFirmware/daplink.cfg` and the pre-existing untracked leak plan in the original worktree were not touched.

**Verification:** The final report can list exact commands and outcomes by evidence category without converting unavailable ARM/hardware evidence into PASS.

## Task 10: Commit the implementation and prepare (but do not merge) the external-review PR

**Steps:**

1. Review the complete diff against the merged `origin/main`, inspect merge-base and changed-file scope, and confirm no generated CubeMX or calibration files were unintentionally modified.
2. Run the verification matrix again after any documentation-only edits; record test counts, ARM tool availability, and limitations.
3. Create a normal commit history on `feature/motion-simple-gait`, push the branch, and open a new PR targeting `main` with the requested Motion/Gait summary and evidence checklist.
4. Do not merge the new PR. Verify its state, head SHA, base, and checks/review status, then report the PR number/link as open and finish with `READY FOR EXTERNAL GITHUB REVIEW`.

**Verification:** New PR is open and unmerged, points from `feature/motion-simple-gait` to `main`, and the final response separates implementation/host evidence from pending ARM/program/hardware evidence.
