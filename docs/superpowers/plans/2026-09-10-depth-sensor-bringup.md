# Depth sensor and decoder-board bring-up implementation plan

## Baseline and ownership

- Work only in `feature/depth-sensor-bringup`, based on the verified
  `origin/main` commit recorded in the task.
- Preserve the detached hardware checkout and its two pre-existing untracked
  files; never add, delete, clean, or overwrite them.
- Discover the complete current Firmware regression set from
  `RoboBeetleFirmware/tests` and its README/CMake instructions before each
  full gate. Do not replace it with a historical subset.
- Coordinator owns shared files: `RoboBeetleFirmware/RoboBeetleFirmware.ioc`,
  `Core/Src/main.c`, `Core/Src/stm32f4xx_hal_msp.c`,
  `Core/Src/stm32f4xx_it.c`, `Core/Inc/stm32f4xx_it.h`,
  `Core/App/app_main.c/.h`, the Firmware/Console protocol message registries,
  and `telemetry_scheduler.c/.h`.
- Agents may edit only disjoint new modules, their focused tests, and isolated
  documentation sections. They must run red/green tests in their worktree and
  report changed paths and commands.

## Checkpoint 0 — approved design and baseline

1. Record the approved design and exact 38-byte `DepthSnapshot` contract.
2. Run the existing complete Firmware host regressions, the current five
   Console CTests, and baseline syntax/portability checks.
3. Commit only the design/plan checkpoint before production implementation.

## Checkpoint 1 — pure-C parser and transport red/green

Agent A owns new files under `RoboBeetleFirmware/Core/Communication/` and
focused Firmware tests:

1. Add failing parser tests for canonical format and the exact documented
   `Depth:...m Temp=...C` format, split CRLF, concatenated lines, signed values, fixed-point exactness,
   malformed units/separators, bare LF, overflow, leading garbage, overlong
   recovery, and invalid-to-valid recovery.
2. Implement the bounded pure-C parser/state with direct standard headers and
   deterministic fixed-point conversion.
3. Add failing transport tests with HAL mocks for byte/overflow counters,
   immediate receive, deferred `HAL_BUSY`, hard `HAL_ERROR`, UART error
   recovery, instance filtering, and continued receive after foreground re-arm.
4. Implement the independent USART6 transport with one-byte interrupt receive,
   512-byte ring storage, foreground maintenance, and observable diagnostics.
5. Run the focused red/green commands, then Coordinator reviews the diff before
   accepting the slice.

## Checkpoint 2 — Firmware codec, policy, and scheduler red/green

Agent B owns new pure-C depth codec/policy files and focused tests, without
editing the shared scheduler or protocol registry:

1. Add failing codec tests for message ID `0x22`, payload length 38, schema 1,
   little-endian signed fields, validity zeroing, reserved flags, saturated age,
   all diagnostics, and golden payload/wire vectors.
2. Implement explicit byte-wise encode/decode helpers; never copy a C struct to
   the wire.
3. Add failing one-second wrap-safe depth policy tests and implement the policy.
4. Provide the new slot/policy API expected by Coordinator's scheduler
   integration, documenting that a successful send alone advances state.

## Checkpoint 3 — Qt model/monitor red/green

Agent C owns new Qt `DepthSnapshot`/`DepthMonitor` files and focused tests:

1. Add failing codec tests for the exact 38-byte payload, golden vector,
   schema/length/reserved flags, signed values, invalid-field zero rules, and
   age handling.
2. Implement the Qt value type and decoder with explicit little-endian reads.
3. Add failing lifecycle tests for Unknown/Receiving/Stale/Error, disconnect,
   liveness loss, invalid measurements, recovery, and old-value clearing.
4. Implement `DepthMonitor` without touching pending ACK/retry state.

## Checkpoint 4 — Coordinator Firmware integration

Coordinator integrates the accepted slices in this order:

1. Extend Firmware and Console message registries with `DepthSnapshot=0x22` and
   known-message/non-actuator handling.
2. Generalize the pure-C telemetry scheduler to Leak/IMU/Depth with immediate
   LeakStatus priority and fair IMU/Depth rotation when LeakStatus is not due.
   Preserve mark-after-success and one optional frame.
3. Extend `app_main` with a depth UART/transport/state dependency, poll and
   parser drain, depth policy, snapshot construction, and post-ACK scheduling.
4. Add the USART6 callbacks while keeping USART1 and USART3 instance behavior
   unchanged. Error callback only records/arms transport maintenance.
5. Run the full Firmware regression set plus all new focused Firmware tests.

## Checkpoint 5 — Coordinator MCU/.ioc integration

1. Re-read the current `.ioc` keys immediately before editing:
   `Mcu.IP*`, `Mcu.IPNb`, `Mcu.Pin*`, `Mcu.PinsNb`, and
   `ProjectManager.functionlistsort`.
2. Append USART6 and PC6/PC7 based on the actual numbering, preserving every
   existing USART1, USART3, Servo, Leak, SWD, GPIO, and Timer entry.
3. Add generated-style USART6 initialization at 115200 8-N-1, PC6/PC7 AF8
   MSP setup/deinit, IRQ declaration/handler, and the main init call.
4. Do not regenerate the full CubeMX project. Run `.ioc` consistency checks and
   HAL/generated-style syntax checks.

## Checkpoint 6 — Qt integration and CMake

Coordinator integrates the Qt slice:

1. Add `DepthMonitor` to `RobotController` routing before ACK handling and
   reset it on transport disconnect/liveness loss.
2. Add a read-only Depth Sensor panel with status, values, age, and counters;
   invalid/stale values render as `--`.
3. Preserve existing IMU, Leak, Servo, command queue, and safety behavior.
4. Register new Firmware sources in root Firmware CMake and new Qt sources/tests
   in Console CMake; do not alter generated-source lists unnecessarily.
5. Run the complete Console CTest set and all Firmware regressions.

## Checkpoint 7 — documentation, review, and delivery

1. Synchronize Firmware README, Console README, Protocol V2 documentation,
   `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`, and
   `docs/engineering-lessons.md` with the exact formats, contract, bandwidth,
   listen-only boundary, and verification labels.
2. Run full Firmware regressions, all Console CTests, focused depth tests,
   direct-standard-header/self-sufficiency audit, `.ioc`/HAL checks, and
   `git diff --check`.
3. Run an independent code review. Fix actionable findings and rerun affected
   gates. Review must specifically check that no decoder command is sent and
   no existing USART1/USART3 behavior changed.
4. Commit implementation in reviewable checkpoints, push the exact feature
   branch, and open one PR against `main`. Do not merge.
5. Stop with `READY FOR EXTERNAL GITHUB REVIEW` and
   `External GitHub Review: Pending`. Program Verify and Hardware Verified stay
   Pending until the user performs the real hardware run.
