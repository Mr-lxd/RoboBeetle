# SimpleGait Shell-Limit Diagnostic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Encode the measured front shell safety envelope and prove the approved logical SimpleGait-to-PWM direction contract through the existing production pipeline on PR #15.

**Architecture:** Preserve the final FR/FL calibration tuples and all generator semantics. Store raw command shell bounds in the Firmware and Qt descriptor command fields, and enforce the common installed angle envelope in MotionManager after generator sampling and before ServoService calibration. Extend existing host tests with a real SimpleGait/MotionManager/ServoService pipeline regression and keep the unresolved physical side/sign evidence explicitly pending.

**Tech Stack:** C11 Firmware host runner with GCC, STM32-independent descriptor and Motion tests, Qt 6/C++20 MinGW Console tests, Markdown design/evidence records, GitHub CLI.

---

## File map

- Create `docs/superpowers/specs/2026-09-14-simple-gait-shell-diagnostic-design.md` for the frozen calibration-versus-operational contract and evidence boundary.
- Create `docs/superpowers/plans/2026-09-14-simple-gait-shell-diagnostic.md` for this test-first execution record.
- Modify `RoboBeetleFirmware/tests/servo_descriptor_tests.c` and `RoboBeetleFirmware/tests/servo_service_tests.c` for exact raw command bounds while retaining full calibration endpoints.
- Modify `RoboBeetleFirmware/tests/motion_manager_tests.c` for front common-guard limits and a real full-pipeline quarter/half-cycle regression.
- Modify `RoboBeetleFirmware/Core/Servo/servo_descriptor.c` for only FR/FL raw command bounds.
- Modify `RoboBeetleFirmware/Core/Motion/motion_config.h` and `RoboBeetleFirmware/Core/Motion/motion_manager.c` for the common front operational envelope.
- Modify `RoboBeetleConsole/tests/servo_descriptor_tests.cpp` and `RoboBeetleConsole/tests/robot_controller_tests.cpp` for Qt parity and raw command boundary payloads.
- Modify `RoboBeetleConsole/src/robot/ServoDescriptor.cpp` for Qt command bounds only.
- Modify `docs/front-assembly-remap-2026-09-14.md`, `docs/simple-gait-bench-remap-2026-09-14.md`, `docs/motion-simple-gait.md`, `docs/cpg-gait-core.md`, `RoboBeetleFirmware/README.md`, `RoboBeetleConsole/README.md`, and `RoboBeetleConsole/docs/protocol.md` for the active operational contract and hardware checklist.

## Task 1: Establish Firmware RED expectations

**Files:**

- Modify `RoboBeetleFirmware/tests/servo_descriptor_tests.c`.
- Modify `RoboBeetleFirmware/tests/servo_service_tests.c`.
- Modify `RoboBeetleFirmware/tests/motion_manager_tests.c`.

- [x] **Step 1: Require the distinct raw command bounds.**

Keep the existing electrical calibration expectations exactly:

```c
FrontRight: 1140U, 1580U, 2020U, -4500, 4500
FrontLeft:  1900U, 1450U, 1000U, -4500, 4500
```

Change only expected command pulse fields and Service raw cases to:

```c
FrontRight: command 1140U..1860U
FrontLeft:  command 1160U..1900U
```

Each lower/upper boundary must be accepted and the adjacent numeric value
rejected. Servo angle tests must continue to accept the full `-4500..+4500`
calibration angle range because the angle command range is intentionally a
separate contract in this checkpoint.

- [x] **Step 2: Add front Motion guard expectations before production changes.**

Extend the alternate-generator common-guard test with targets
`front_right=-5000`, `front_left=4000`, `rear_right=-4000`, and
`rear_left=5000`. After the production tick, require
`front_right=-4500`, `front_left=2800`, `rear_right=-3000`, and
`rear_left=4500`, with four clamp events. Extend the stop-path sanitizer test
to prove a retained front target is also bounded.

- [x] **Step 3: Add the real SimpleGait full-pipeline regression.**

Extend the existing MotionManager fixture's fake driver with per-servo pulse
storage. Use the real `simple_gait_generator_interface`, real MotionManager,
real ServoService, and real calibration descriptors. Set the exposed generator
phase to `pi/2 - 3*pi/4 = -pi/4`, process the 750 ms start transition, and
assert the resulting logical targets and pulses:

```c
FrontRight +1000 -> pulse > 1580U
FrontLeft  +1000 -> pulse < 1450U
RearRight  -1000 -> pulse < 1570U
RearLeft   -1000 -> pulse > 1450U
```

Then keep the same fixture alive for one 1000 ms half-cycle with a fresh
heartbeat and assert all four logical signs and pulse-direction inequalities
reverse. Do not construct the expected logical output as a fake generator.

- [x] **Step 4: Run the Firmware RED gate.**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\run_host_tests.ps1
```

Expected RED evidence is limited to the newly changed raw-bound and Motion
front-guard assertions against the unchanged production descriptor/guard. The
existing CPG, safety-before-catch-up, Protocol, and unrelated regressions must
remain distinguishable from these intentional failures.

## Task 2: Implement the minimal Firmware contract

**Files:**

- Modify `RoboBeetleFirmware/Core/Servo/servo_descriptor.c`.
- Modify `RoboBeetleFirmware/Core/Motion/motion_config.h`.
- Modify `RoboBeetleFirmware/Core/Motion/motion_manager.c`.

- [x] **Step 1: Change only FR/FL command pulse fields.**

Retain every calibration field, neutral, timer, channel, ID, and mask. Set:

```c
FrontRight: .command_min_pulse_us = 1140U,
             .command_max_pulse_us = 1860U,
FrontLeft:  .command_min_pulse_us = 1160U,
             .command_max_pulse_us = 1900U,
```

- [x] **Step 2: Define the common front operational constants.**

In `motion_config.h`, add `MOTION_FRONT_MIN_CDEG (-4500)` and
`MOTION_FRONT_MAX_CDEG 2800` beside the existing rear constants, with static
assertions that freeze both values. Do not change `MOTION_PROFILE_FREQUENCY_HZ`,
profile amplitude, CPG sources, or calibration angle ranges.

- [x] **Step 3: Apply the front limits in the common sanitizer.**

In `motion_manager_sanitize_targets`, clamp both `front_right_cdeg` and
`front_left_cdeg` to `MOTION_FRONT_MIN_CDEG..MOTION_FRONT_MAX_CDEG`, incrementing
the existing diagnostic count once per clamped target. Leave the rear clamps
and the generator files unchanged. The already shared sanitizer call sites
must continue to cover normal ticks and graceful STOP ticks.

- [x] **Step 4: Run the focused Firmware GREEN gate.**

Run the host runner again. The descriptor, Service, Motion front-guard, real
SimpleGait pipeline, CPG, Safety, and Protocol tests must all pass.

## Task 3: Establish Qt RED expectations and parity

**Files:**

- Modify `RoboBeetleConsole/tests/servo_descriptor_tests.cpp`.
- Modify `RoboBeetleConsole/tests/robot_controller_tests.cpp`.
- Modify `RoboBeetleConsole/src/robot/ServoDescriptor.cpp`.

- [x] **Step 1: Change Qt tests before changing the Qt descriptor.**

Require FrontRight command range `1140..1860` and FrontLeft `1160..1900`, while
retaining electrical tuples FR `1140/1580/2020` and FL `1900/1450/1000`.
Extend `testPwmCalibrationAndBounds` to reject FR `1861`, accept `1860`, and
check payload `01004407`. Add FrontLeft raw checks: reject `1159` and `1901`,
accept `1160` (`01018804`) and `1900` (`01016c07`).

- [x] **Step 2: Run the focused Qt RED test.**

Configure a temporary MinGW build with `D:\Qt\6.11.2\mingw_64`, build the
descriptor and controller tests, and run them. The old Qt descriptor/table
must fail the changed bounds before its production row is edited.

- [x] **Step 3: Update only the Qt command fields and run Qt GREEN.**

Set the Qt FrontRight command range to `1140..1860` and FrontLeft to
`1160..1900`, without touching Qt electrical calibration, labels, IDs, masks,
or UI layout. Run a clean configure/build/CTest and require all six tests to
pass.

## Task 4: Update active documentation and evidence status

**Files:**

- Modify `docs/front-assembly-remap-2026-09-14.md`.
- Modify `docs/simple-gait-bench-remap-2026-09-14.md`.
- Modify `docs/motion-simple-gait.md`.
- Modify `docs/cpg-gait-core.md`.
- Modify `RoboBeetleFirmware/README.md`.
- Modify `RoboBeetleConsole/README.md`.
- Modify `RoboBeetleConsole/docs/protocol.md`.

- [x] **Step 1: Update operational tables without rewriting calibration.**

Use raw command ranges FR `1140–1860 us` and FL `1160–1900 us` in active
descriptor/protocol tables. Keep calibration rows FR `1140/1580/2020` and FL
`1900/1450/1000` unchanged. Describe Motion front operational angles as
`-4500..+2800 cdeg`, rear as `-3000..+4500 cdeg`, and call `T=2.0 s` a
`nominal period parameter` wherever the active documentation discusses it.

- [x] **Step 2: Record the latest physical evidence boundary.**

Add the measured shell limits, approximate `+28.6/+29.0 deg` conversions,
conservative `+28.0 deg` common cap, unresolved same-direction anomaly, and
the static anti-phase pose checklist. Mark explicit logical-to-physical side
confirmation and automatic Forward anti-phase as **Pending**. Keep FrontAxis
desk direction **Bench Mechanical Verified** and water effect **Pending Water
Verification**.

## Task 5: Verification, commit, and PR #15 push

- [x] **Step 1: Run all required fresh verification.**

Run the complete Firmware host runner, clean Qt configure/build/CTest, the
existing 34-header C11 self-sufficiency check, and `git diff --check`. Verify
the safety-before-catch-up regression reports no post-gap actuator write and
no auto-resume. Keep CPG tests in the runner but do not claim CPG is the active
actuator backend.

- [x] **Step 2: Record target evidence honestly.**

Check `arm-none-eabi-gcc.exe` and `arm-none-eabi-size.exe`. If unavailable,
record `NOT_FOUND` and report ARM FLASH/RAM deltas, one 10 ms substep time,
representative 20/70/100 ms catch-up times, and worst bounded catch-up timing
as pending. Do not change production CPG `double` types or infer target timing
from host output.

- [x] **Step 3: Audit scope and preserve the original checkout.**

Confirm no changes in `SimpleGaitGenerator`, `CPG` core/generator, Safety
source, Protocol IDs, Qt UI, or calibration algorithm. Confirm the original
detached checkout still has only its pre-existing untracked
`RoboBeetleFirmware/daplink.cfg` and
`docs/superpowers/plans/2026-09-09-leak-telemetry.md`.

- [x] **Step 4: Commit and push the independent checkpoint.**

Stage only the files named in this plan, inspect the cached diff, and create
one commit:

```powershell
git commit -m "fix: enforce front shell operational limits"
git push origin feature/cpg-gait-core
```

Do not create a PR and do not merge. Re-query PR #15 and require OPEN,
non-draft, unmerged state with the new head SHA. Finish with the exact phrase
`READY FOR STATIC ANTI-PHASE HARDWARE DIAGNOSIS`.
