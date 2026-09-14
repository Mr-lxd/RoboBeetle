# Front Assembly Coordinate Remap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. This plan is intentionally test-first: RED against the existing descriptor table, then the minimal GREEN descriptor changes.

**Goal:** Adapt the logical five-servo descriptors to the 2026-09-14 mechanical installation remap while preserving logical IDs, body-frame angle semantics, CPG behavior, MotionMode behavior, Protocol V2 values, and Qt labels/layout.

**Architecture:** Keep `Legacy Source-Compatible CPG v1 -> semantic adapter -> LogicalJointTargets -> MotionManager common guard -> ServoService -> ServoCalibration` unchanged. Absorb the front mechanical remap only in the logical-servo-to-physical timer/channel/calibration descriptor layer. Reuse the existing signed-delta angle interpolation; do not add a reverse flag.

**Scope:**

- Logical `FrontRight` (ID 0, mask `0x0001`) moves to the former physical FrontLeft: TIM3_CH2 / PA7, `-45/0/+45 cdeg -> 2020/1580/1140 us`, raw command bounds `1140..2020 us`.
- Logical `FrontLeft` (ID 1, mask `0x0002`) moves to the former physical FrontRight: TIM3_CH1 / PA6, `-45/0/+45 cdeg -> 1000/1450/1900 us`, raw command bounds `1000..1900 us`.
- `FrontAxis` (ID 2, mask `0x0004`) remains TIM3_CH3 / PB0, retains raw bounds `1060..2430 us` and neutral `1745 us`, but uses `-90/0/+90 cdeg -> 2430/1745/1060 us`.
- Rear descriptors and the rear `-30..+45 degree` Motion envelope remain unchanged.
- No CPG, gait, turn, MotionManager, MotionMode, Protocol V2 ID, or Qt left/right name/layout production changes are permitted.

## Working rules and baseline

- Work only in `C:\Users\laixindong\.config\superpowers\worktrees\RoboBeetle\feature-cpg-gait-core` on `feature/cpg-gait-core`.
- Preserve unrelated user work in the original checkout, including `RoboBeetleFirmware/daplink.cfg` and `docs/superpowers/plans/2026-09-09-leak-telemetry.md`.
- Add one independent commit to existing PR #15; do not create a PR, merge, reset, clean, or force-push.
- Preserve the historical calibration values in documentation as a superseded pre-remap baseline; distinguish logical identity from physical timer/channel.

## Task 1: Add Firmware RED expectations before production changes

**Files:**

- Modify `RoboBeetleFirmware/tests/servo_descriptor_tests.c`.
- Modify `RoboBeetleFirmware/tests/servo_calibration_tests.c`.
- Modify `RoboBeetleFirmware/tests/servo_driver_stm32_tests.c`.
- Modify `RoboBeetleFirmware/tests/servo_service_tests.c`.
- Modify `RoboBeetleFirmware/tests/protocol_dispatcher_tests.c`.

Update tests first so they require the exact remap:

1. Assert FrontRight TIM3_CH2, FrontLeft TIM3_CH1, FrontAxis TIM3_CH3; preserve IDs and masks.
2. Assert the exact three-point calibration and independent ascending raw bounds above.
3. Assert angle mapping: FrontRight `-45/0/+45 -> 2020/1580/1140`; FrontLeft `-> 1000/1450/1900`; FrontAxis `-90/0/+90 -> 2430/1745/1060`.
4. Assert FrontAxis signed-delta midpoints `-45 -> 2087` and `+45 -> 1403`, including the existing integer truncation behavior.
5. Update service neutral/range and Protocol V2 ID 0 angle/neutral expectations.
6. Make observable STM32 binding tests prove logical FrontRight -> CH2 and logical FrontLeft -> CH1. Keep CH1 HAL-zero validity coverage by exercising the logical FrontLeft binding where appropriate; no HAL sentinel change.
7. Leave rear values and all CPG/Motion semantic assertions unchanged.

Run the focused/full Firmware host runner against the unchanged production descriptor and record the intentional RED failures before modifying `servo_descriptor.c`.

## Task 2: Minimal Firmware GREEN implementation

**File:** `RoboBeetleFirmware/Core/Servo/servo_descriptor.c`

Change only the FrontRight, FrontLeft, and FrontAxis descriptor rows as specified. Do not change `servo_calibration.c`, `servo_driver_stm32.c`, timer GPIO setup, logical IDs, masks, or any CPG/Motion source. Run the focused tests, then the complete Firmware host regression suite.

## Task 3: Add Console RED expectations, then GREEN descriptor parity

**Files:**

- Modify `RoboBeetleConsole/tests/servo_descriptor_tests.cpp` first.
- Modify `RoboBeetleConsole/tests/robot_controller_tests.cpp` to keep controller payload/range assertions aligned with the unchanged logical FrontRight ID after the descriptor remap.
- Modify `RoboBeetleConsole/src/robot/ServoDescriptor.cpp` second.

First update the Qt test to require the new logical calibration/ranges and reversed FrontAxis endpoint direction while preserving semantic names, masks, IDs, and display labels. Run the descriptor test against the unchanged table and record RED. Then update only the Console descriptor rows to match Firmware. The Qt descriptor has no timer/channel field; document that Firmware owns the physical binding while Qt mirrors logical calibration and command ranges.

## Task 4: Documentation and evidence boundary

**Files:**

- Create `docs/front-assembly-remap-2026-09-14.md`.
- Update the active current-contract sections of `RoboBeetleConsole/README.md`, `RoboBeetleConsole/docs/protocol.md`, and `docs/engineering-lessons.md` as needed.

Record the remap date, exact logical-to-physical table, former physical calibration baseline, FrontAxis sign inversion, stable Protocol IDs, and the unchanged CPG/Motion contract. Keep historical sections intact but label superseded values as pre-remap. Record evidence as:

- Front assembly remap: **Software Verified** after host tests.
- Physical channel mapping and FrontAxis direction: **Pending Hardware Verification**.
- CPG target ARM/performance and physical CPG behavior: **Pending**.
- Water/hydrodynamic Ascend/Descend verification: **Pending Water Verification**.

Include the required post-push hardware checklist: FrontRight only ±10°, FrontLeft only ±10°, FrontAxis 0/+10/-10°, Forward CPG side/turn sanity, and desk-only Ascend/Descend with hydrodynamics pending.

## Task 5: Verification, commit, and PR handoff

Run and capture fresh results for:

- all Firmware Servo, Motion, CPG, Protocol, safety, and host regression tests;
- Qt clean configure/build/CTest;
- public-header/self-sufficiency checks and `git diff --check`;
- ARM status using the existing target gate. If `arm-none-eabi-gcc` is unavailable, report that fact; do not change `double` to `float` and do not claim target timing.

Confirm the final diff contains no production CPG semantic changes, no Protocol ID changes, no Qt left/right layout changes, and no unrelated files. Re-query PR #15 state/head after pushing. Commit as `fix: remap front servos for rotated assembly`, push to the existing `feature/cpg-gait-core`, leave PR #15 OPEN and unmerged, and finish with `READY FOR EXTERNAL GITHUB RE-REVIEW`.
