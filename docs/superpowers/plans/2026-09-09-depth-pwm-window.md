# Depth PWM Calibration Window Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Expand only the FrontAxis/Depth PWM-only command envelope from the current 1200–1800 us window to 1000–2000 us while preserving the five-servo descriptor contract and keeping angle control disabled.

**Architecture:** Firmware and Qt retain independent descriptor tables at the existing C/C++ boundary. The seller/electrical metadata remains 500/1500/2500 us, while the descriptor command envelope becomes 1000–2000 us in both implementations; the data-driven Qt UI consumes that envelope without a separate hard-coded range. Existing four-servo ranges, protocol framing, calibration mapping, and safety behavior remain untouched.

**Tech Stack:** C11 Firmware descriptor/service tests, C++20/Qt 6 Console descriptors/controller, CMake/Ninja, CTest, host GCC syntax checks, Markdown handoff and engineering documentation.

---

### Task 1: Verify the current range sources and define regression coverage

**Files:**
- Inspect: `RoboBeetleConsole/src/robot/ServoDescriptor.cpp`, `RoboBeetleConsole/src/ui/MainWindow.cpp`
- Inspect: `RoboBeetleFirmware/Core/Servo/servo_descriptor.c`, `RoboBeetleFirmware/Core/Servo/servo_service.c`
- Test: `RoboBeetleConsole/tests/servo_descriptor_tests.cpp`, `RoboBeetleConsole/tests/robot_controller_tests.cpp`, `RoboBeetleFirmware/tests/servo_descriptor_tests.c`, `RoboBeetleFirmware/tests/servo_service_tests.c`

- [x] **Step 1: Search all Depth-specific 1450/1550 uses.**

Run:

```powershell
rg -n -i "FrontAxis|Depth|1450|1550" RoboBeetleConsole RoboBeetleFirmware docs ROBObeetle_HARDWARE_CONTROL_HANDOFF.md
```

Expected: the current descriptor command envelope and assertions use 1200/1800; no independent UI/controller/firmware clamp exists outside the descriptor-driven paths. Any occurrence in documentation or tests is updated only when it describes that envelope.

- [x] **Step 2: Add/adjust boundary assertions before production edits.**

The Qt and Firmware descriptor/controller/service regressions must assert:

```text
FrontAxis command 1000 and 2000 are accepted;
999 and 2001 are rejected;
FrontAxis angle_supported/angleSupported remains false;
FrontRight, FrontLeft, RearRight, and RearLeft retain their existing command limits;
```

Run the focused host tests before changing descriptors and confirm the new 1000/2000 expectations fail because the current descriptor still exposes 1200/1800.

### Task 2: Update the two independent FrontAxis command envelopes

**Files:**
- Modify: `RoboBeetleConsole/src/robot/ServoDescriptor.cpp`
- Modify: `RoboBeetleFirmware/Core/Servo/servo_descriptor.c`

- [x] **Step 1: Change only command min/max fields.**

Use `1000` and `2000` for FrontAxis `commandMinPwmUs`/`commandMaxPwmUs` and Firmware `command_min_pulse_us`/`command_max_pulse_us`. Leave electrical `500/1500/2500`, neutral candidate `1500`, angle ranges `0,0`, `angle_supported=false`, timer/channel mapping, and all other descriptor rows unchanged.

- [x] **Step 2: Run the focused Console and Firmware tests.**

Run the existing focused test executables/build targets. Expected: the new boundary assertions pass, FrontAxis SetAngle remains rejected, and all four other servo range assertions remain green.

### Task 3: Synchronize implementation-facing documentation

**Files:**
- Modify: `RoboBeetleConsole/README.md`
- Modify: `RoboBeetleConsole/docs/protocol.md`
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `ROBObeetle_HARDWARE_CONTROL_HANDOFF.md`
- Modify: `docs/engineering-lessons.md`

- [x] **Step 1: Document the two FrontAxis range layers.**

State that 500/1500/2500 us is seller/electrical capability metadata only, while the current PWM-only command/calibration exploration window is 1000–2000 us in both Qt and Firmware. State that 1500 us is a provisional bring-up center candidate, not a calibrated mechanical center, and angle control remains unavailable.

- [x] **Step 2: Record validation status without upgrading it.**

Keep FrontRight, FrontLeft, RearLeft, and RearRight A12/PWM path as the supplied Hardware Verified facts; keep the original RearRight actuator as a hardware fault/replacement item. Record only the supplied Depth 1480/1500/1520 direction result as Hardware Verified, and mark the new 1000–2000 full-travel/endpoint/angle scope Pending Hardware Verification.

- [x] **Step 3: Add the next unloaded Depth test sequence and validation taxonomy.**

Record `1500,1400,1300,1200,1100,1000`, return to 1500, then `1500,1600,1700,1800,1900,2000`, with smaller steps near resistance and no hard-stop forcing. Distinguish Host Test, ARM Build, Program Verify, Hardware Verified, and Pending; do not claim ARM/program/hardware results that were not observed in this session.

### Task 4: Full verification and delivery checkpoint

**Files:**
- Inspect all changed files and `git diff --stat`

- [x] **Step 1: Run Qt configure/build and CTest.**

Use the existing Qt 6 MinGW/CMake/Ninja environment and run from the worktree root:

```powershell
$qtBuild = Join-Path ([System.IO.Path]::GetTempPath()) "robobeetle-pr8-depth-qt"
cmake -S RoboBeetleConsole -B $qtBuild -G Ninja
cmake --build $qtBuild
ctest --test-dir $qtBuild --output-on-failure
```

Expected: configure/build succeeds and all three registered Console tests pass, including the 1000/2000 FrontAxis boundaries.

- [x] **Step 2: Run Firmware pure-C and HAL syntax checks.**

Compile all existing pure-C regression sources with the repository's established host GCC commands, run them, and run the established host HAL syntax-only command. If `arm-none-eabi-gcc` or STM32CubeIDE is unavailable, record ARM Build and Program Verify as Pending rather than substituting a claim.

- [x] **Step 3: Run hygiene and scope checks.**

Run:

```powershell
git diff --check
git diff --stat
git status --short --branch
```

Confirm no changes to Protocol V2, `.ioc`, timer/pin mappings, safety code, angle mapping, or the four unchanged servo descriptors.

- [x] **Step 4: Commit only the approved scope.**

After all checks pass, commit the descriptor/test/documentation changes with a focused message such as:

```text
feat: widen FrontAxis PWM bring-up window
```
