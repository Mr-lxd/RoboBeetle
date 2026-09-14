# SimpleGait Bench Default Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Switch the normal bench Firmware image to the existing SimpleGait backend for isolated front-assembly remap verification.

**Architecture:** Retain the existing `MOTION_DEFAULT_GAIT_BACKEND_CPG` compile-time selector and `app_main.c` branch. Change only its default to `0`, pass exactly `simple_gait_generator_interface(&simple_gait_generator)` to `MotionManager`, and prove the selected interface through a host integration test that wraps `motion_manager_init`.

**Tech Stack:** C11 Firmware, GCC host runner, STM32 HAL stubs, CMake/Ninja ARM project, Qt 6/C++20 Console CTest.

---

### Task 1: Add the failing app-main backend regression

**Files:**
- Create: `RoboBeetleFirmware/tests/app_main_backend_tests.c`
- Modify: `RoboBeetleFirmware/tests/run_host_tests.ps1`

- [ ] **Step 1: Add a wrapper-based integration test before changing the default.**

Create a host executable that provides the existing minimal HAL stubs, defines
`__wrap_motion_manager_init()` to capture the `gait_generator_t` passed by
`app_main_init()`, and checks the captured interface against the public
SimpleGait interface. The test must call the real `app_main_init()` and use
the following assertions after initialization:

```c
simple_gait_generator_t expected_simple;
simple_gait_generator_init(&expected_simple);
const gait_generator_t simple_interface =
    simple_gait_generator_interface(&expected_simple);

expect(motion_manager_init_calls == 1U,
       "app_main must initialize exactly one MotionManager generator");
expect(captured_generator.ops == simple_interface.ops,
       "default bench app_main backend must be SimpleGait");
expect(captured_generator.context != NULL,
       "selected SimpleGait context must be non-null");

captured_generator.ops->advance(
    captured_generator.context,
    500U);
expect(captured_generator.ops->sample(
           captured_generator.context,
           MOTION_FORWARD,
           1.0F,
           1.0F,
           &targets),
       "selected bench generator must sample Forward");
expect(targets.front_right_cdeg == 1000 &&
           targets.front_left_cdeg == 1000 &&
           targets.rear_right_cdeg == -1000 &&
           targets.rear_left_cdeg == -1000,
       "selected bench generator must preserve SimpleGait Forward anti-phase");
```

The wrapper must not call the real `motion_manager_init()`; this test only
verifies the interface selected by `app_main_init()` and does not process
Motion commands. Link it with the same App/Communication/Motion/Servo/Safety/
Sensor sources as `app_main_depth_telemetry_tests`, `-lm`, and
`-Wl,--wrap=motion_manager_init`.

- [ ] **Step 2: Register the test in the host runner and run it against the current default.**

Add an `app_main_backend_tests` case to `RoboBeetleFirmware/tests/run_host_tests.ps1`
using the real `Core/App/app_main.c` and its existing dependencies. Do not
pass `MOTION_DEFAULT_GAIT_BACKEND_CPG` for this case so it tests the source
default. Run:

```powershell
$firmwareBuildRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("robobeetle-firmware-simple-gait-red-" + [guid]::NewGuid().ToString("N"))
powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\run_host_tests.ps1 -BuildRoot $firmwareBuildRoot
```

Expected RED result before production change: the new executable reports that
the captured default interface is not SimpleGait because `app_main.c` still
defaults to `MOTION_DEFAULT_GAIT_BACKEND_CPG 1`.

### Task 2: Switch only the normal bench default and add evidence

**Files:**
- Modify: `RoboBeetleFirmware/Core/App/app_main.c`
- Create: `docs/simple-gait-bench-remap-2026-09-14.md`
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `docs/cpg-gait-core.md`

- [ ] **Step 1: Change the fallback default without changing the selector branch.**

Replace only the fallback macro and add a comment immediately above it:

```c
#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
/* Diagnostic bench image: isolate mechanical remap with SimpleGait. */
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 0
#endif
```

Do not change `simple_gait_generator.c`, `cpg_core.c`,
`cpg_gait_generator.c`, MotionManager, Servo calibration, Protocol V2, or Qt.

- [ ] **Step 2: Record explicit backend evidence.**

Add a build note containing exactly:

```text
Motion backend: SimpleGait (bench-remap verification)
```

Document that the default source value is `0`, that
`-DMOTION_DEFAULT_GAIT_BACKEND_CPG=1` explicitly selects CPG for a later CPG
verification image, that the CPG benchmark remains separately controlled and
OFF by default, and that this is not a CPG retirement or physical verification
result. Link the note from the active Firmware Motion section and clarify the
temporary bench exception in the CPG documentation.

- [ ] **Step 3: Run the new regression and the focused SimpleGait/Motion tests.**

Run the host runner again and confirm the new backend executable passes, the
SimpleGait and MotionManager tests pass, and the compile-contract matrix still
passes for selector values `1` and `0` with benchmark values `0` and `1`.

### Task 3: Full verification and PR handoff

**Files:**
- Test: all files selected by `RoboBeetleFirmware/tests/run_host_tests.ps1`
- Test: all Qt CTest targets under a new clean build directory

- [ ] **Step 1: Run the complete Firmware regression.**

```powershell
$firmwareBuildRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("robobeetle-firmware-simple-gait-final-" + [guid]::NewGuid().ToString("N"))
powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\run_host_tests.ps1 -BuildRoot $firmwareBuildRoot
```

Expected: all existing executables plus `app_main_backend_tests` pass; all CPG
tests remain present and pass; the output identifies the expanded executable
count and the four backend/benchmark compile-contract combinations.

- [ ] **Step 2: Run the clean Qt build and CTest.**

```powershell
$qtBin = 'D:\Qt\6.11.2\mingw_64\bin'
$mingwBin = Split-Path (Get-Command g++.exe).Source
$env:Path = "$mingwBin;$qtBin;$env:Path"
$qtBuildRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("robobeetle-qt-simple-gait-" + [guid]::NewGuid().ToString("N"))
cmake -S .\RoboBeetleConsole -B $qtBuildRoot -G 'MinGW Makefiles' -DCMAKE_PREFIX_PATH='D:\Qt\6.11.2\mingw_64' -DBUILD_TESTING=ON
cmake --build $qtBuildRoot
ctest --test-dir $qtBuildRoot --output-on-failure
```

Expected: clean configure/build and 6/6 Qt tests pass. No Qt source or layout
file may be changed for backend evidence.

- [ ] **Step 3: Audit scope, ARM boundary, and commit.**

Run `git diff --check`, confirm the only production behavior change is the
`app_main.c` default, confirm no CPG source or current Servo calibration file
changed, and record `arm-none-eabi-gcc` status without changing `double` or
claiming target timing. Commit all implementation/spec/plan/docs/tests as one
commit with:

```text
test: switch bench gait baseline to simple generator
```

- [ ] **Step 4: Push the existing PR branch and stop.**

```powershell
git push origin feature/cpg-gait-core
gh pr view 15 --repo Mr-lxd/RoboBeetle --json number,state,isDraft,headRefName,headRefOid,url
```

Do not create a PR, merge, reset, clean, or force-push. Confirm PR #15 remains
OPEN and report the required hardware checklist as Pending. Finish with:

```text
READY FOR SIMPLE-GAIT HARDWARE REMAP VERIFICATION
```
