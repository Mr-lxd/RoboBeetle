# Front Paddle Calibration Sign Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reverse only the FrontRight and FrontLeft logical angle calibration endpoints after the 180-degree front assembly installation, while preserving the existing physical channel remap and all gait semantics.

**Architecture:** Keep `SimpleGait/CPG -> MotionManager -> ServoService` unchanged. Apply the correction in the Firmware and Qt descriptor calibration tuples, where logical signed angles are converted to physical PWM values; keep independent ascending raw bounds and the existing signed-delta interpolation.

**Tech Stack:** C11 Firmware host runner, STM32 HAL binding tests, Qt 6/C++20 Console CTest, Markdown documentation, GitHub CLI.

---

### Task 1: Freeze the approved calibration contract in tests

**Files:**
- Modify: `RoboBeetleFirmware/tests/servo_descriptor_tests.c:82-100`
- Modify: `RoboBeetleFirmware/tests/servo_calibration_tests.c:94-98`
- Modify: `RoboBeetleFirmware/tests/protocol_dispatcher_tests.c:329-397`
- Modify: `RoboBeetleConsole/tests/servo_descriptor_tests.cpp:120-145`

- [ ] **Step 1: Change Firmware descriptor expectations first.**

Require these exact Firmware values before touching production descriptors:

```c
expect_calibration(&table[SERVO_ID_FRONT_RIGHT],
                   1140U, 1580U, 2020U, -4500, 4500,
                   1140U, 2020U, -4500, 4500);
expect_calibration(&table[SERVO_ID_FRONT_LEFT],
                   1900U, 1450U, 1000U, -4500, 4500,
                   1000U, 1900U, -4500, 4500);
```

Keep the existing channel assertions: FrontRight `SERVO_CHANNEL_2`, FrontLeft
`SERVO_CHANNEL_1`, and FrontAxis `SERVO_CHANNEL_3`.

- [ ] **Step 2: Change Firmware interpolation expectations.**

Set the paddle cases to:

```c
{SERVO_ID_FRONT_RIGHT, 1140U, 1580U, 2020U, 1360U, 1800U},
{SERVO_ID_FRONT_LEFT, 1900U, 1450U, 1000U, 1675U, 1225U},
```

The five tested points are then `-4500 -> min`, `-2250 -> negative_mid`,
`0 -> neutral`, `2250 -> positive_mid`, and `4500 -> max`. Leave FrontAxis
midpoints and both rear cases unchanged.

- [ ] **Step 3: Change the Protocol V2 angle regression.**

In `test_servo_commands`, keep the same semantic ID 0, wire payloads, and
neutral. Change the expected angle outputs to:

```c
positive +4500 cdeg -> 2020U
negative -4500 cdeg -> 1140U
neutral 0 cdeg      -> 1580U
```

Keep raw Set PWM and all Protocol IDs unchanged.

- [ ] **Step 4: Change the Qt descriptor expectations.**

Require `FrontRight` electrical tuple `1140, 1580, 2020` with command range
`1140..2020`, and `FrontLeft` electrical tuple `1900, 1450, 1000` with command
range `1000..1900`. Keep IDs, masks, names, display labels, angle limits,
FrontAxis, and rear rows unchanged.

- [ ] **Step 5: Run the RED tests before production edits.**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\run_host_tests.ps1
$qtBin = 'D:\Qt\6.11.2\mingw_64\bin'
$mingwBin = Split-Path (Get-Command g++.exe).Source
$env:Path = "$mingwBin;$qtBin;$env:Path"
$redQtBuildRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("robobeetle-qt-front-calibration-red-" + [guid]::NewGuid().ToString("N"))
cmake -S .\RoboBeetleConsole -B $redQtBuildRoot -G 'MinGW Makefiles' -DCMAKE_PREFIX_PATH='D:\Qt\6.11.2\mingw_64' -DBUILD_TESTING=ON
cmake --build $redQtBuildRoot --target servo_descriptor_tests
ctest --test-dir $redQtBuildRoot -R servo_descriptor_tests --output-on-failure
```

Expected RED result: Firmware descriptor/calibration/Protocol assertions and
the Qt descriptor assertions report the old front tuples. No production source
has been changed at this point.

### Task 2: Apply the minimal Firmware calibration fix

**File:** `RoboBeetleFirmware/Core/Servo/servo_descriptor.c:15-61`

- [ ] **Step 1: Replace only the two front calibration records.**

Use these production records and retain their existing channels:

```c
.calibration = {
    .min_pulse_us = 1140U,
    .neutral_pulse_us = 1580U,
    .max_pulse_us = 2020U,
    .min_angle_cdeg = -4500,
    .max_angle_cdeg = 4500,
},
```

for `SERVO_ID_FRONT_RIGHT`, and:

```c
.calibration = {
    .min_pulse_us = 1900U,
    .neutral_pulse_us = 1450U,
    .max_pulse_us = 1000U,
    .min_angle_cdeg = -4500,
    .max_angle_cdeg = 4500,
},
```

for `SERVO_ID_FRONT_LEFT`. Keep command bounds `1140..2020` and `1000..1900`,
all timer/channel selectors, FrontAxis, and rear rows byte-for-byte otherwise
unchanged.

- [ ] **Step 2: Run focused Firmware GREEN tests.**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\RoboBeetleFirmware\tests\run_host_tests.ps1
```

Expected: all Firmware executables and compile-contract objects pass, including
Servo descriptor/calibration/Service, STM32 binding, Protocol, SimpleGait,
MotionManager, CPG, and Safety tests.

### Task 3: Apply Qt descriptor parity

**Files:**
- Modify: `RoboBeetleConsole/src/robot/ServoDescriptor.cpp:10-47`
- Test: `RoboBeetleConsole/tests/servo_descriptor_tests.cpp`

- [ ] **Step 1: Replace the two Qt electrical tuples.**

Set FrontRight fields to `1140, 1580, 2020, -4500, 4500, 1140, 2020,
-4500, 4500` and FrontLeft fields to `1900, 1450, 1000, -4500, 4500,
1000, 1900, -4500, 4500`. Do not edit semantic names, IDs, masks, FrontAxis,
or rear descriptors.

- [ ] **Step 2: Run a clean Qt configure/build/CTest.**

Use MinGW before Qt's bundled runtime in PATH:

```powershell
$qtBin = 'D:\Qt\6.11.2\mingw_64\bin'
$mingwBin = Split-Path (Get-Command g++.exe).Source
$env:Path = "$mingwBin;$qtBin;$env:Path"
$qtBuildRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("robobeetle-qt-front-calibration-" + [guid]::NewGuid().ToString("N"))
cmake -S .\RoboBeetleConsole -B $qtBuildRoot -G 'MinGW Makefiles' -DCMAKE_PREFIX_PATH='D:\Qt\6.11.2\mingw_64' -DBUILD_TESTING=ON
cmake --build $qtBuildRoot
ctest --test-dir $qtBuildRoot --output-on-failure
```

Expected: configure/build exit 0 and all six Qt tests pass.

### Task 4: Record Stage 1 and Stage 2 evidence

**Files:**
- Modify: `docs/front-assembly-remap-2026-09-14.md`
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `RoboBeetleConsole/README.md`
- Modify: `RoboBeetleConsole/docs/protocol.md`
- Create: `docs/superpowers/specs/2026-09-14-front-paddle-calibration-fix-design.md`
- Create: `docs/superpowers/plans/2026-09-14-front-paddle-calibration-fix.md`

- [ ] **Step 1: Update all active current-contract tuples.**

Change current FrontRight references to `-45/0/+45 -> 1140/1580/2020 us` and
current FrontLeft references to `-45/0/+45 -> 1900/1450/1000 us`, while keeping
raw bounds ascending (`1140..2020`, `1000..1900`). Update the Protocol V2
descriptor and Set Angle prose to the same values. Do not rewrite historical
pre-remap sections.

- [ ] **Step 2: Add the explicit evidence chain.**

The remap document must state: Stage 1 swapped only physical channels and
still produced physical Front/Rear same-direction motion under SimpleGait;
the observed result established a second angle-coordinate inversion; Stage 2
reversed only the two front logical calibration endpoint pairs. Mark the
software contract as Software Verified after tests, but keep physical remap,
sign, Forward/Turn, and FrontAxis re-verification pending. Keep water effect
**Pending Water Verification**.

### Task 5: Full verification, one commit, and existing PR push

- [ ] **Step 1: Run the full Firmware and Qt gates.**

Run the complete Firmware runner, the clean Qt configure/build/CTest command,
the existing 34-header self-sufficiency check with host C11 `-Wall -Wextra
-Werror`, and `git diff --check`. Report the exact counts and any failure.

- [ ] **Step 2: Check target-tool availability without changing numeric types.**

Run `Get-Command arm-none-eabi-gcc.exe` and
`Get-Command arm-none-eabi-size.exe`. If either is absent, record `NOT_FOUND`
and make no ARM FLASH/RAM/timing claim; do not change production `double` or
the CPG backend.

- [ ] **Step 3: Audit the final scope.**

Confirm no diff in `RoboBeetleFirmware/Core/Motion`, Protocol ID definitions,
Qt UI files, `simple_gait_generator.c`, `cpg_core.c`, or
`cpg_gait_generator.c`; preserve the original checkout's untracked
`RoboBeetleFirmware/daplink.cfg` and leak plan.

- [ ] **Step 4: Commit and push once.**

Stage only the files listed in this plan and commit:

```powershell
git commit -m "fix: reverse front paddle calibration after rotated install"
git push origin feature/cpg-gait-core
```

Re-query PR #15 and confirm it remains OPEN, non-draft, and unmerged with the
new head SHA. Finish with `READY FOR SIMPLE-GAIT FRONT REMAP RE-VERIFICATION`.
