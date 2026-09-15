# CPG Desktop Verification and Host Trace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record the completed CPG desktop hardware verification, provide an exact STM32F407 DWT benchmark runbook, and add a deterministic host-only SimpleGait-versus-CPG CSV trace without changing production CPG mathematics, Qt runtime selection, or PR #15 integration scope.

**Architecture:** Keep the existing `cpg_target_benchmark.c` DWT/CYCCNT implementation and expose its debugger-observed report through documentation only; target values remain pending until a real STM32F407 run. Add a host-only trace library/CLI under `RoboBeetleFirmware/tests/tools` that directly invokes the existing SimpleGaitGenerator and CPGGaitGenerator, writes integer logical targets plus a post-guard projection using the canonical Motion bounds, and writes CPG internal research fields separately. Add the CLI and a deterministic file-content regression to the existing host runner; all generated files go to the runner's external build directory.

**Tech Stack:** C11, host GCC, existing Firmware production generator sources, Markdown, PowerShell host runner, STM32F407 CMSIS DWT/CYCCNT, CMake/Ninja runbook, GitHub CLI.

---

### Task 1: Freeze the desktop evidence boundary and DWT runbook

**Files:**

- Modify: `docs/cpg-gait-performance.md`
- Modify: `docs/cpg-gait-core.md`
- Modify: `docs/motion-simple-gait.md`
- Modify: `docs/simple-gait-bench-remap-2026-09-14.md`
- Modify: `docs/front-assembly-remap-2026-09-14.md`
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `RoboBeetleConsole/README.md`
- Modify: `RoboBeetleConsole/docs/protocol.md`
- Modify: `ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md`

- [x] **Step 1: Record only the user-provided CPG desktop PASS.**

State that the current CPG-default desktop image passed Forward, front-pair
synchrony, rear-pair synchrony, physical Front-versus-Rear opposite motion,
Turn Left, Turn Right, Ascend mechanical direction, Descend mechanical
direction, Stop, and Disable All. Label this **[Hardware Verified]** as desktop
physical CPG gait. Keep true water propulsion, Turn hydrodynamic effectiveness,
and Ascend/Descend hydrodynamics **[Pending Water Verification]**. State that
the visual similarity between CPG steady state and SimpleGait is expected and
non-blocking; do not imply that visual similarity proves numerical identity or
change CPG math because of it.

- [x] **Step 2: Preserve the frozen production boundary.**

Explicitly repeat that `beta=0.75`, `2*beta*(1-beta)`, Forward Euler
`dt=0.01`, exact `theta_dot` semantics, phase coupling, signed target
amplitudes, `double`, semantic node mapping, Motion operational guards, and
Servo calibration are unchanged. Keep the normal backend default
`MOTION_DEFAULT_GAIT_BACKEND_CPG=1`, explicit SimpleGait `=0`, and exactly one
generator/output path per build. Do not add a Qt runtime backend selector or
any Protocol V2 trajectory telemetry.

- [x] **Step 3: Replace ambiguous DWT status wording with an executable target runbook.**

Document two identical-configuration target builds from the repository root:

```powershell
$toolchain = (Resolve-Path 'RoboBeetleFirmware\cmake\gcc-arm-none-eabi.cmake').Path
$normalBuild = Join-Path $env:TEMP 'robobeetle-cpg-normal-target'
cmake -S RoboBeetleFirmware -B $normalBuild -G Ninja `
  "-DCMAKE_BUILD_TYPE=Debug" `
  "-DCMAKE_TOOLCHAIN_FILE=$toolchain" `
  "-DROBOBEETLE_CPG_TARGET_BENCHMARK=OFF"
cmake --build $normalBuild
arm-none-eabi-size $normalBuild\RoboBeetleFirmware.elf
```

Record toolchain version, CMake generator, build type/optimization, linker
script, `SystemCoreClock`, board revision, commit SHA, Build PASS, Program
Verify PASS, and `text data bss dec hex`. Calculate `FLASH=text+data`,
`RAM=data+bss`, and CPG-minus-baseline deltas only from the same size-tool
configuration. The normal build must be flashed and verified before the
benchmark build.

Then run the benchmark build with the same configuration and a separate
directory:

```powershell
$benchmarkBuild = Join-Path $env:TEMP 'robobeetle-cpg-dwt-target'
cmake -S RoboBeetleFirmware -B $benchmarkBuild -G Ninja `
  "-DCMAKE_BUILD_TYPE=Debug" `
  "-DCMAKE_TOOLCHAIN_FILE=$toolchain" `
  "-DROBOBEETLE_CPG_TARGET_BENCHMARK=ON"
cmake --build $benchmarkBuild
arm-none-eabi-size $benchmarkBuild\RoboBeetleFirmware.elf
```

Flash the benchmark ELF, start a debugger session, stop after the
`app_main_init()` call in `RoboBeetleFirmware/Core/Src/main.c`, and inspect the
volatile `cpg_target_benchmark_report` in Watch/Expressions. The benchmark is
already executed once during `app_main_init()`; do not add UART output or
re-run it through a live protocol command. Record `enabled`,
`system_core_clock_hz`, `repetitions`, every listed min/median/max cycle field,
and every listed min/median/max microsecond field for nominal, 20 ms, 70 ms,
100 ms, and bounded 1000 ms input. The bounded case is 100 executed 10 ms
substeps under the existing cap. Preserve the cycle fields alongside the
microseconds.

- [x] **Step 4: Document acceptance arithmetic without a threshold.**

Include the exact report-only calculation:

```text
CPG nominal compute budget:
nominal_median_us / 10000 us = utilization
utilization_percent = 100 * nominal_median_us / 10000
```

Also report the worst observed `max_us` across all five cases and identify the
bounded case separately. Do not hard-code a pass/fail threshold and do not
claim foreground headroom until actual target numbers are supplied. Keep
`arm-none-eabi-gcc.exe`/`arm-none-eabi-size.exe` unavailable status as
`NOT_FOUND` in this environment and keep all target fields pending.

### Task 2: Add a host-only trace API and CSV CLI

**Files:**

- Create: `RoboBeetleFirmware/tests/tools/gait_trace_compare.h`
- Create: `RoboBeetleFirmware/tests/tools/gait_trace_compare.c`
- Create: `RoboBeetleFirmware/tests/tools/gait_trace_compare_main.c`

- [x] **Step 1: Define the wished-for host API before implementation.**

Declare one host-only entry point and the fixed timeline constants:

```c
#define GAIT_TRACE_SAMPLE_INTERVAL_MS 10U
#define GAIT_TRACE_FINAL_TIME_MS 17000U
#define GAIT_TRACE_ROW_COUNT 1701U

int gait_trace_compare_generate(const char *output_dir);
```

The implementation must return nonzero for a null output directory, an
unopenable output file, a path-format error, a generator failure, or a write
failure. It must not be included by `RoboBeetleFirmware/CMakeLists.txt` and
must not reference Protocol V2 or UART code.

- [x] **Step 2: Implement one deterministic timeline using production APIs.**

Use half-open mode intervals at 10 ms samples: `[0,5000)` Forward,
`[5000,8000)` Turn Left, `[8000,11000)` Forward, `[11000,14000)` Turn Right,
`[14000,17000)` Forward, and `[17000,17000]` as one final STOP row. At
`time_ms=0` sample the initialized generators; for every later row advance each
generator by exactly 10 ms before sampling. Invoke
`simple_gait_generator_advance/sample()` and
`cpg_gait_generator_advance/sample()` directly. Use the same mode, amplitude
scale `1.0F`, and bias scale `1.0F` for both backends. Do not copy any CPG or
SimpleGait formula into the tool.

- [x] **Step 3: Write canonical raw-plus-guard CSV files.**

Produce `gait_trace_simple.csv` and `gait_trace_cpg.csv` with exactly 1701
data rows and a header beginning with these canonical integer target fields:

```text
time_ms,backend,mode,front_right_cdeg,front_left_cdeg,rear_right_cdeg,rear_left_cdeg,front_axis_cdeg
```

The listed generator target columns are raw logical targets. Append
`guarded_front_right_cdeg`, `guarded_front_left_cdeg`,
`guarded_rear_right_cdeg`, `guarded_rear_left_cdeg`, and
`guarded_front_axis_cdeg`. Project the paddle values with the existing
canonical bounds from `motion_config.h`: front `-4500..+2800 cdeg`, rear
`-3000..+4500 cdeg`; leave FrontAxis unchanged because it has no paddle guard.
Document that this host-only projection is for algorithm-request versus
installed-safe-command comparison and is not a replacement for the production
MotionManager sanitizer. Do not make PWM the primary curve.

- [x] **Step 4: Write the separate CPG internal research CSV.**

Produce `gait_trace_cpg_internal.csv` with the same 1701 `time_ms` rows and
columns for `mode`, `node0_phase..node3_phase`,
`node0_amplitude..node3_amplitude`, `node0_target_amplitude..node3_target_amplitude`,
`theta_dot0..theta_dot3`, and `raw_output0..raw_output3`. Use sufficient
`double` text precision for deterministic comparison. Keep this file strictly
host-only research output; do not add its states to Protocol V2 telemetry.

- [x] **Step 5: Add the CLI entry point.**

`gait_trace_compare_main.c` accepts zero or one argument, uses `.` when no
argument is supplied, calls `gait_trace_compare_generate()`, and returns a
nonzero usage/generation status on failure. On success it prints only the
output directory and the three generated filenames; no runtime firmware path
or UART is involved.

### Task 3: TDD the deterministic trace and integrate it with the host runner

**Files:**

- Create: `RoboBeetleFirmware/tests/gait_trace_compare_tests.c`
- Modify: `RoboBeetleFirmware/tests/run_host_tests.ps1`
- Modify: `docs/motion-simple-gait.md`
- Modify: `RoboBeetleFirmware/README.md`

- [x] **Step 1: Write the failing deterministic regression first.**

The test executable accepts two pre-created output directories. It calls the
generation API once in each directory, compares all three files byte-for-byte,
then checks the canonical header, backend names, 1701 rows, 10 ms time steps,
the exact mode transitions at 5000/8000/11000/14000/17000 ms, STOP zeros in the
final canonical row, and all guarded paddle values within the frozen bounds.
It also checks the internal header and row count. Compile this test with the
trace implementation and production `simple_gait_generator.c`,
`cpg_gait_generator.c`, and `cpg_core.c`, but without the CLI main.

- [x] **Step 2: Run RED and verify the failure reason.**

Add the test case to the host runner before adding the implementation or CLI.
Run it with the existing C11 warning flags and expect a compile/link failure
because `gait_trace_compare_generate()` is not defined. Fix only test/runner
wiring errors until the failure is specifically the missing trace API.

- [x] **Step 3: Implement the minimum trace API and CLI to turn GREEN.**

Add the three CSV writers and the single timeline loop described in Task 2.
Extend `Invoke-HostCase` with optional `Arguments`, pre-create external
per-case output directories under the host build root, and add two cases:
`gait_trace_compare_tests` and `gait_trace_compare_tool`. The tool case must
compile with the CLI main and receive one output directory; the test case must
receive two output directories. The tool/test sources remain under `tests` and
are never added to the STM32 CMake target.

- [x] **Step 4: Run the trace regression and the complete host suite.**

Require the deterministic trace test and CLI to pass, then require the complete
runner to report 29 executable tests plus the existing 9 app/backend/benchmark
compile-contract objects. Keep all existing CPG golden/differential, period,
safety-before-catch-up, SimpleGait, MotionManager, Servo, Safety, Protocol,
Leak, IMU, and Depth tests green.

### Task 4: Verify, document outputs, commit, and update PR #15

**Files:**

- Modify: `docs/cpg-gait-performance.md`
- Modify: the active evidence/control documents listed in Task 1
- Modify: `RoboBeetleFirmware/README.md`
- Modify: `docs/motion-simple-gait.md`
- External update: PR #15 body via `gh pr edit`

- [x] **Step 1: Generate a final external trace artifact and inspect it.**

Run `gait_trace_compare_tool` into a unique directory outside the worktree and
record the absolute paths of `gait_trace_simple.csv`, `gait_trace_cpg.csv`, and
`gait_trace_cpg_internal.csv`. Report only numerical observations supported by
the generated integer CSV and internal trace; do not infer hardware propulsion
or alter the CPG because the two traces look similar.

- [x] **Step 2: Run final verification.**

Run the complete Firmware host suite, clean Qt configure/build/CTest, 34 Core
header C11 self-sufficiency, compile contracts, `git diff --check`, and the
ARM-tool availability check. Confirm the original detached checkout still has
only its pre-existing `daplink.cfg` and leak-telemetry plan untracked. Confirm
no production CPG math, semantic adapter, ServoCalibration, Safety source, Qt
UI, Protocol IDs, or runtime backend selector changed.

- [x] **Step 3: Update the PR body and push only the existing branch.**

Use one small commit on `feature/cpg-gait-core`, push it without force, update
PR #15 with the current 29+9 host counts, CPG desktop Hardware Verified result,
DWT runbook status, CSV tool path, and pending target measurements. Re-query PR
#15 and require `OPEN`, non-draft, unmerged, with head equal to the pushed SHA.
Do not create or merge a PR.

- [x] **Step 4: Close out with the exact final phrase.**

The final response must end its substantive report with:

```text
READY FOR STM32F407 TARGET PERFORMANCE MEASUREMENT
```
