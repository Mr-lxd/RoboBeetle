# CPG target performance evidence and STM32F407 DWT runbook

## Evidence status

The production CPG core remains `double` throughout its model parameters,
states, intermediate arithmetic, and logical-output conversion. The historical
source defines `real_T` as `double`; STM32F407 single-precision FPU hardware
does not authorize changing this source-compatible implementation to `float`.

### Latest CPG desktop physical evidence — 2026-09-14

The current Firmware image used the normal CPG backend default
`MOTION_DEFAULT_GAIT_BACKEND_CPG=1`. The user completed a real desktop hardware
exercise with these results:

| Check | Status |
| --- | --- |
| Forward | **[Hardware Verified]** |
| Front pair synchrony | **[Hardware Verified]** |
| Rear pair synchrony | **[Hardware Verified]** |
| Front versus Rear physical opposite motion | **[Hardware Verified]** |
| Turn Left | **[Hardware Verified]** |
| Turn Right | **[Hardware Verified]** |
| Ascend mechanical direction | **[Hardware Verified]** |
| Descend mechanical direction | **[Hardware Verified]** |
| Stop | **[Hardware Verified]** |
| Disable All | **[Hardware Verified]** |
| CPG desktop physical gait | **[Hardware Verified]** |

The steady-state CPG motion looked similar to the SimpleGait sinusoidal motion
by eye. That is expected and non-blocking: visual similarity is not a claim of
numerical identity, and it is not a reason to modify the CPG.

The evidence boundary remains explicit:

- True water propulsion: **[Pending Water Verification]**.
- Turn hydrodynamic effectiveness: **[Pending Water Verification]**.
- Ascend/Descend hydrodynamics: **[Pending Water Verification]**.

### Frozen production contract

The following are unchanged and must remain source-compatible: `beta=0.75`,
the phase-coupling factor `2*beta*(1-beta)`, Forward Euler `dt=0.01`, exact
`theta_dot` semantics, phase coupling, signed target amplitudes, `double`
precision, semantic node mapping, Motion operational guards, and Servo
calibration. The normal backend is CPG (`=1`); explicit `=0` selects the
retained SimpleGait baseline; each build registers exactly one generator/output
path. This PR does not add a Qt runtime backend selector or Protocol V2
trajectory telemetry.

### Target evidence matrix

The target measurements below are **[Pending User STM32F407 Run]**. No host
executable time, visual desktop result, or unconnected debugger estimate may
populate them.

| Evidence | Status | Required direct evidence |
| --- | --- | --- |
| ARM firmware build | **[Pending User STM32F407 Run]** | Successful STM32F407 build with captured toolchain and linker configuration |
| FLASH delta | **[Pending User STM32F407 Run]** | Same-configuration baseline/CPG `text`, `data`, and `bss` comparison |
| RAM delta | **[Pending User STM32F407 Run]** | Same-configuration baseline/CPG `data + bss` comparison |
| nominal 10 ms CPG substep | **[Pending User STM32F407 Run]** | DWT cycle and microsecond report |
| 20 ms catch-up | **[Pending User STM32F407 Run]** | DWT report for two source-equivalent substeps |
| 70 ms catch-up | **[Pending User STM32F407 Run]** | DWT report for seven source-equivalent substeps |
| 100 ms catch-up | **[Pending User STM32F407 Run]** | DWT report for ten source-equivalent substeps |
| worst bounded catch-up | **[Pending User STM32F407 Run]** | DWT report for 100 executed substeps from 1000 ms input |
| real-time budget decision | **[Pending User STM32F407 Run]** | Measured worst case compared with the project budget; no threshold is assumed here |

## DWT benchmark implementation

When `ROBOBEETLE_CPG_TARGET_BENCHMARK=ON`, the firmware includes
`RoboBeetleFirmware/Core/App/cpg_target_benchmark.c`. The option is OFF by
default, so the normal firmware image does not add the benchmark path. The
implementation uses the STM32F407 DWT `CYCCNT` counter, enables trace and
instruction barriers, performs one warm-up operation, and records 32 repeated
measurements for every case. It reports minimum, upper-median, and maximum
cycles plus integer microseconds converted with `SystemCoreClock`.

The report is the volatile
`cpg_target_benchmark_report` structure. It is observed by a debugger or later
target harness; it is not Protocol V2 telemetry and it does not change the
runtime control path. The benchmark contains no UART printing. Every sample
restores the initial generator state before the next measurement, and the
benchmark restores the caller's generator state before returning.

| Report case | Call | Executed 10 ms steps |
| --- | --- | ---: |
| nominal | `cpg_gait_generator_advance(generator, 10U)` | 1 |
| catch-up 20 | `cpg_gait_generator_advance(generator, 20U)` | 2 |
| catch-up 70 | `cpg_gait_generator_advance(generator, 70U)` | 7 |
| catch-up 100 | `cpg_gait_generator_advance(generator, 100U)` | 10 |
| bounded | `cpg_gait_generator_advance(generator, 1000U)` | 100, existing cap applied |

The bounded case is the worst bounded catch-up operation for this contract.
A stale foreground gap beyond the heartbeat/liveness deadline is not allowed
to reach it: SafetySupervisor/liveness must abort before MotionManager invokes
the generator.

## STM32CubeIDE target runbook

Run the command-line portions from an STM32CubeIDE terminal at the repository
root, or use the same cache variables in the STM32CubeIDE CMake configuration.
Use a separate build directory for every configuration. Record the branch
commit SHA, board revision, toolchain version, CMake generator, build type and
optimization, linker script, and `SystemCoreClock` with every result.

### A. Normal firmware build — benchmark OFF

First check that the target tools are available:

```powershell
Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue | Select-Object Source
Get-Command arm-none-eabi-size.exe -ErrorAction SilentlyContinue | Select-Object Source
arm-none-eabi-gcc --version
```

If either command is `NOT_FOUND`, stop the target run and leave ARM, size, and
DWT evidence Pending. Do not substitute host timing.

Configure and build the normal CPG image:

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

In STM32CubeIDE, create/select an ST-LINK STM32 Cortex-M debug launch for the
resulting `RoboBeetleFirmware.elf`, program the image, and require the IDE's
verification result to be PASS. Record the exact `arm-none-eabi-size` line:

```text
text data bss dec hex
```

For a FLASH/RAM delta, build an agreed pre-CPG baseline ref in a separate
checkout or external worktree using the same compiler, linker script, CMake
generator, build type, and generated-source configuration. Do not checkout the
baseline over this worktree. Calculate only:

```text
FLASH = text + data
RAM   = data + bss
FLASH_delta = FLASH_cpg - FLASH_baseline
RAM_delta   = RAM_cpg - RAM_baseline
```

When the baseline denominator is nonzero, also record
`100 * delta / baseline` for each resource. A host executable size is not ARM
FLASH/RAM evidence.

### B. DWT benchmark build — benchmark ON

Use the same target configuration and a new build directory:

```powershell
$benchmarkBuild = Join-Path $env:TEMP 'robobeetle-cpg-dwt-target'
cmake -S RoboBeetleFirmware -B $benchmarkBuild -G Ninja `
  "-DCMAKE_BUILD_TYPE=Debug" `
  "-DCMAKE_TOOLCHAIN_FILE=$toolchain" `
  "-DROBOBEETLE_CPG_TARGET_BENCHMARK=ON"
cmake --build $benchmarkBuild
arm-none-eabi-size $benchmarkBuild\RoboBeetleFirmware.elf
```

In STM32CubeIDE, select the benchmark ELF in the same ST-LINK launch, program
and verify it, then start Debug. The benchmark runs once during
`app_main_init()` before the main loop. Set a breakpoint immediately after the
`app_main_init()` call in `RoboBeetleFirmware/Core/Src/main.c`, let startup
reach that point, and inspect the volatile `cpg_target_benchmark_report` in
Watch/Expressions. Do not trigger it through a live protocol command and do
not add UART logging to the measured path.

Record these identity fields:

```text
enabled
system_core_clock_hz
repetitions
```

Record every cycle field and its microsecond companion:

```text
nominal_min_cycles          nominal_median_cycles          nominal_max_cycles
nominal_min_us              nominal_median_us              nominal_max_us

catch_up_20_min_cycles      catch_up_20_median_cycles      catch_up_20_max_cycles
catch_up_20_min_us          catch_up_20_median_us          catch_up_20_max_us

catch_up_70_min_cycles      catch_up_70_median_cycles      catch_up_70_max_cycles
catch_up_70_min_us          catch_up_70_median_us          catch_up_70_max_us

catch_up_100_min_cycles     catch_up_100_median_cycles     catch_up_100_max_cycles
catch_up_100_min_us         catch_up_100_median_us         catch_up_100_max_us

bounded_min_cycles          bounded_median_cycles          bounded_max_cycles
bounded_min_us              bounded_median_us              bounded_max_us
```

The bounded row must be reported as 100 executed 10 ms substeps from the
1000 ms input, with the existing maximum-catch-up behavior. Preserve the cycle
fields alongside the microsecond fields; do not report only converted time.

## Performance acceptance arithmetic

Do not hard-code an unapproved pass/fail threshold in this document. Once the
target report exists, calculate and show:

```text
CPG nominal compute budget:
nominal_median_us / 10000 us = utilization
utilization_percent = 100 * nominal_median_us / 10000

worst_max_us = max(
    nominal_max_us,
    catch_up_20_max_us,
    catch_up_70_max_us,
    catch_up_100_max_us,
    bounded_max_us)
```

Report the bounded maximum separately because it is a catch-up workload, not a
nominal tick. ChatGPT will decide whether the measured result leaves enough
foreground headroom after the user supplies real target numbers. Until then,
the timing decision remains **[Pending]**.

## Current unavailable boundary

The availability check in this Codex environment is:

```text
arm-none-eabi-gcc.exe=NOT_FOUND
arm-none-eabi-size.exe=NOT_FOUND
```

Therefore no ARM image size, DWT cycle/microsecond result, Program Verify
result, or target real-time conclusion is claimed here. The production core
remains `double`; a failure to meet the eventual budget must start a separate
double-reference/float-production parity design rather than changing this PR.
