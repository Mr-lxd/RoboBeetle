# CPG target performance evidence and STM32F407 DWT runbook

## Evidence status

The production CPG core remains `double` throughout its model parameters,
states, intermediate arithmetic, and logical-output conversion. The historical
source defines `real_T` as `double`; STM32F407 single-precision FPU hardware
does not authorize changing this source-compatible implementation to `float`.

### Historical CPG desktop physical evidence — 2026-09-14

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

The CPG model contract remains source-compatible: `beta=0.75`, the phase-coupling factor `2*beta*(1-beta)`, Forward Euler `dt=0.01`, exact `theta_dot` semantics, signed target amplitudes, `double` precision, semantic node mapping, Motion operational guards, and Servo calibration. At startup CPG (`=1`) remains the default, while Firmware registers SimpleGait (`0`), CPG (`1`), and ExperimentalFlex (`2`) for runtime selection while STOPPED. This report is a CPG-only performance record; current selector and coordination behavior is documented in the [Firmware README](../RoboBeetleFirmware/README.md).

### Target evidence matrix

The following PR #15 values are historical real STM32F407 target evidence. They are recorded exactly and remain specific to that earlier benchmark image. The 2026-09-24 current-image build, memory, programming, and verify results are in [the hardware acceptance document](experimental-flex-gait-coordination-hardware-acceptance-2026-09-24.md). The run used
`SystemCoreClock=16,000,000 Hz` and `repetitions=32`.

| Evidence | Status | Recorded result |
| --- | --- | --- |
| ARM firmware build | **[ARM Build: PASS]** | STM32F407 target build completed |
| target clock | **[Measured target evidence]** | `SystemCoreClock=16,000,000 Hz` |
| benchmark ELF FLASH | **[Measured target evidence]** | `55060 B / 512 KB = 10.50%` |
| benchmark ELF RAM | **[Measured target evidence]** | `5440 B / 128 KB = 4.15%` |
| FLASH delta | **[Not supplied]** | No same-configuration pre-CPG baseline size was supplied; the absolute benchmark ELF size above is retained |
| RAM delta | **[Not supplied]** | No same-configuration pre-CPG baseline size was supplied; the absolute benchmark ELF size above is retained |
| nominal 10 ms CPG substep | **[Measured target evidence]** | `3071 / 3072 / 3076 us` min/median/max; cycles `49149 / 49153 / 49217` min/median/max |
| 20 ms catch-up | **[Measured target evidence]** | `6127 / 6127 / 6131 us` min/median/max |
| 70 ms catch-up | **[Measured target evidence]** | `25504 / 25508 / 25510 us` min/median/max |
| 100 ms catch-up | **[Measured target evidence]** | `39436 / 39440 / 39442 us` min/median/max |
| worst bounded catch-up | **[Measured target evidence]** | `478438 / 478446 / 478453 us` min/median/max for 100 executed substeps from 1000 ms input |
| nominal isolated CPG deadline | **[PASS]** | `3072 us < 10000 us`; median compute utilization `30.72%` |
| system-level foreground timing margin | **[Pending separate investigation]** | UART, sensor, protocol, and foreground jitter are not included in this isolated benchmark |
| Program Verify | **[Not supplied]** | No independent DAP/ST-LINK program-verification record was supplied |

Catch-up cycle fields other than the nominal row were not included in the
supplied evidence and are intentionally not reconstructed from microseconds.

### PR #15 target-performance conclusion

The nominal 10 ms CPG isolated target compute deadline is **PASS** because
`3072 us < 10000 us`. The nominal median compute utilization at the measured
16 MHz clock is **30.72%**.

This is an isolated CPG compute result. It does not establish a large
system-level real-time margin: UART, sensor, protocol, and foreground work are
outside the measurement. System-level foreground timing margin requires a
separate scheduler/UART jitter investigation. Servo-stutter investigation is
explicitly deferred.

The `1000 ms` bounded case is not a normal operating load. Its median is
`478446 us`, and a foreground gap beyond the heartbeat/liveness contract must
be rejected by SafetySupervisor/liveness before MotionManager can perform
actuator catch-up.

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

Therefore this checkout did not independently reproduce the ARM image size,
DWT result, or Program Verify record during this documentation-only closeout.
The recorded target values above remain the supplied STM32F407 evidence. The
production core remains `double`; if a future clock or target run fails its
approved budget, that must start a separate double-reference/float-production
parity design rather than changing PR #15.
