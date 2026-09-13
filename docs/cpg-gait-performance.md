# CPG target performance evidence

## Evidence status

The production CPG core remains `double` throughout its model parameters,
states, intermediate arithmetic, and logical-output conversion. The historical
source defines `real_T` as `double`, so STM32F407 single-precision FPU
availability is not a reason to change this source-compatible implementation.

The following target evidence is currently `[UNKNOWN]` because this worktree
does not have a confirmed `arm-none-eabi-gcc` toolchain invocation or a
connected STM32F407 target result:

| Evidence | Status | Required direct evidence |
| --- | --- | --- |
| ARM firmware build | `[UNKNOWN]` | Successful STM32F407 CMake/Ninja build with captured compiler and linker configuration |
| FLASH delta | `[UNKNOWN]` | Baseline and CPG image `text`, `data`, and `bss` size comparison |
| RAM delta | `[UNKNOWN]` | Baseline and CPG image `data + bss` comparison |
| nominal 10 ms CPG substep | `[UNKNOWN]` | STM32F407 DWT cycle and microsecond report |
| 20 ms catch-up | `[UNKNOWN]` | STM32F407 DWT report for two source-equivalent substeps |
| 70 ms catch-up | `[UNKNOWN]` | STM32F407 DWT report for seven source-equivalent substeps |
| 100 ms catch-up | `[UNKNOWN]` | STM32F407 DWT report for ten source-equivalent substeps |
| worst bounded catch-up | `[UNKNOWN]` | STM32F407 DWT report for the 100-step cap and discarded-step count |
| real-time budget decision | `[UNKNOWN]` | Project timing budget compared with the measured worst bounded operation |

`[UNKNOWN]` means not directly measured; it is not a pass, a failure, or a
host timing estimate. No float-production parity implementation is included
in this change. If measured double execution cannot meet the real-time budget,
that result must start a separate design for double-reference versus
float-production parity.

## Measurement implementation

When `ROBOBEETLE_CPG_TARGET_BENCHMARK=ON`, the firmware includes
`RoboBeetleFirmware/Core/App/cpg_target_benchmark.c`. The benchmark is
disabled by default and does not add DWT code to the normal firmware image.
It uses the STM32F407 DWT `CYCCNT` counter with trace enable and instruction
barriers, performs one warm-up operation, and records 32 repeated measurements
for each case. It reports minimum, upper-median, and maximum cycles plus the
integer microsecond conversion at `SystemCoreClock`.

The report is exposed through the volatile
`cpg_target_benchmark_report` structure for a debugger, target test harness,
or a later board-specific telemetry adapter. It does not add a Protocol V2
message or change the runtime control path. Each measurement restores the
generator state before the next sample; the benchmark restores the original
generator state before returning.

| Report case | Operation | Source-equivalent work |
| --- | --- | --- |
| nominal | `cpg_gait_generator_advance(generator, 10U)` | one 10 ms substep |
| catch-up 20 | `cpg_gait_generator_advance(generator, 20U)` | two 10 ms substeps |
| catch-up 70 | `cpg_gait_generator_advance(generator, 70U)` | seven 10 ms substeps |
| catch-up 100 | `cpg_gait_generator_advance(generator, 100U)` | ten 10 ms substeps |
| bounded | `cpg_gait_generator_advance(generator, 1000U)` | 100 executed substeps, with the core cap applied |

The bounded case is the worst bounded catch-up measurement for the current
contract. A 700 ms stale foreground gap is not a performance case that may
reach this benchmark path: the safety-before-catch-up contract must abort
before `MotionManager` invokes the generator.

## Build and size procedure

Run these commands from the repository root using a unique build directory
outside the worktree. Record the exact toolchain version, CMake generator,
optimization/debug flags, linker script, `SystemCoreClock`, board revision,
and firmware commit SHA with the result.

```powershell
Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue | Select-Object Source
arm-none-eabi-gcc --version
Get-Content RoboBeetleFirmware\CMakePresets.json
$targetBuild = Join-Path $env:TEMP 'robobeetle-cpg-target-build'
$toolchain = (Resolve-Path 'RoboBeetleFirmware\cmake\gcc-arm-none-eabi.cmake').Path
cmake -S RoboBeetleFirmware -B $targetBuild -G Ninja `
  "-DCMAKE_BUILD_TYPE=Debug" `
  "-DCMAKE_TOOLCHAIN_FILE=$toolchain" `
  "-DROBOBEETLE_CPG_TARGET_BENCHMARK=OFF"
cmake --build $targetBuild
arm-none-eabi-size $targetBuild\RoboBeetleFirmware.elf
```

Build an equivalent baseline image from the parent commit with the same
toolchain, flags, linker script, and generated-source configuration. The
feature image must contain the CPG sources and the baseline image must not.
Do not compare Debug to Release or change optimization between the two
images.

For each image, record the `arm-none-eabi-size` columns:

```text
text data bss dec hex
```

Calculate the deltas as:

```text
FLASH = text + data
RAM   = data + bss
FLASH_delta = FLASH_cpg - FLASH_baseline
RAM_delta   = RAM_cpg - RAM_baseline
```

When the baseline denominator is nonzero, also report:

```text
FLASH_delta_percent = 100 * FLASH_delta / FLASH_baseline
RAM_delta_percent   = 100 * RAM_delta / RAM_baseline
```

The linker map or `arm-none-eabi-nm --print-size --size-sort` may be used to
explain a delta, but the reported totals must come from the same size tool and
the same image configuration. A host executable size is not STM32 FLASH/RAM
evidence.

## Target result table

Populate the measured columns only from the target benchmark report. The
current result is intentionally left `[UNKNOWN]` until that evidence exists.

| Case | Executed steps | Min cycles | Median cycles | Max cycles | Min us | Median us | Max us | Status |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| nominal 10 ms | 1 | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` |
| catch-up 20 ms | 2 | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` |
| catch-up 70 ms | 7 | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` |
| catch-up 100 ms | 10 | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` |
| bounded catch-up | 100 | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` | `[UNKNOWN]` |

The host period result in `docs/cpg-gait-core.md` is a numerical behavior
check, not a target timing measurement. Likewise, the host test runner's
runtime is not evidence for STM32F407 execution time.

## Current unavailable boundary

The exact availability check was run in this worktree and produced:

```text
arm-none-eabi-gcc: NOT FOUND
```

The target configure command was also attempted with the repository toolchain
file in an external temporary build directory:

```powershell
$targetBuild = Join-Path $env:TEMP 'robobeetle-cpg-target-build-20260914-r2'
$toolchain = (Resolve-Path 'RoboBeetleFirmware\cmake\gcc-arm-none-eabi.cmake').Path
cmake -S RoboBeetleFirmware -B $targetBuild -G Ninja `
  "-DCMAKE_BUILD_TYPE=Debug" `
  "-DCMAKE_TOOLCHAIN_FILE=$toolchain" `
  "-DROBOBEETLE_CPG_TARGET_BENCHMARK=OFF"
```

It failed at `RoboBeetleFirmware/CMakeLists.txt:28` because
`arm-none-eabi-gcc` and `arm-none-eabi-g++` were not found in `PATH`. No ARM
image-size result, DWT timing result, or target-board result is therefore
claimed. This unavailable-toolchain boundary does not authorize changing the
production core from `double` to `float`.
