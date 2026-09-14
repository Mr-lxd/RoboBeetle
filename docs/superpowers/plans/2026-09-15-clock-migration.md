# Clock Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate the STM32F407 firmware from the current HSI-direct 16 MHz clock to the approved initial HSI-to-PLL 168 MHz candidate while preserving the 1 us servo timer contract, UART protocols, millisecond software time, safety ordering, and double-precision CPG behavior.

**Architecture:** Keep HSI as the only oscillator input for this first migration: HSI 16 MHz -> PLLM=16, PLLN=336, PLLP=2, PLLQ=7 -> SYSCLK/HCLK 168 MHz. Use APB1 DIV4 and APB2 DIV2, producing PCLK1=42 MHz, PCLK2=84 MHz, and 84 MHz TIM3/TIM4 timer clocks. Change only the timer prescaler needed to retain a 1 MHz timer counter; leave CPG, gait semantics, ServoCalibration, UART payloads, and Motion/Safety policy unchanged.

**Tech Stack:** STM32F407VET6 C11 firmware, STM32F4 HAL/CMSIS, CMake/Ninja, arm-none-eabi-gcc, existing PowerShell host-test runner, DWT CYCCNT, DAP/ST-LINK debugger, and external logic-analyzer/oscilloscope measurements.

---

## Scope and dependency boundary

This branch is codex/clock-migration, based on PR #15 reviewed head b7e2787. PR #15 remains the first integration dependency; the final clock PR must target the appropriate post-PR-15 base so CPG evidence and clock changes remain separately reviewable. This plan does not modify the CPG equations, double types, semantic adapter, ServoCalibration, installed limits, gait semantics, UART protocol settings, or the PR #15 evidence commit.

### Task 1: Add a failing clock and peripheral contract test

Files:
- Create: RoboBeetleFirmware/tests/clock_config_contract_tests.ps1
- Modify: RoboBeetleFirmware/tests/run_host_tests.ps1
- Test: RoboBeetleFirmware/tests/clock_config_contract_tests.ps1

- [x] Step 1: Write the source/config assertions before changing firmware.

The new script must read RoboBeetleFirmware/Core/Src/main.c and RoboBeetleFirmware/RoboBeetleFirmware.ioc, fail on the current HSI-direct configuration, and require these exact contract strings after migration:

    $ErrorActionPreference = 'Stop'

    $firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    $main = Get-Content -Raw -LiteralPath (Join-Path $firmwareRoot 'Core\Src\main.c')
    $ioc = Get-Content -Raw -LiteralPath (Join-Path $firmwareRoot 'RoboBeetleFirmware.ioc')

    function Assert-Contains([string]$text, [string]$needle) {
        if (-not $text.Contains($needle)) {
            throw ('Missing clock contract: ' + $needle)
        }
    }

    @(
        'RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;',
        'RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;',
        'RCC_OscInitStruct.PLL.PLLM = 16;',
        'RCC_OscInitStruct.PLL.PLLN = 336;',
        'RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;',
        'RCC_OscInitStruct.PLL.PLLQ = 7;',
        'RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;',
        'RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;',
        'RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;',
        'RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;',
        'HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5)',
        'htim3.Init.Prescaler = 83;',
        'htim4.Init.Prescaler = 83;',
        'huart1.Init.BaudRate = 9600;',
        'huart3.Init.BaudRate = 9600;',
        'huart6.Init.BaudRate = 115200;'
    ) | ForEach-Object { Assert-Contains $main $_ }

    if ($main.Contains('RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;')) {
        throw 'PLL must not remain disabled'
    }

    @(
        'RCC.AHBFreq_Value=168000000',
        'RCC.APB1Freq_Value=42000000',
        'RCC.APB2Freq_Value=84000000',
        'RCC.CortexFreq_Value=168000000',
        'RCC.PLLCLKFreq_Value=168000000',
        'RCC.PLLQCLKFreq_Value=48000000',
        'RCC.VCOInputFreq_Value=1000000',
        'RCC.VCOOutputFreq_Value=336000000',
        'RCC.SYSCLKFreq_VALUE=168000000'
    ) | ForEach-Object { Assert-Contains $ioc $_ }

    $timerKernelHz = 84000000
    $prescaler = 83
    if (($timerKernelHz / ($prescaler + 1)) -ne 1000000) {
        throw 'TIM3/TIM4 1 us timer contract does not evaluate to 1 MHz'
    }

    Write-Host 'PASS clock_config_contract_tests'

- [x] Step 2: Run the new test against the current baseline and record the red result.

Run from RoboBeetleFirmware:

    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\clock_config_contract_tests.ps1

Expected: FAIL because the current source contains RCC_PLL_NONE, selects RCC_SYSCLKSOURCE_HSI, uses APB DIV1, and uses TIM3/TIM4 prescaler 15.

- [x] Step 3: Register the contract test in the existing host gate.

Append an invocation of clock_config_contract_tests.ps1 to run_host_tests.ps1, check LASTEXITCODE, and print the same PASS label. The existing 29 executable tests and 9 compile-contract objects must remain unchanged.

### Task 2: Implement the exact HSI-to-PLL 168 MHz clock tree

Files:
- Modify: RoboBeetleFirmware/Core/Src/main.c, SystemClock_Config
- Modify: RoboBeetleFirmware/RoboBeetleFirmware.ioc, derived RCC values
- Read-only verification: RoboBeetleFirmware/Core/Src/system_stm32f4xx.c
- Read-only verification: RoboBeetleFirmware/Core/Inc/stm32f4xx_hal_conf.h

- [x] Step 1: Replace only the active RCC configuration with this source-equivalent configuration.

The resulting SystemClock_Config() must retain PWR clock enable and PWR_REGULATOR_VOLTAGE_SCALE1, then configure:

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

The implementation must preserve the existing HAL_ERROR handling. The expected clocks are SYSCLK/HCLK=168 MHz, PCLK1=42 MHz, PCLK2=84 MHz, and PLLQ=48 MHz. HSE remains unused in this first implementation; the 25 MHz HSE_VALUE definition is not an activation of HSE.

- [x] Step 2: Synchronize only the .ioc derived clock metadata.

Set the derived values to AHB/Cortex/SYSCLK 168000000, APB1 42000000, APB2 84000000, PLLCLK 168000000, PLLQCLK 48000000, VCO input 1000000, and VCO output 336000000. Do not add an HSE requirement or change unrelated peripheral metadata.

- [ ] Step 3: Confirm device limits before target programming.

Check the STM32F407 HAL/device definitions for Scale 1 and FLASH_LATENCY_5, and confirm the actual board VDD is in the voltage range for 168 MHz. If the physical MCU, VDD, VCAP, or regulator evidence does not satisfy the F407 conditions, stop before programming and report the exact mismatch.

### Task 3: Preserve the TIM3/TIM4 servo timing contract

Files:
- Modify: RoboBeetleFirmware/Core/Src/main.c, MX_TIM3_Init
- Modify: RoboBeetleFirmware/Core/Src/main.c, MX_TIM4_Init
- Test: RoboBeetleFirmware/tests/clock_config_contract_tests.ps1
- Do not modify: RoboBeetleFirmware/Core/Servo/servo_calibration.c

- [x] Step 1: Change only both timer prescalers from 15 to 83.

At PCLK1=42 MHz and APB1 DIV4, the timer kernel clock is 84 MHz. With PSC=83, the counter is exactly 1 MHz, so one counter count remains 1 us. Keep ARR/Period=3002, PWM mode, polarity, preload, initial pulse values, all CCR writes, calibration tuples, and installed operational bounds unchanged.

- [x] Step 2: Make the contract test prove the arithmetic.

Require 84000000 / (83 + 1) = 1000000 and require both htim3 and htim4 source assignments to use 83. Do not add a runtime Servo or CPG behavior change to prove this arithmetic.

- [ ] Step 3: Verify the physical PWM period with actuator power OFF.

After programming, read back TIM3/TIM4 PSC and ARR and measure the PWM frame. The expected period remains (3002 + 1) * 1 us = 3003 us, approximately 333.0003 Hz. Stop immediately if the counter or PWM period is not proven.

### Task 4: Preserve UART and software-time contracts

Files:
- Modify: none in RoboBeetleFirmware/Core/Communication
- Test: RoboBeetleFirmware/tests/clock_config_contract_tests.ps1
- Read-only verification: RoboBeetleFirmware/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
- Read-only verification: RoboBeetleFirmware/Core/Src/stm32f4xx_it.c

- [x] Step 1: Keep all application UART settings byte-for-byte.

Preserve USART1=9600 8-N-1, USART3=9600 8-N-1, and USART6=115200 8-N-1. Preserve APC Series=9600, APC RF TRx=19200, and APC RF frequency. Do not change framing, transport, retry, or Protocol V2 behavior.

- [ ] Step 2: Verify HAL BRR inputs and initialization order.

Confirm SystemClock_Config() still runs before the three HAL_UART_Init calls. At the new clock, USART1/USART6 must use PCLK2=84 MHz and USART3 must use PCLK1=42 MHz; HAL must calculate BRR from those PCLK values with the existing oversampling-16 configuration. Read the actual BRR registers on the target before any actuator power is enabled.

- [ ] Step 3: Verify the millisecond timebase without changing deadlines.

Confirm HAL_RCC_ClockConfig() reinitializes SysTick from the new SystemCoreClock, SysTick_Handler still calls HAL_IncTick(), and HAL_GetTick() advances by 1000 ms over a measured one-second interval. Keep Motion=10 ms, the existing heartbeat/liveness timeout, STOP transition 750 ms, sensor-stale timeout, and telemetry periods unchanged. No millisecond constant may be converted to a CPU-cycle delay.

### Task 5: Run host, ARM, and safety verification before actuator power

Files:
- Test: RoboBeetleFirmware/tests/run_host_tests.ps1
- Test: RoboBeetleFirmware/tests/clock_config_contract_tests.ps1
- Read-only verification: RoboBeetleFirmware/Core/App/app_main.c
- Read-only verification: RoboBeetleFirmware/Core/Motion/motion_manager.c
- Read-only verification: RoboBeetleFirmware/Core/Safety/safety_supervisor.c

- [x] Step 1: Run the red-green host gate after the implementation.

Run:

    $testBuild = Join-Path $env:TEMP ('robobeetle-clock-host-' + (Get-Date -Format 'yyyyMMddHHmmss'))
    powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1 -BuildRoot $testBuild

Expected: all existing 29 executables, 9 app/backend/benchmark compile contracts, and PASS clock_config_contract_tests pass. CPG output, period, theta-dot, SimpleGait, calibration, and safety-before-catch-up tests must be unchanged and passing.

- [x] Step 2: Check target-tool availability; ARM build remains Pending because `arm-none-eabi-gcc` and `arm-none-eabi-size` are `NOT_FOUND`.

The following is the deferred target command; it was not run because the
required executables are unavailable in this environment:

    $toolchain = (Resolve-Path '.\cmake\gcc-arm-none-eabi.cmake').Path
    $normalBuild = Join-Path $env:TEMP ('robobeetle-clock-normal-' + (Get-Date -Format 'yyyyMMddHHmmss'))
    cmake -S . -B $normalBuild -G Ninja -DCMAKE_BUILD_TYPE=Debug ('-DCMAKE_TOOLCHAIN_FILE=' + $toolchain) -DROBOBEETLE_CPG_TARGET_BENCHMARK=OFF
    if ($LASTEXITCODE -ne 0) { throw 'ARM configure failed' }
    cmake --build $normalBuild
    if ($LASTEXITCODE -ne 0) { throw 'ARM build failed' }
    arm-none-eabi-size (Join-Path $normalBuild 'RoboBeetleFirmware.elf')

Record compiler/toolchain version, linker script, build type, optimization, text/data/bss, and FLASH/RAM deltas against the same-configuration PR #15 baseline. If arm-none-eabi-gcc is unavailable, record the exact not-run result and do not claim ARM or target performance.

- [x] Step 3: Perform the host safety-order regression; target/physical execution remains Pending.

Verify in host instrumentation and then on the target that:

    Motion active
      -> foreground gap > heartbeat/liveness timeout
      -> SafetySupervisor/liveness aborts first
      -> no post-gap CPG catch-up
      -> no post-gap Motion actuator command
      -> no automatic resume after heartbeat recovery alone

The existing app_main_process() ordering must remain SafetySupervisor before MotionManager. Do not use the 1000 ms bounded catch-up as a normal-load test; the safety contract must prevent a stale foreground gap from reaching it.

- [ ] Step 4: Program and verify with Servo actuator power OFF.

Use the established DAP/ST-LINK sequence. Before enabling any servo power, read and record SystemCoreClock==168000000, RCC PLL/APB registers, FLASH->ACR, PWR voltage scale, TIM3/TIM4 PSC/ARR, and USART1/3/6 BRR.

### Task 6: Execute the ordered physical bring-up

Files:
- Evidence: docs/clock-migration-2026-09-15.md
- Read-only reference: docs/cpg-gait-performance.md
- Read-only reference: ROBOBEETLE_HARDWARE_CONTROL_HANDOFF.md

- [ ] Step 1: Complete the no-servo-power checks in this exact order.

1. ARM build.
2. Program Verify.
3. Servo actuator power OFF.
4. Debugger read: SystemCoreClock == 168000000.
5. HAL millisecond timing.
6. TIM3/TIM4 counter frequency = 1 MHz.
7. Configured PWM period = 3003 us.
8. USART register/configuration and actual baud.
9. DAP/direct UART host link.
10. APC host link with unchanged radio settings.
11. JY901S USART3 and ROVMAKER USART6 tests.

Stop immediately on any unproven or incorrect PWM timing; do not continue to servo power.

- [ ] Step 2: Only after the preceding checks pass, exercise actuators in order.

Servo power ON, Neutral only, individual low-amplitude servo commands, SimpleGait, CPG, then the long-run test. Keep ServoCalibration, FrontAxis semantics, installed operational limits, Forward/Turn/Ascend/Descend mapping, and Backward pending/disabled unchanged. Servo-stutter investigation remains out of this clock PR unless the clock change directly reproduces it.

### Task 7: Run the 16 MHz versus 168 MHz DWT A/B benchmark

Files:
- Evidence: docs/clock-migration-2026-09-15.md
- Read-only benchmark: RoboBeetleFirmware/Core/App/cpg_target_benchmark.c
- Read-only CPG: RoboBeetleFirmware/Core/Motion/cpg_core.c

- [ ] Step 1: Build the benchmark image with the same toolchain settings.

Use a clean external build directory and -DROBOBEETLE_CPG_TARGET_BENCHMARK=ON. Program only after the normal image has passed the hardware-safe checks. Do not add UART logging or alter the measured CPG path.

- [ ] Step 2: Record both clock runs without scaling estimates.

Record SystemCoreClock, repetitions, min/median/max cycles and min/median/max microseconds for nominal 10 ms, 20 ms, 70 ms, 100 ms, and 1000 ms bounded catch-up. Record FLASH and RAM for both images and compute deltas from the same baseline. Preserve the current 16 MHz evidence exactly:

    SystemCoreClock=16,000,000 Hz; repetitions=32
    nominal 10 ms: 3071 / 3072 / 3076 us; cycles 49149 / 49153 / 49217
    20 ms: 6127 / 6127 / 6131 us
    70 ms: 25504 / 25508 / 25510 us
    100 ms: 39436 / 39440 / 39442 us
    1000 ms bounded / 100 substeps: 478438 / 478446 / 478453 us
    benchmark FLASH=55060 B / 512 KB = 10.50%
    benchmark RAM=5440 B / 128 KB = 4.15%

The new 168 MHz values must be directly measured. Do not estimate them from 16/168 scaling. The production core remains double; if the measured double implementation fails the approved budget, stop and create a separate double-reference/float-production-parity design.

- [ ] Step 3: Measure target-representative long-run oscillator period.

Keep T=2.0 s documented as a nominal period parameter. Measure actual steady-state phase crossings/frequency on the target-representative run and report its relationship to the nominal parameter; do not relabel it as exactly 0.5 Hz without evidence.

### Task 8: Final verification and review gate

Files:
- Create/update: docs/clock-migration-2026-09-15.md
- Review only: all production and evidence diffs in this branch

- [x] Step 1: Write the clock evidence report only from captured results.

The report must include exact RCC/APB/TIM/UART/SysTick configuration, target register readback, ARM size delta, ordered bring-up results, 16/168 DWT table, long-run period result, and explicit unavailable evidence. It must not upgrade water evidence, alter PR #15 CPG evidence, or claim system-level real-time margin from isolated CPG timing.

- [ ] Step 2: Run final verification before any completion claim.

Run git diff --check, the full Firmware host gate, the ARM build/size command, and the exact target/physical checks that are actually available. Confirm the only intended production changes are RCC configuration and TIM3/TIM4 PSC, and confirm no CPG, calibration, gait, UART, or safety semantic diff exists.

- [ ] Step 3: Request external review before merge.

Push the clock branch only after the plan is approved and implementation verification is complete. Do not merge PR #15 or the future clock PR in this task; preserve the dependency order CPG PR first, clock PR second.

## Approval gate

This plan was approved for implementation. Software/configuration verification is complete to the extent available in the workspace; target programming, register readback, DWT A/B, and physical bring-up remain explicitly Pending until the user performs the hardware-safe sequence.
