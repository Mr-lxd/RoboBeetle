# Clock migration evidence — 2026-09-15

## Status and dependency

The approved clock migration is implemented on branch `codex/clock-migration`,
based on PR #15 reviewed head `b7e2787fe9eb4fdd46de0fd7380a23bbc961cc52`.
PR #15 remains the first integration dependency; this branch is intentionally
separate and is not merged here. The source/configuration and host contract
checks are software-verified. CubeMX regeneration, ARM build, debugger
readback, DWT measurements, and physical bring-up remain **Pending** because
the CubeMX/target tools and hardware evidence are not available in this
workspace.

No servo actuator power was enabled, and no claim of Hardware Verified or Water
Verified is made by this document.

## Implemented clock tree

`SystemClock_Config()` retains HSI as the source and enables the approved PLL:

| Item | Value |
| --- | ---: |
| HSI input | 16 MHz |
| PLLM / PLLN / PLLP / PLLQ | 16 / 336 / 2 / 7 |
| SYSCLK / HCLK | 168 MHz |
| APB1 / PCLK1 | DIV4 / 42 MHz |
| APB2 / PCLK2 | DIV2 / 84 MHz |
| PLLQ clock | 48 MHz |
| Voltage scale | Scale 1 |
| Flash latency | `FLASH_LATENCY_5` |

The arithmetic is `16 MHz / 16 × 336 / 2 = 168 MHz`. HSE remains unused; the
`.ioc` HSE value is not an HSE activation.

## Regeneration-safe CubeMX `.ioc` contract

`RoboBeetleFirmware.ioc` retains the project metadata
`MxCube.Version=6.18.1` and `ProjectManager.FirmwarePackage=STM32Cube FW_F4
V1.28.3`. It now persists the actual RCC input selections, rather than only
the calculated frequency results:

| `.ioc` key | Value |
| --- | --- |
| `RCC.PLLSourceVirtual` | `RCC_PLLSOURCE_HSI` |
| `RCC.PLLM` / `RCC.PLLN` | `16` / `336` |
| `RCC.PLLP` / `RCC.PLLQ` | `RCC_PLLP_DIV2` / `7` |
| `RCC.SYSCLKSource` | `RCC_SYSCLKSOURCE_PLLCLK` |
| `RCC.AHBCLKDivider` | `RCC_SYSCLK_DIV1` |
| `RCC.APB1CLKDivider` / `RCC.APB2CLKDivider` | `RCC_HCLK_DIV4` / `RCC_HCLK_DIV2` |
| `RCC.PWR_Regulator_Voltage_Scale` | `PWR_REGULATOR_VOLTAGE_SCALE1` |
| `PCC.Vdd` | `3.3` |

The related timer-kernel, core, and frequency metadata is also persisted:
`RCC.APB1TimFreq_Value=84000000`, `RCC.APB2TimFreq_Value=168000000`,
`RCC.HCLKFreq_Value=168000000`, `RCC.FCLKCortexFreq_Value=168000000`,
`RCC.PLLQCLKFreq_Value=48000000`, and the existing VCO/PLL/SYSCLK/APB
derived values. Every RCC input key above is included in `RCC.IPParameters`.
The key spelling was cross-checked against F4 FW 1.28.3 `.ioc` examples for
the AHB/APB/PLL fields and the voltage-scale field ([F407 clock example](https://github.com/ATGXicefires/STM32_final_project/blob/fdeb2cb41e223864361e150f1cd632a02d37ffaa/NIM_Assistant_F407/NIM_Assistant_F407.ioc),
[F407 source/voltage example](https://github.com/Sonboy97/STM32F407/blob/7be643b3dd96ccc8f10c069879eea57b8349c58e/FSBL/FSBL.ioc)).

`clock_config_contract_tests.ps1` now checks both sides of the contract: the
production `main.c` assignments and exact `.ioc` key/value pairs, including
membership in `RCC.IPParameters`, voltage metadata, TIM3/TIM4 prescalers, and
all three UART baud values. It therefore cannot pass an HSI-direct or
derived-only `.ioc` beside a PLL-based `main.c`.

CubeMX 6.18.1 was not available in this workspace, so no `.ioc` Generate Code
run or generated-file comparison was performed. **CubeMX regeneration
verification = Pending user/tool run.** This document intentionally makes no
regeneration PASS claim. The pending check is to open this `.ioc` with
CubeMX 6.18.1 and STM32Cube FW_F4 V1.28.3, Generate Code, then inspect the
generated `SystemClock_Config()` and rerun the contract test.

The local STM32F407 HAL/device definitions identify Scale 1 and 5 wait states
as the configuration for the approved 168 MHz candidate. Actual board VDD,
VCAP, regulator, and debugger register readback still require the ordered
hardware-safe bring-up below.

## Preserved peripheral and time contracts

TIM3 and TIM4 remain the existing PWM peripherals with ARR/Period `3002` and
all pulse, polarity, preload, calibration, servo, and motion-limit values
unchanged. Because APB1 is prescaled, the F4 timer kernel is `2 × PCLK1 =
84 MHz`. Both prescalers are now `83`, so the counter is:

`84,000,000 / (83 + 1) = 1,000,000 Hz = 1 us/count`.

The configured PWM frame therefore remains `(3002 + 1) us = 3003 us` in the
source contract, approximately 333.0003 Hz. The counter and waveform still
need target readback and measurement with actuator power OFF.

UART application settings remain unchanged: USART1 9600 8-N-1, USART3 9600
8-N-1, and USART6 115200 8-N-1. HAL initialization still runs after
`SystemClock_Config()`, so BRR calculation uses the new PCLK2 for USART1/6 and
PCLK1 for USART3. APC Series remains 9600, APC RF TRx remains 19200, and the RF
frequency is unchanged. The SYSCLK migration does not change those radio
settings; an incorrect HAL BRR would still break the host/APC link and must be
read back on target.

`HAL_GetTick()` remains the HAL millisecond timebase. Motion10ms,
STOP750ms, heartbeat/liveness, depth stale, telemetry, and all safety
deadlines remain unchanged. `app_main_process()` continues to evaluate
SafetySupervisor/liveness before MotionManager elapsed-time catch-up. A stale
foreground gap must abort before any catch-up or Servo write and must not
auto-resume from heartbeat recovery alone.

The production CPG remains the source-compatible `double` implementation. No
CPG equation, theta-dot state, gait semantics, semantic adapter,
LogicalJointTargets, calibration, Protocol V2, Qt, or sensor protocol was
changed in this branch.

## Verification record

| Check | Result |
| --- | --- |
| Focused clock contract before implementation | RED as expected: legacy `RCC_PLL_NONE` was missing the required PLL contract |
| Focused clock contract after implementation | PASS `clock_config_contract_tests`: main.c + `.ioc` source inputs + `RCC.IPParameters` + derived values + TIM/UART contracts |
| CubeMX 6.18.1 regeneration parity | **Pending user/tool run**: CubeMX executable unavailable; no Generate Code comparison and no PASS claim |
| Firmware host gate | PASS: 29 executables + 9 app/backend/benchmark compile-contract objects, including the clock contract |
| Public-header self-sufficiency | PASS: 34 headers with host C11 `-Wall -Wextra -Werror`; CMSIS host pointer-width warnings explicitly suppressed |
| ARM compiler / `arm-none-eabi-size` | **Pending / NOT_FOUND** in this environment; no ARM image or size claim made |
| SystemCoreClock 168 MHz debugger readback | Pending hardware access |
| TIM3/TIM4 PSC/ARR and 3003 us waveform | Pending hardware access |
| HAL_GetTick one-second measurement | Pending hardware access |
| USART BRR and host/APC/JY901S/ROVMAKER links | Pending hardware access |
| 16 MHz versus 168 MHz DWT A/B | Pending target benchmark run; no 16/168 scaling estimate used |
| Program Verify | Pending |
| Water evidence | Pending Water Verification |

The available isolated 16 MHz baseline is preserved exactly for the later A/B
run: repetitions 32; nominal 10 ms min/median/max `3071 / 3072 / 3076 us`,
cycles `49149 / 49153 / 49217`; 20 ms `6127 / 6127 / 6131 us`; 70 ms
`25504 / 25508 / 25510 us`; 100 ms `39436 / 39440 / 39442 us`; and bounded
1000 ms / 100 substeps `478438 / 478446 / 478453 us`. The same baseline image
recorded FLASH `55060 B / 512 KB = 10.50%` and RAM
`5440 B / 128 KB = 4.15%`. The 168 MHz values must be measured directly with
the unchanged DWT path and reported with SystemCoreClock, cycles, microseconds,
FLASH/RAM, and same-configuration deltas.

## Hardware-safe bring-up order

After an ARM build and Program Verify are available, keep servo actuator power
OFF and complete, in order: debugger clock/readback; HAL millisecond timing;
TIM3/TIM4 counter frequency and PWM period; USART register/actual baud;
DAP/direct UART; APC unchanged-radio link; JY901S USART3; and ROVMAKER USART6.
Stop on any unproven or incorrect PWM timing. Only after those checks pass may
the user proceed to Servo power ON, Neutral, individual low-amplitude commands,
SimpleGait, CPG, and the long-run oscillator-period measurement. The long-run
test must report the actual steady-state period/frequency relative to the
nominal period parameter `T=2.0 s`; it must not label the result exactly 0.5 Hz
without evidence.
