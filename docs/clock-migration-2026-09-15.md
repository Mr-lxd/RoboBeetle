# Clock migration evidence — 2026-09-15

## Status and dependency

The approved clock migration is implemented on branch `codex/clock-migration`,
based on PR #15 reviewed head `b7e2787fe9eb4fdd46de0fd7380a23bbc961cc52`.
PR #15 remains the first integration dependency; this branch is intentionally
separate and is not merged here. The source/configuration and host contract
checks are software-verified. The user has now supplied target/tool evidence
for 168 MHz CubeMX regeneration, ARM Build, Program Verify,
SystemCoreClock/RCC/TIM readback, the three UART/runtime links, individual
servo safety behavior, and the direct DWT A/B benchmark. HAL tick target
verification, physical PWM waveform measurement, and gait/water verification
remain separately classified below.

The supplied individual servo checks include no unexpected movement after the
168 MHz migration. Gait behavior and water performance are not inferred from
those checks; no Water Verified claim is made by this document.

## 2026-09-15 independent software re-verification

Against commit `24bd2a409c8298a8950a1ad4b3ae5174a0ebcd75` on the clean
`codex/clock-migration` worktree, the clock contract test and the full Firmware
host gate were rerun. The result was **Software Verified / PASS**:
`clock_config_contract_tests.ps1` passed, `run_host_tests.ps1` passed with 29
executables plus 9 app/backend/benchmark compile-contract objects, and
`git diff --check` passed. The checked configuration was the HSI 16 MHz to
PLL 168 MHz clock tree with TIM3/TIM4 PSC `83`, ARR `3002`, and USART1/3/6
baud values `9600/9600/115200`.

Evidence boundary: this is source/configuration and host-only evidence from a
clean branch worktree. It does not itself establish CubeMX Generate Code parity,
an ARM image, Program Verify, target register readback, HAL tick measurement,
PWM waveform timing, UART/APC physical links, servo/gait behavior, DWT A/B, or
water performance. The separately supplied target evidence below is authoritative
for the items it covers; HAL tick, physical waveform, gait, and Water remain
**Pending** where explicitly stated.

## 2026-09-15 user-supplied 168 MHz target and actuator evidence

The user supplied the following results for the 168 MHz image associated with
clock branch commit `24bd2a409c8298a8950a1ad4b3ae5174a0ebcd75`. The report is
recorded as supplied. It includes ARM Build 168 MHz = PASS and Program Verify =
PASS; raw debugger dumps, BRR values, detailed build/size logs, and waveform
captures were not included.

| Check | Measurement / configuration | Result | Evidence class |
| --- | --- | --- | --- |
| CubeMX 6.18.1 regeneration | User-supplied Generate Code parity result | PASS | CubeMX / Tool Evidence |
| ARM Build 168 MHz | User-supplied target build result | PASS | Target Build Evidence |
| Program Verify 168 MHz | User-supplied programming and verification result | PASS | Target Program Evidence |
| CLOCK READBACK | `SystemCoreClock = 168000000`; RCC PLL/APB runtime readback passed | PASS | Target Measured |
| RCC clock tree | PLL/APB runtime register readback PASS; raw register values not supplied | PASS | Target Measured |
| TIM3/TIM4 registers | `PSC=83`, `ARR=3002` runtime readback | PASS | Target Measured |
| PWM TIMEBASE CONTRACT | SystemCoreClock/RCC/TIM readback plus `84 MHz / (83 + 1) = 1 MHz = 1 us/count` | PASS | Target Measured / calculated contract |
| USART1 / APC | USART1 `9600`; APC link operational | PASS | Target Measured + Hardware Verified |
| USART3 / JY901S | USART3 `9600`; JY901S state `Receiving` | PASS | Target Measured + Hardware Verified |
| USART6 / ROVMAKER | USART6 `115200`; Depth state `Receiving` | PASS | Target Measured + Hardware Verified |
| INDIVIDUAL SERVO SAFE BRING-UP | Neutral, small-angle, Release, and Disable behavior | PASS | Hardware Verified |
| Unexpected movement | No unexpected servo movement after migration | PASS | Hardware Verified observation |
| PWM PHYSICAL WAVEFORM SCOPE MEASUREMENT | No oscilloscope/logic-analyzer pulse-width capture supplied | Pending | Physical Scope Evidence |
| HAL TICK PHYSICAL/TARGET VERIFICATION | One-second target interval measurement not supplied | Pending | Target Timing Evidence |
| GAIT AFTER CLOCK MIGRATION | No new Forward/Turn/Ascend/Descend/SimpleGait/CPG exercise supplied | Pending | Separate Gait Evidence |
| Water | No propulsion/hydrodynamic result supplied | Pending | Water Evidence |
| DWT 16/168 A/B | Direct 168 MHz target benchmark and frozen 16 MHz comparison recorded below | PASS | Target Performance Evidence |

These results close the listed 168 MHz ARM Build, Program Verify, clock
readback, PWM timebase contract, UART/APC, JY901S/ROVMAKER UART,
individual-servo safe bring-up, CubeMX regeneration, and DWT A/B statuses. The
ARM Build, Program Verify, CubeMX, and DWT claims are recorded as user-supplied
evidence; detailed build/size logs, the Generate Code transcript/diff, and the
Program Verify transcript were not included. Physical PWM waveform, HAL tick,
gait, and Water remain separate evidence items.

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

CubeMX 6.18.1 was not available in this agent workspace, so Generate Code
parity was not independently reproduced here. The user-supplied evidence
records **CubeMX regeneration verification = PASS** for this 168 MHz image.
The raw Generate Code transcript/diff was not included; this status is recorded
as supplied and is not inferred from the host contract test.

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
source contract, approximately 333.0003 Hz. The user-supplied target readback
proves the configured counter contract. No physical period/pulse capture was
supplied, so the physical PWM waveform remains Pending.

UART application settings remain unchanged: USART1 9600 8-N-1, USART3 9600
8-N-1, and USART6 115200 8-N-1. HAL initialization still runs after
`SystemClock_Config()`, so BRR calculation uses the new PCLK2 for USART1/6 and
PCLK1 for USART3. APC Series remains 9600, APC RF TRx remains 19200, and the RF
frequency is unchanged. The supplied target run reports the runtime settings
and APC/JY901S/ROVMAKER links as passing. Raw BRR values and baud-error
calculations were not supplied and remain a separate follow-up if required.

`HAL_GetTick()` remains the HAL millisecond timebase. Motion10ms,
STOP750ms, heartbeat/liveness, depth stale, telemetry, and all safety
deadlines remain unchanged. `app_main_process()` continues to evaluate
SafetySupervisor/liveness before MotionManager elapsed-time catch-up. A stale
foreground gap must abort before any catch-up or Servo write and must not
auto-resume from heartbeat recovery alone. A measured one-second HAL tick
interval was not included in the supplied result and remains Pending.

The production CPG remains the source-compatible `double` implementation. No
CPG equation, theta-dot state, gait semantics, semantic adapter,
LogicalJointTargets, calibration, Protocol V2, Qt, or sensor protocol was
changed in this branch.

## Verification record

| Check | Result |
| --- | --- |
| Focused clock contract before implementation | RED as expected: legacy `RCC_PLL_NONE` was missing the required PLL contract |
| Focused clock contract after implementation | PASS `clock_config_contract_tests`: main.c + `.ioc` source inputs + `RCC.IPParameters` + derived values + TIM/UART contracts |
| CubeMX 6.18.1 regeneration parity | **CubeMX / Tool Evidence / PASS**: user supplied; local CubeMX executable unavailable for independent reproduction |
| Firmware host gate | PASS: 29 executables + 9 app/backend/benchmark compile-contract objects, including the clock contract |
| Public-header self-sufficiency | PASS: 34 headers with host C11 `-Wall -Wextra -Werror`; CMSIS host pointer-width warnings explicitly suppressed |
| ARM Build 168 MHz | **Target Build Evidence / PASS**: user supplied; detailed compiler/size log not included |
| ARM toolchain / size record | **Pending / NOT_FOUND** in this environment; no independent size claim made |
| SystemCoreClock 168 MHz debugger readback | **Target Measured / PASS**: user supplied `168000000` |
| RCC PLL/APB runtime register readback | **Target Measured / PASS**: user supplied runtime readback; raw values not included |
| PWM TIMEBASE CONTRACT | **Target Measured / PASS**: SystemCoreClock/RCC/TIM readback, `PSC=83`, `ARR=3002`, and `1 us/count` result |
| PWM PHYSICAL WAVEFORM SCOPE MEASUREMENT | **Pending**: no oscilloscope/logic-analyzer capture supplied |
| HAL TICK PHYSICAL/TARGET VERIFICATION | **Pending**: no one-second target interval measurement supplied |
| USART runtime settings and host/APC/JY901S/ROVMAKER links | **Target Measured + Hardware Verified / PASS** for supplied runtime/link results; raw BRR not included |
| Individual Servo Neutral/small-angle/Release/Disable | **Hardware Verified / PASS**: no unexpected movement reported |
| GAIT AFTER CLOCK MIGRATION | **Pending**: no new Forward/Turn/Ascend/Descend/SimpleGait/CPG exercise supplied |
| 16 MHz versus 168 MHz DWT A/B | **Target Performance Evidence / PASS**: user supplied direct benchmark and comparison; no ideal scaling estimate used |
| Program Verify 168 MHz | **Target Program Evidence / PASS**: user supplied; transcript not included |
| Water | **Pending Water Verification** |

The isolated 16 MHz baseline is preserved exactly as the comparison source for
the completed A/B record below: repetitions 32; nominal 10 ms min/median/max
`3071 / 3072 / 3076 us`,
cycles `49149 / 49153 / 49217`; 20 ms `6127 / 6127 / 6131 us`; 70 ms
`25504 / 25508 / 25510 us`; 100 ms `39436 / 39440 / 39442 us`; and bounded
1000 ms / 100 substeps `478438 / 478446 / 478453 us`. The same baseline image
recorded FLASH `55060 B / 512 KB = 10.50%` and RAM
`5440 B / 128 KB = 4.15%`. The 168 MHz values below were measured directly with
the unchanged DWT path and are reported with SystemCoreClock, cycles,
microseconds, FLASH/RAM, and same-configuration deltas.

## Hardware-safe bring-up order

The supplied run closes the 168 MHz CubeMX regeneration, ARM Build, Program
Verify, debugger clock/RCC readback, PWM timebase contract, UART/runtime-link,
individual-servo, and DWT A/B checks. HAL millisecond timing and the physical
PWM waveform still require their own evidence. The previously required safe
order remains:
ARM build and Program Verify, servo power OFF for clock/
timer/UART checks, then only after those checks pass the individual servo and
gait exercises. The supplied result does not include SimpleGait or CPG runtime
exercise. Water propulsion and hydrodynamic performance remain Pending Water
Verification.

## 2026-09-15 user-supplied 168 MHz DWT A/B benchmark

The user supplied a direct STM32F407 target benchmark from the temporary
`ROBOBEETLE_CPG_TARGET_BENCHMARK=ON` image. The target reported
`SystemCoreClock = 168000000 Hz` and `repetitions = 32`. Values are recorded
exactly; no ideal 10.5x scaling is substituted.

| Case | Cycles min / median / max | Microseconds min / median / max |
| --- | ---: | ---: |
| CPG nominal 10 ms | `51371 / 51371 / 51455` | `305 / 305 / 306 us` |
| 20 ms catch-up | `102373 / 102447 / 102466` | `609 / 609 / 609 us` |
| 70 ms catch-up | `426627 / 426704 / 426790` | `2539 / 2539 / 2540 us` |
| 100 ms catch-up | `661035 / 661069 / 661175` | `3934 / 3934 / 3935 us` |
| 1000 ms bounded / 100 substeps | `8080787 / 8081297 / 8081955` | `48099 / 48102 / 48106 us` |

The benchmark image usage was supplied as FLASH `55084 B / 512 KB = 10.51%`
and RAM `5440 B / 128 KB = 4.15%`. The supplied `arm-none-eabi-size` output
was `text=54988`, `data=92`, `bss=5344`, `dec=60424`.

### Frozen 16 MHz versus measured 168 MHz

The comparison uses the frozen 16 MHz median values recorded above. It is a
direct measured comparison, not an ideal clock-ratio estimate.

| Case | 16 MHz median | 168 MHz median | Measured speedup |
| --- | ---: | ---: | ---: |
| Nominal 10 ms | `3072 us` | `305 us` | approximately `10.07x` |
| 20 ms catch-up | `6127 us` | `609 us` | approximately `10.06x` |
| 70 ms catch-up | `25508 us` | `2539 us` | approximately `10.05x` |
| 100 ms catch-up | `39440 us` | `3934 us` | approximately `10.03x` |
| 1000 ms bounded / 100 substeps | `478446 us` | `48102 us` | approximately `9.95x` |

Nominal isolated compute utilization improved from `30.72%` to `3.05%`
(`305 us < 10000 us`). The measured nominal median cycle count increased
slightly from `49153` at 16 MHz to `51371` at 168 MHz while wall time improved
by approximately 10x. The frozen baseline did not include catch-up cycle
fields, so no catch-up cycle delta is reconstructed.

The isolated 168 MHz CPG compute deadline is **[PASS]**. This substantially
increases foreground compute margin, but it does not prove zero Motion jitter
or solve blocking UART behavior. Servo stutter investigation remains separate.

### Temporary benchmark image provenance

The `ROBOBEETLE_CPG_TARGET_BENCHMARK=ON` image is a temporary measurement image,
not the production image. After the DWT measurement, normal operation must be
rebuilt with `ROBOBEETLE_CPG_TARGET_BENCHMARK=OFF`. This documentation update
does not change production source, CPG mathematics, Servo calibration, or the
normal build configuration; the benchmark image must not be classified as the
production firmware.
