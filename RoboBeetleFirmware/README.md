# RoboBeetleFirmware

RoboBeetleFirmware is the current STM32F407VET6 Phase 1 firmware for Laptop/Qt ↔ USART1 ↔ STM32 Protocol V2 bring-up and the five-servo semantic descriptor path. This README records the merged hardware-verified modularization baseline and the PR #8 five-servo implementation; the new five-servo layout remains pending target hardware regression.

## Status labels

- **[Implemented]** Confirmed in current source or active `.ioc`.
- **[Implemented / Software Verified]** Confirmed in current source and host-side software checks; this label does not claim target hardware execution.
- **[Hardware Verified]** Reported in the current development record; source alone cannot prove physical execution.
- **[Provisional]** Bring-up values or incomplete calibration.
- **[Planned]** Recommended future work, not current behavior.
- **[Historical Reference]** Old F407ZE, STM32, Simulink, CPG, paper, slide, or resource-tree material that is not the current firmware.

## Current PR #8 five-servo bring-up

The current implementation freezes five semantic IDs and the supported mask at `0x001F` (bits 0–4). Firmware and Qt maintain independent descriptor tables; host tests and pure-C tests assert the same IDs, masks, capabilities, and calibration envelopes so descriptor drift is detected without crossing the C/C++ boundary.

| ID / mask | Semantic actuator | Hardware / timer channel | Capability and command envelope |
|---:|---|---|---|
| `0` / `0x0001` | `FrontRight` / 前足右 | SAVOX SW-0250MG+, TIM3_CH1 / PA6 | PWM 1050–1950 μs; angle −45…+45°; electrical 1000/1500/2000 μs |
| `1` / `0x0002` | `FrontLeft` / 前足左 | SAVOX SW-0250MG+, TIM3_CH2 / PA7 | PWM 1050–1950 μs; angle −45…+45°; electrical 1000/1500/2000 μs |
| `2` / `0x0004` | `FrontAxis` / 升潜前足轴 | HDKJ S3150D, TIM3_CH3 / PB0 | PWM-only 1450–1550 μs; angle disabled; 1500 μs is a provisional startup/center candidate |
| `3` / `0x0008` | `RearRight` / 后足右 | GDW IPX896HV, TIM4_CH1 / PD12 | PWM 1020–2020 μs; angle −45…+45°; electrical 520/1520/2520 μs |
| `4` / `0x0010` | `RearLeft` / 后足左 | GDW IPX896HV, TIM4_CH2 / PD13 | PWM 1020–2020 μs; angle −45…+45°; electrical 520/1520/2520 μs |

TIM3 and TIM4 run at approximately 333 Hz with a 1 μs tick (PSC=15, ARR=3002). `servo_descriptor` is pure C and HAL-independent: it stores abstract timer/channel selectors, never `TIM_CHANNEL_x` constants. `servo_driver_stm32` is the only layer that maps those selectors to `TIM_HandleTypeDef *` and HAL channel values.

`FrontAxis` calibration is **[Calibration Pending]**. Its 1450–1550 μs clamp is a bring-up safety envelope, not a calibrated neutral and not a Hardware Verified claim. The first target check must be performed with the mechanism unloaded and horn/linkage detached: `1500 → 1450 → 1500 → 1550 → 1500`. Set Angle is intentionally rejected for this actuator.

Enable accepts a multi-bit mask only with all-or-nothing semantics: if any requested channel fails to start, already-started channels from that call are stopped and the pre-call enabled mask is restored. Disable and Disable All retain fail-closed/best-effort stop behavior.

This is a hardware-layout compatibility break. Historical v0.4 `Servo1`/PA6 bring-up referred to `RearLeft`; PR #8 formally assigns PA6/ID 0 to `FrontRight` and assigns `RearLeft` to PD13/TIM4_CH2. Do not mix a pre-PR8 Console/Firmware binary with the PR8 five-servo wiring. The Qt `Servo1` name is only a deprecated source-compatibility alias for `FrontRight`; new firmware code uses semantic names.

PR #8 software descriptor, service, and dispatch regressions are the implementation gate. Five-servo target build/download and physical motion verification are still **[Pending Hardware Verification]**; the earlier Servo1-only hardware milestones remain historical evidence for the old layout.

## Active target and CubeMX configuration

The active configuration file is `RoboBeetleFirmware/RoboBeetleFirmware.ioc`. A separately referenced `D:\RoboBeetle\RoboBeetle.ioc` was not present during this audit.

| Item | Current configuration |
|---|---|
| MCU | **STM32F407VET6**, LQFP100; build define `STM32F407xx` |
| CubeMX / HAL | STM32CubeMX 6.18.1; STM32Cube FW_F4 1.28.3 |
| Debug | Serial Wire on PA13/SWDIO and PA14/SWCLK |
| System clock | HSI 16 MHz, PLL off; SYSCLK/HCLK/PCLK1/PCLK2 all 16 MHz |
| USART1 | PA9 TX, PA10 RX; 9600 baud, 8 data bits, no parity, 1 stop bit, no flow control, oversampling 16 |
| USART1 NVIC | Enabled; preemption/subpriority 0/0 |
| TIM3 PWM | TIM3_CH1/PA6, TIM3_CH2/PA7, TIM3_CH3/PB0, AF2, PWM mode 1, active high |
| TIM3 timing | PSC=15, ARR=3002, CCR1/CCR2/CCR3 initial=1500 |
| TIM4 PWM | TIM4_CH1 on PD12 and TIM4_CH2 on PD13, AF2, PWM mode 1, active high |
| TIM4 timing | PSC=15, ARR=3002, CCR1/CCR2 initial=1520 |
| PWM result | 16 MHz / (15+1) = 1 MHz counter (1 μs/tick); 1 MHz / (3002+1) ≈ **333.0 Hz** |
| Debug GPIO | PB2 push-pull output labelled `DBG_LED`; initialized low, no runtime toggling in current code |

Do not apply the old STM32F407ZE configuration to this project. The current MCU and pinout are established by the active `.ioc`, generated HAL code, linker/startup files, and CMake define.

## Current runtime architecture

```text
USART1 RX byte
  ↓ USART1_IRQHandler → HAL_UART_IRQHandler
HAL_UART_RxCpltCallback
  ↓ push byte; immediately re-arm 1-byte HAL_UART_Receive_IT
single-producer/single-consumer ring buffer
  ↓ main-loop uart_transport_stm32_pop
Protocol V2 delimiter accumulator
  ↓ COBS + header/length + CRC decode
protocol_dispatcher_handle
  ├─ Heartbeat / host liveness outcome
  ├─ semantic five-servo enable / disable
  ├─ Set Servo PWM
  ├─ Set Servo Angle calibration mapping
  ├─ Neutral semantic command
  └─ one-entry duplicate suppression / result replay
        ↓
main-loop ACK generation / UART TX
        ↓
servo_service → servo_driver_stm32
        ├─ TIM3_CH1 / PA6 → FrontRight
        ├─ TIM3_CH2 / PA7 → FrontLeft
        ├─ TIM3_CH3 / PB0 → FrontAxis (PWM-only)
        ├─ TIM4_CH1 / PD12 → RearRight
        └─ TIM4_CH2 / PD13 → RearLeft
```

The interrupt handler delegates to the HAL. The HAL completion callback performs only a ring-buffer push and re-arms the next one-byte interrupt receive. Protocol parsing, command dispatch, ACK encoding, blocking UART transmit, and PWM control occur in the main-loop context, not in the UART ISR.

The current communication split is:

- **[Implemented]** `Core/Communication/ring_buffer.c/.h` owns the fixed 128-byte single-producer/single-consumer ring. It reserves one slot (127-byte effective capacity) and silently rejects a push while full, preserving the original behavior.
- **[Implemented]** `Core/Communication/uart_transport_stm32.c/.h` owns the one-byte RX staging byte, USART1 receive interrupt arm/re-arm, ring interaction, main-loop byte retrieval, and the blocking `HAL_UART_Transmit(..., 100U)` wrapper.
- **[Hardware Verified]** `Core/App/app_main.c/.h` owns the application orchestration: Protocol V2 wire accumulation and decode integration, ACK/result transmission, diagnostics, module instances, initialization order, RX draining, and post-drain Safety timeout action. It calls existing Protocol, UART, Safety, Servo, and dispatcher modules without implementing their policies or touching TIM registers directly.
- **[Implemented]** `Core/Servo/servo_descriptor.c/.h` owns the pure-C semantic ID, capability, calibration-envelope, and abstract timer/channel table.
- **[Implemented]** `Core/Servo/servo_calibration.c/.h` owns per-descriptor integer angle-to-pulse mapping.
- **[Implemented]** `Core/Servo/servo_service.c/.h` owns supported-mask validation, enabled-state policy, command range checks, Neutral semantics, multi-bit Enable rollback, and driver-independent Servo results.
- **[Implemented]** `Core/Servo/servo_driver_stm32.c/.h` owns the HAL/TIM3/TIM4 channel adapter. It maps abstract descriptor selectors to timer handles and HAL channels and has no Protocol or heartbeat knowledge.
- **[Hardware Verified]** `Core/Safety/safety_supervisor.c/.h` owns host liveness, the last valid Heartbeat timestamp, strict timeout evaluation, and one-shot timeout transition reporting. It has no HAL, Protocol, UART, or Servo dependency.
- **[Hardware Verified]** `Core/Communication/protocol_dispatcher.c/.h` owns decoded command payload validation, HostAlive gating, Servo service invocation/result mapping, Heartbeat semantics, and the one-entry successful-command cache. It has no HAL, UART, TIM3, or Console dependency.
- **[Hardware Verified]** `main.c` keeps the CubeMX entry/configuration, `app_main_init`/`app_main_process` calls, and a small UART callback transport delegate. Protocol, Safety, Servo, ACK, diagnostics, and RX-drain orchestration live in `Core/App/app_main.c`.

The Servo service/calibration/driver extraction is now **[Hardware Verified]** in PR #3. STM32CubeIDE target build passed, ST-LINK download completed with “Download verified successfully”, and the physical Servo regression passed for Connect + Heartbeat, Enable + ACK, Neutral, 0°, ±10°, ±45°, ±90°, Disable, Disable All, Disconnect, reconnect without automatic Enable, and manual Enable + ACK recovery. This confirms the behavior-preserving extraction on the target hardware.

The App/Main extraction in PR #6 is now **[Hardware Verified]**. STM32CubeIDE build PASS, ST-LINK download PASS, and full physical regression PASS covered cold boot/reset without automatic Enable, Connect + Heartbeat, Enable + ACK, Neutral, Set Angle 0/+10/-10/+45/-45/+90/-90 degrees, Set PWM 1520 us, Disable/Disable All, re-enable, Disconnect, the >500 ms safe-disable transition, reconnect without automatic Enable, manual Enable + ACK recovery, and repeated disconnect/reconnect.

## UART receive and transmit audit

- **[Implemented]** `ring_buffer` storage is 128 bytes with `uint16_t` head/tail indices.
- The empty/full distinction reserves one slot, so usable capacity is **127 bytes**.
- On full buffer, `ring_buffer_push()` silently drops the new byte. There is no overflow flag/counter and no host-visible error.
- Head and tail remain volatile, with the same one-byte ISR producer / main-loop consumer model as the original implementation.
- `uart_transport_stm32` calls `HAL_UART_Receive_IT()` at startup and re-arms it in the callback; no blocking receive remains.
- Return values from initial and callback receive-arm calls are ignored.
- The transport calls `HAL_UART_Transmit(..., 100U)` only while main-loop dispatch sends an ACK. It is blocking but not ISR-blocking. At 9600 8-N-1 a short ACK frame normally takes milliseconds, yet a stalled transmit can block the loop for up to 100 ms.

## Protocol V2

`rb_protocol_v2.c/.h` is already a HAL-independent C codec containing little-endian helpers, CRC-16/CCITT-FALSE, COBS encode/decode, logical header validation, and wire encoding. It should remain a pure protocol library.

Current constants:

- Magic `52 42`, version `02`
- Header 8 bytes, CRC 2 bytes, payload ≤64 bytes
- `WireFrame = COBS(LogicalFrame) + 00`
- Message IDs: Heartbeat `01`, ACK `02`, Error `03`, Servo Enable `10`, Servo Disable `11`, Set Servo PWM `12`, Set Servo Angle `13`, Neutral `14`

See `../RoboBeetleConsole/docs/protocol.md` for the detailed Console ↔ Firmware matrix. Important current behavior is:

- Heartbeat, ACK, semantic five-servo Enable/Disable, and per-descriptor Set PWM agree with the Console.
- Error is declared but never sent by Firmware.
- Neutral validates liveness/mask/enabled state, writes each descriptor's neutral pulse, and leaves the selected channels enabled.
- Set Angle is available only for the four calibrated SAVOX/GDW angle-capable descriptors; FrontAxis is PWM-only. Each angle is range-checked and mapped with `int32_t` intermediates.
- ACK result values are frozen as `OK=0`, `InvalidPayload=1`, `HostNotAlive=2`, `UnsupportedServo=3`, `ServoNotEnabled=4`, `OutOfRange=5`, and `HardwareFailure=6`.
- Supported mask is exactly `0x001F`. Zero mask is invalid; any unknown bit fails with `UnsupportedServo`. Multi-bit Enable is all-or-nothing with rollback on a channel-start failure.
- The most recent successful non-Heartbeat request is cached by sequence and type. Its retry replays the ACK without executing the Servo action again. Heartbeats refresh liveness but do not evict this cache.

## Heartbeat and safety state

### [Implemented]

- The Safety Supervisor marks the host alive only after a valid four-byte Heartbeat.
- The supervisor stores `last_heartbeat_rx_ms`; `app_main` injects local `HAL_GetTick()`, not the host timestamp.
- Servo Enable, Set PWM, and Set Angle reject commands while the supervisor reports the host not alive.
- If more than 500 ms elapse after the last Heartbeat, the supervisor reports one timeout transition; the main loop stops all enabled PWM channels and clears the enabled mask.
- Reconnection/recovery requires a new valid Heartbeat followed by a new Servo Enable.
- Boot leaves PWM stopped. TIM3/TIM4 are configured with descriptor-specific initial CCR values, but `HAL_TIM_PWM_Start()` is called only on accepted Enable.
- Each accepted Enable writes its descriptor's neutral pulse before starting the requested channels.
- Watchdog timeout also invalidates the duplicate cache, preventing an old successful action ACK from bypassing re-enable after recovery.

### Limitations

- Watchdog processing shares the main loop with blocking ACK transmission and all frame dispatch.
- There is no independent hardware watchdog, fault state, persisted reset reason, leak/battery/current input, or emergency-stop message in this Phase 1 source.
- Duplicate suppression intentionally retains one successful non-Heartbeat request rather than a multi-entry replay window. A later distinct successful actuator request replaces it.
- Disconnect safety relies on the host's best-effort Disable All plus the 500 ms Firmware heartbeat timeout.

## Legacy Servo1-only state (Historical Reference)

The following Servo1-only notes describe the pre-PR #8 PA6/RearLeft layout and are retained for traceability. They do not describe the current five-servo ID map above; use the PR #8 section and active `.ioc` for current behavior.

### [Implemented]

- Logical identity: Servo1 ID 0, mask bit `0x0001`.
- Hardware mapping: TIM3_CH1 / PA6.
- PWM carrier: approximately 333 Hz.
- Set PWM requires a live host, exactly one payload item, Servo ID 0, enabled state, and 520–2520 μs inclusive.
- Set Angle requires a live host, exactly one payload item, Servo ID 0, enabled state, and −9000…+9000 cdeg inclusive; out-of-range values are rejected rather than clamped.
- Neutral requires a live host, valid Servo1 mask, and enabled state; it writes 1520 μs without stopping PWM or clearing enable state.
- Disable and heartbeat timeout call `HAL_TIM_PWM_Stop()`.

### [Provisional]

- Minimum ≈520 μs ≈−90°.
- Neutral 1520 μs =0°.
- Maximum ≈2520 μs ≈+90°.
- Servo model in the development record: GDW IPX896HV; merchant specification is PWM, 333 Hz, approximately 500/1500/2500 μs, 180°±5°, 4.8–8.4 V.
- The horn was mechanically re-centered near 1520 μs.

### [Hardware Verified]

The current development record states that Qt → Set Servo PWM → TIM3 CCR was observed, TIM3_CH1 drove Servo1, the GDW IPX896HV produced real motion, and the horn was mechanically centered. These are development-record claims, not conclusions produced by static code inspection.

The UART transport/ring-buffer extraction in this refactor is now **[Hardware Verified]**. STM32CubeIDE target build passed, ST-LINK download completed with “Download verified successfully”, and physical UART/Servo regression passed for Connect + Heartbeat, Enable + ACK, Neutral, +10°, 0°, −10°, Disable, Disconnect, reconnect without automatic Enable, and manual Enable + ACK recovery. The existing Servo1 and Set Angle hardware verification remains valid.

## PR #8 software verification status

- Pure-C descriptor, calibration, Servo service, ring-buffer, safety, and Protocol Dispatcher regressions pass with warnings treated as errors.
- The five-servo descriptor/service/dispatcher changes are software-verified; STM32CubeIDE target build, ST-LINK download, and physical five-servo regression remain **[Pending Hardware Verification]** for the PR8 wiring.
- The target commands remain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

## Historical target build snapshots (Historical Reference)

The following records refer to pre-PR8 or earlier modularization snapshots and are retained for traceability; they are not evidence that the current five-servo image has passed target build or hardware regression. The undefined angle calibration constants from the previous audit were replaced by one consistent Servo1 calibration set. A new-directory Debug configure/build using STM32CubeIDE's bundled CMake 4.3.1, Ninja 1.13.2, and GNU Tools for STM32 14.3.1 succeeded for that earlier snapshot.

Normal project commands remain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The project uses C11, Ninja, `arm-none-eabi-gcc`, and the generated STM32CubeMX CMake target. The generated CubeMX CMake remains untouched; the user-maintained top-level CMake lists the App, Communication, Servo, and Safety modules and their include directories. The pure-C checks `tests/protocol_golden_vectors.c`, `tests/ring_buffer_tests.c`, `tests/servo_calibration_tests.c`, `tests/servo_service_tests.c`, `tests/safety_supervisor_tests.c`, and `tests/protocol_dispatcher_tests.c` are compiled manually with `-Wall -Wextra -Werror`; there is still no integrated host-side wire-parser/HAL test target.

## App/Main maintainability audit

The App/Main boundary is now **[Hardware Verified]**. `main.c` is limited to the CubeMX-generated startup and peripheral initialization, `app_main_init`/`app_main_process` delegation, the thin UART completion callback, and the existing error/assert handlers. `Core/App/app_main.c` owns the cooperative application loop and glue code while delegating Protocol, UART, Safety, Servo, and dispatcher policy to their existing modules. STM32CubeIDE build, ST-LINK download, and full physical regression all passed for PR #6.

The split follows dependency direction instead of mechanically creating folders:

```text
Core/
├─ App/
│  ├─ app_main.c
│  └─ app_main.h
├─ Communication/
│  ├─ uart_transport_stm32.c/h
│  ├─ ring_buffer.c/h
│  └─ protocol_dispatcher.c/h
├─ Protocol/
│  └─ rb_protocol_v2.c/h
├─ Servo/
│  ├─ servo_driver_stm32.c/h
│  ├─ servo_service.c/h
│  └─ servo_calibration.c/h
├─ Safety/
│  └─ safety_supervisor.c/h
└─ Src/main.c
```

Recommended boundaries:

- Keep `main.c` limited to `HAL_Init`, clock/MX initialization, `app_main_init`, `app_main_process`, and CubeMX-safe callbacks that immediately delegate.
- `uart_transport_stm32` remains HAL-aware and owns `UART_HandleTypeDef`, RX re-arm, and eventually a nonblocking TX queue.
- `ring_buffer` is pure C and reusable; its silent full-buffer drop is preserved until a separately reviewed overflow policy is introduced.
- `rb_protocol_v2` remains pure C and HAL-independent.
- `protocol_dispatcher` parses command payloads, enforces the existing validation order, and calls service interfaces; it must not write TIM registers directly.
- `servo_driver_stm32` is the HAL-aware TIM3/TIM4 channel adapter: start, stop, and write descriptor pulse ticks.
- `servo_service` is pure C policy: supported IDs/masks, enabled state, bounds, and command semantics. It calls the driver through a narrow interface.
- `servo_calibration` is pure C data/mapping: per-servo min/neutral/max angle and pulse, direction, and later nonlinear points if required.
- `safety_supervisor` is pure C state/timing policy: host liveness, deadline evaluation, and one-shot timeout transition reporting. It has no HAL or Servo dependency; `app_main` requests safe actions through `servo_service`.
- `app_main` wires modules together and owns cooperative scheduling.

### Recommended order

1. Preserve this clean Protocol V2 / Servo1 baseline and perform a controlled hardware check of Neutral and Set Angle before expanding capability.
2. **[Implemented in this refactor]** Extract ring buffer and UART transport, preserving exact ISR behavior.
3. **[Hardware Verified in this refactor]** Extract the Servo HAL driver and pure Servo service/calibration, preserving the existing policy and calibration values.
4. **[Hardware Verified in this refactor]** Extract the Safety Supervisor with externally injected time and preserve the strict 500 ms timeout policy.
5. **[Hardware Verified in this refactor]** Extract the Protocol Dispatcher while preserving command validation order, Heartbeat semantics, and duplicate suppression.
6. **[Hardware Verified]** Reduce `main.c` to initialization and `app_main_init`/`app_main_process` delegation. STM32CubeIDE build, ST-LINK download, and full physical regression passed for PR #6.

Do not split the already isolated Protocol V2 codec further during Phase 1, add an RTOS, or introduce generic device frameworks before a second actuator/transport actually requires them.

## Remaining technical debt

- P1: the HAL-coupled App/Main layer has no dedicated host integration test; target build and physical regression are the verification gate.
- P1: the one-entry duplicate cache is deliberately minimal and is not a general replay window.
- P1: ring overflow and RX re-arm failures are silent.
- P1: blocking UART ACK transmit shares the watchdog/parser loop.
- P2: Error `0x03` remains reserved; command failures currently use the frozen ACK result enum.
- P2: diagnostics are volatile counters only and are not exposed as telemetry.
- P2: debug LED is configured but unused.
- P2: no integrated Firmware-native codec/dispatcher/safety/calibration CTest target; current pure-C checks are manually compiled.

PR #3 adds only the three `Core/Servo` modules, their user-maintained top-level CMake source/include entries, pure-C Servo regression tests, the `main.c` service/driver delegation points, and this documentation. `.ioc`, generated CubeMX CMake, pins, clocks, USART settings, base frame format, CRC, COBS rules, message IDs, heartbeat policy, and Servo calibration values were not changed. The prior UART/ring-buffer extraction remains hardware verified, and the Servo service/calibration/driver extraction is now hardware verified: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Servo regression PASS. No further code refactoring is included in this change.

PR #4 adds only the pure-C `Core/Safety` supervisor, its user-maintained top-level CMake source/include entries, the liveness regression test, the `main.c` time-injection/timeout delegation points, and this documentation. The strict `(now - last_heartbeat_rx_ms) > 500U` behavior, Protocol V2 parsing, ACKs, Servo behavior, UART, `.ioc`, and generated CubeMX CMake remain unchanged. The Safety Supervisor is now **[Hardware Verified]**: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Safety regression PASS, covering Heartbeat loss, the strict timeout safe-disable transition, reconnect without automatic Enable, manual Enable + ACK recovery, and repeated disconnect/reconnect behavior.

PR #5 adds only the pure-C `Core/Communication/protocol_dispatcher` module, its host regression test, the user-maintained CMake source entry, the `main.c` decoded-frame/outcome integration, and this documentation. Protocol V2 wire framing, ACK encoding/transmit, UART transport, Safety policy, Servo behavior/calibration, `.ioc`, and generated CubeMX CMake remain unchanged. The Protocol Dispatcher extraction is now **[Hardware Verified]**: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Protocol regression PASS covering Connect + Heartbeat, Enable + ACK, Neutral, Set Angle 0/+10/-10 degrees, Set PWM 1520 us near Neutral, Disable/re-enable, Disconnect, the >500 ms safe-disable transition, reconnect without automatic Enable, and manual Enable + ACK recovery.

PR #6 adds only the `Core/App/app_main.c/.h` orchestration layer, its user-maintained CMake source/include entries, the `main.c` delegation points, and this documentation. Protocol V2 wire framing, ACK encoding/transmit, duplicate cache, UART transport, Safety policy, Servo behavior/calibration, `.ioc`, generated CubeMX CMake, and peripheral initialization values remain unchanged. App/Main extraction is now **[Hardware Verified]**: STM32CubeIDE Build PASS, ST-LINK Download PASS, and Full physical regression PASS.
