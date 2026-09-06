# RoboBeetleFirmware

RoboBeetleFirmware is the current STM32F407VET6 Phase 1 firmware for Laptop/Qt ↔ USART1 ↔ STM32 Protocol V2 bring-up and Servo1 PWM/angle control. This README records the repaired and clean-built baseline verified on 2026-09-05.

## Status labels

- **[Implemented]** Confirmed in current source or active `.ioc`.
- **[Hardware Verified]** Reported in the current development record; source alone cannot prove physical execution.
- **[Provisional]** Bring-up values or incomplete calibration.
- **[Planned]** Recommended future work, not current behavior.
- **[Historical Reference]** Old F407ZE, STM32, Simulink, CPG, paper, slide, or resource-tree material that is not the current firmware.

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
| TIM3 PWM | TIM3_CH1 on PA6, AF2, PWM mode 1, active high |
| TIM3 timing | PSC=15, ARR=3002, CCR1 initial=1520 |
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
protocol_handle_frame
  ├─ Heartbeat / host liveness
  ├─ ACK generation
  ├─ Servo1 enable / disable
  ├─ Set Servo PWM
  ├─ Set Servo Angle calibration mapping
  ├─ Neutral semantic command
  └─ one-entry duplicate suppression / ACK replay
        ↓
servo state + TIM3_CH1 CCR/start/stop
        ↓
PA6 → Servo1
```

The interrupt handler delegates to the HAL. The HAL completion callback performs only a ring-buffer push and re-arms the next one-byte interrupt receive. Protocol parsing, command dispatch, ACK encoding, blocking UART transmit, and PWM control occur in the main-loop context, not in the UART ISR.

The current communication split is:

- **[Implemented]** `Core/Communication/ring_buffer.c/.h` owns the fixed 128-byte single-producer/single-consumer ring. It reserves one slot (127-byte effective capacity) and silently rejects a push while full, preserving the original behavior.
- **[Implemented]** `Core/Communication/uart_transport_stm32.c/.h` owns the one-byte RX staging byte, USART1 receive interrupt arm/re-arm, ring interaction, main-loop byte retrieval, and the blocking `HAL_UART_Transmit(..., 100U)` wrapper.
- **[Implemented]** `main.c` keeps the CubeMX entry/configuration, protocol wire accumulator/dispatch, safety policy, duplicate cache, and Servo/TIM3 behavior. Its UART callback is a small transport delegate.

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

- Heartbeat, ACK, Servo1 Enable/Disable, and Servo1 Set PWM agree with the Console.
- Error is declared but never sent by Firmware.
- Neutral validates liveness/mask/enabled state, writes the calibrated 1520 μs neutral, and leaves Servo1 enabled.
- Set Angle validates −9000…+9000 cdeg and maps −9000/0/+9000 to 520/1520/2520 μs with `int32_t` intermediates.
- ACK result values are frozen as `OK=0`, `InvalidPayload=1`, `HostNotAlive=2`, `UnsupportedServo=3`, `ServoNotEnabled=4`, `OutOfRange=5`, and `HardwareFailure=6`.
- Only mask `0x0001` is supported. Zero mask is invalid; any Servo2/unknown bit fails atomically with `UnsupportedServo`.
- The most recent successful non-Heartbeat request is cached by sequence and type. Its retry replays the ACK without executing the Servo action again. Heartbeats refresh liveness but do not evict this cache.

## Heartbeat and safety state

### [Implemented]

- `host_alive` becomes 1 only after a valid four-byte Heartbeat.
- `last_heartbeat_rx_ms` uses local `HAL_GetTick()`, not the host timestamp.
- Servo Enable, Set PWM, and Set Angle reject commands while `host_alive==0`.
- If more than 500 ms elapse after the last Heartbeat, the main loop clears `host_alive`, stops Servo1 PWM if enabled, and clears the enabled mask.
- Reconnection/recovery requires a new valid Heartbeat followed by a new Servo Enable.
- Boot leaves PWM stopped. TIM3 is configured with CCR1=1520, but `HAL_TIM_PWM_Start()` is called only on accepted Servo1 Enable.
- Each accepted enable writes CCR1=1520 before starting PWM.
- Watchdog timeout also invalidates the duplicate cache, preventing an old successful action ACK from bypassing re-enable after recovery.

### Limitations

- Watchdog processing shares the main loop with blocking ACK transmission and all frame dispatch.
- There is no independent hardware watchdog, fault state, persisted reset reason, leak/battery/current input, or emergency-stop message in this Phase 1 source.
- Duplicate suppression intentionally retains one successful non-Heartbeat request rather than a multi-entry replay window. A later distinct successful actuator request replaces it.
- Disconnect safety relies on the host's best-effort Disable All plus the 500 ms Firmware heartbeat timeout.

## Servo1 state

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

The UART transport/ring-buffer extraction in this refactor has not been flashed or regression-tested on the physical board yet. The existing Servo1 and Set Angle hardware verification remains valid for the unchanged functional behavior; hardware regression of this refactor is **pending**.

## Current build and test status

The undefined angle calibration constants from the previous audit were replaced by one consistent Servo1 calibration set. A new-directory Debug configure/build using STM32CubeIDE's bundled CMake 4.3.1, Ninja 1.13.2, and GNU Tools for STM32 14.3.1 succeeds and links `RoboBeetleFirmware.elf` with no compiler warnings.

Normal project commands remain:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The project uses C11, Ninja, `arm-none-eabi-gcc`, and the generated STM32CubeMX CMake target. The generated CubeMX CMake remains untouched; the user-maintained top-level CMake now lists the communication modules and include directory. There is still no host-side dispatcher/HAL unit-test target; `tests/protocol_golden_vectors.c` and `tests/ring_buffer_tests.c` are standalone pure-C checks compiled manually with `-Wall -Wextra -Werror`.

## `main.c` maintainability audit

Current `main.c` contains both CubeMX-generated entry/configuration code and almost all application policy. Its responsibilities include:

1. HAL startup and SystemClock configuration.
2. GPIO, USART1, and TIM3 initialization.
3. CubeMX USART1 initialization and a tiny UART completion-callback delegate.
4. Protocol wire accumulation, delimiter resynchronization, and decode dispatch.
5. ACK payload creation, Firmware TX sequence, wire encoding, and calls to the blocking transport transmit wrapper.
6. Heartbeat diagnostics and host-liveness state.
7. Command validation and ACK result selection.
8. Servo ID/mask and enabled-state policy.
9. PWM start, stop, CCR update, and safe-start position.
10. Provisional angle bounds and angle-to-pulse mapping.
11. 500 ms communication-loss safety action.
12. Bring-up counters and debug LED initialization.
13. Main-loop scheduling.

This is already too many responsibilities for a generated entry file. The codec and UART transport are separated, but dispatch, actuator policy, calibration, and safety remain coupled.

## [Planned] Remaining refactor

The split should follow dependency direction instead of mechanically creating folders:

```text
Core/
├─ App/
│  ├─ app_main.c/h
│  └─ app_config.h
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

- Keep `main.c` limited to `HAL_Init`, clock/MX initialization, `App_Init`, `App_Process`, and CubeMX-safe callbacks that immediately delegate.
- `uart_transport_stm32` remains HAL-aware and owns `UART_HandleTypeDef`, RX re-arm, and eventually a nonblocking TX queue.
- `ring_buffer` is pure C and reusable; its silent full-buffer drop is preserved until a separately reviewed overflow policy is introduced.
- `rb_protocol_v2` remains pure C and HAL-independent.
- `protocol_dispatcher` parses command payloads and calls service interfaces; it must not write TIM registers directly.
- `servo_driver_stm32` is the HAL-aware TIM3/channel adapter: start, stop, and write pulse ticks.
- `servo_service` is pure C policy: supported IDs/masks, enabled state, bounds, and command semantics. It calls the driver through a narrow interface.
- `servo_calibration` is pure C data/mapping: per-servo min/neutral/max angle and pulse, direction, and later nonlinear points if required.
- `safety_supervisor` is pure C state/timing policy: host liveness, deadline evaluation, transition to disabled/fault states, and reason reporting. It requests safe actions through `servo_service` rather than calling HAL.
- `app_main` wires modules together and owns cooperative scheduling.

### Recommended order

1. Preserve this clean Protocol V2 / Servo1 baseline and perform a controlled hardware check of Neutral and Set Angle before expanding capability.
2. **[Implemented in this refactor]** Extract ring buffer and UART transport, preserving exact ISR behavior.
3. Extract the Servo HAL driver and pure Servo service/calibration.
4. Extract the Safety supervisor and make time injectable for host tests.
5. Reduce `main.c` to initialization and `App_Init`/`App_Process` delegation.

Do not split the already isolated Protocol V2 codec further during Phase 1, add an RTOS, or introduce generic device frameworks before a second actuator/transport actually requires them.

## Remaining technical debt

- P1: dispatcher/Servo/safety behavior remains HAL-coupled in `main.c`, so duplicate and CCR side effects lack a host unit test.
- P1: the one-entry duplicate cache is deliberately minimal and is not a general replay window.
- P1: ring overflow and RX re-arm failures are silent.
- P1: blocking UART ACK transmit shares the watchdog/parser loop.
- P2: Error `0x03` remains reserved; command failures currently use the frozen ACK result enum.
- P2: diagnostics are volatile counters only and are not exposed as telemetry.
- P2: debug LED is configured but unused.
- P2: no Firmware-native codec/dispatcher/safety/calibration test target.

The UART transport refactor adds only the two `Core/Communication` modules, their user-maintained top-level CMake source/include entries, the pure-C ring-buffer regression test, the `main.c` delegation points, and this documentation. `.ioc`, generated CubeMX CMake, pins, clocks, USART settings, base frame format, CRC, COBS rules, protocol dispatch, safety policy, and Servo calibration were not changed. No firmware was flashed and no Servo was moved during this refactor; hardware regression remains pending.
