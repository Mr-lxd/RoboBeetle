# RoboBeetleFirmware

RoboBeetleFirmware is the current STM32F407VET6 Phase 1 firmware for Laptop/Qt ↔ USART1 ↔ STM32 Protocol V2 bring-up, the five-servo semantic descriptor path, and the PR #9 leak-status telemetry path. This README records the merged hardware-verified modularization baseline, the PR #8 Servo/Depth bench findings, and the PR #9 leak-status hardware acceptance. Depth endpoint calibration remains pending; the LeakStatus path is now hardware verified.

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
| `0` / `0x0001` | `FrontRight` | SAVOX SW-0250MG+, TIM3_CH1 / PA6 | PWM 1050–1950 μs; angle −45…+45°; electrical 1000/1500/2000 μs |
| `1` / `0x0002` | `FrontLeft` | SAVOX SW-0250MG+, TIM3_CH2 / PA7 | PWM 1050–1950 μs; angle −45…+45°; electrical 1000/1500/2000 μs |
| `2` / `0x0004` | `Depth` (`FrontAxis` internal ID) | HDKJ S3150D, TIM3_CH3 / PB0 | Electrical metadata 500/1500/2500 μs; PWM command envelope 500–2500 μs; angle disabled; 1500 μs is provisional |
| `3` / `0x0008` | `RearRight` | GDW IPX896HV, TIM4_CH1 / PD12 | PWM 1020–2020 μs; angle −45…+45°; electrical 520/1520/2520 μs |
| `4` / `0x0010` | `RearLeft` | GDW IPX896HV, TIM4_CH2 / PD13 | PWM 1020–2020 μs; angle −45…+45°; electrical 520/1520/2520 μs |

TIM3 and TIM4 run at approximately 333 Hz with a 1 μs tick (PSC=15, ARR=3002). `servo_descriptor` is pure C and HAL-independent: it stores abstract timer/channel selectors, never `TIM_CHANNEL_x` constants. `servo_driver_stm32` is the only layer that maps those selectors to `TIM_HandleTypeDef *` and HAL channel values.

`FrontAxis` calibration is **[Calibration Pending]**. Seller-provided electrical metadata is 500–2500 μs pulse width, 1500 μs center candidate, 4.8–7.4 V operating voltage, 0–270° controllable travel, and 4 μs dead band. These values describe electrical/absolute capability metadata; they do not by themselves establish a final mechanically safe command range. The matching Firmware/Qt command envelope is the provisional 500–2500 μs PWM-only endpoint-exploration window. This `500–2500 μs` window is **[Pending Hardware Verification]**, not the final mechanically safe endpoint range. On the current bench, approximately 1100–2500 μs produced approximately the intended 180-degree mechanism travel, while commands below approximately 1100 μs tended to cause ACK timeouts; do not continue probing below approximately 1100 μs for now. Final mechanical safe min/max, practical center, and angle mapping remain deferred until the complete mechanical assembly is installed. The supplied 1480/1500/1520 direction check is Hardware Verified, but 1500 μs remains only a provisional bring-up center candidate, not the final mechanical center. Set Angle is intentionally rejected for this actuator. User-facing labels use exact ASCII names (`FrontRight`, `FrontLeft`, `Depth`, `RearRight`, `RearLeft`) while `FrontAxis` remains an internal identifier. Waterproof capability is **[Unverified]**: the seller parameter page says “not waterproof,” while the product photo/shell says “Water proof Robot Servo.” Do not claim or test direct immersion without reliable IP/sealing evidence.

Current supplied bring-up evidence keeps `FrontRight`, `FrontLeft`, and `RearLeft` **[Hardware Verified]**; the `RearRight` STM32/A12 PWM path is **[Hardware Verified]**, while the original RearRight actuator/lead is a hardware fault scheduled for replacement. Depth direction at 1480/1500/1520 μs is **[Hardware Verified]** only; the observed approximately 1100–2500 μs bench travel and the expanded 500–2500 μs endpoint-exploration window remain **[Pending Hardware Verification]**. Commands below approximately 1100 μs are not to be probed further in the current setup.

Enable accepts a multi-bit mask only with all-or-nothing semantics. Requested channels already present in the pre-call enabled mask are idempotent and receive no pulse write, start, or stop. If any newly requested channel fails to start, only channels newly started by that call are stopped and the pre-call enabled state—including the physical pulse of an already-running channel—is preserved. Disable and Disable All retain fail-closed/best-effort stop behavior.

This is a hardware-layout compatibility break. Historical v0.4 `Servo1`/PA6 bring-up referred to `RearLeft`; PR #8 formally assigns PA6/ID 0 to `FrontRight` and assigns `RearLeft` to PD13/TIM4_CH2. Do not mix a pre-PR8 Console/Firmware binary with the PR8 five-servo wiring. The Qt `Servo1` name is only a deprecated source-compatibility alias for `FrontRight`; new firmware code uses semantic names.

PR #8 software descriptor, service, and dispatch regressions are the implementation gate. Five-servo target build/download and physical motion verification are still **[Pending Hardware Verification]**; the earlier Servo1-only hardware milestones remain historical evidence for the old layout.

## PR #9 leak-status telemetry: leak D0 on PA11 — [Hardware Verified]

The first sensor phase adds a polled digital leak input and a monitoring-only
Protocol V2 telemetry path. The module is
powered from 3.3 V with common GND; its digital output `D0` is wired to
STM32 `PA11`, while analog `A0` is intentionally unused. The current path is:

```text
leak D0 → PA11 GPIO input → leak_sensor_stm32 raw reader
  → leak_sensor pure-C mapper → app_main internal state
  → LeakStatus `0x20` telemetry after an accepted Heartbeat ACK
  → Qt leak indicator
```

`PA11` is configured in the existing `MX_GPIO_Init()` path as
`GPIO_MODE_INPUT` with `GPIO_NOPULL`; there is no EXTI, debounce, alarm, or
Safety action. The module's output-stage type is not fully established by the
available documentation, so `GPIO_NOPULL` is a bring-up assumption rather than
a verified electrical conclusion. A source-level resource scan checked PA11
against the active `.ioc`, `main.c`, HAL MSP, USART1, TIM3/TIM4, SWD, and
existing GPIO assignments and found it free at the software resource level. The
initial polarity is PA11 HIGH → `LEAK_SENSOR_STATE_DRY` and PA11 LOW →
`LEAK_SENSOR_STATE_WET`.

Firmware emits `LeakStatus` (`0x20`) as one unacknowledged byte (`0=UNKNOWN`,
`1=DRY`, `2=WET`) only after an accepted Heartbeat has had its normal ACK
fully transmitted. The telemetry has an independent sequence space and is
published on the first valid sample, on state change, or at most once per
500 ms refresh interval; it has no independent transmit timer and never
changes Servo or Safety state. The Console returns to `Unknown` on disconnect,
APC liveness loss, invalid payload, or a stale telemetry interval of 1500 ms
(three 500 ms Firmware refresh opportunities; provisional).

The pure-C mapper/policy, Protocol V2 vectors, and Qt/controller regressions are
**[Host Test: PASS]**. The current STM32CubeIDE Debug ARM configure/build is
**[ARM Build: PASS]**, and the rebuilt current ELF was programmed and verified
with **[Program Verify: PASS]**. Physical acceptance is now recorded as:

| Acceptance item | Status |
|---|---|
| PA11 leak detection path | **[Hardware Verified]** |
| Protocol V2 `LeakStatus (0x20)` real-link exchange | **[Hardware Verified]** |
| Qt Leak indicator | **[Hardware Verified]** |
| End-to-end Leak monitoring | **[Hardware Verified]** |
| Leak Safety response | **[Pending / Not Implemented]** |

The earlier persistent `Leak: Unknown` observation was caused by programming/build
artifact provenance. After rebuilding the correct current ELF and programming and
verifying that image, the complete PA11 → Firmware → Protocol V2 → Qt path worked
as intended. No numeric voltage or response-time values are asserted here because
they were not part of the recorded acceptance result.

At the PR #9 closeout, the remaining sensor sequence was JY901S IMU →
depth/sensor board; the current JY901S phase is documented below. LeakStatus
remains monitoring-only and is not connected to Servo or Safety actions.

## JY901S listen-only bring-up — Review / Hardware Verification Ready

This phase adds only the receive and parser path:

```text
JY901S TX
  → PB11 / USART3_RX
  → one-byte interrupt receive
  → independent 256-byte ring buffer (255-byte effective capacity)
  → pure-C 11-byte parser
  → internal Acc / Gyro / Angle state
```

JY901S RX is connected to PB10 / USART3_TX in the hardware design. PB10 is
configured for the UART path, but this application does not send baud-rate,
output-rate, output-mask, save, restart, calibration, or any other JY901S
command. STM32 USART3 is locally configured for 9600 baud, 8-N-1, TX/RX, and
no hardware flow control so the firmware can listen to the sensor's current
persistent/default configuration. The bench expectation is approximately
10 Hz with Acc, Gyro, Angle, and possibly Mag frames.

The parser decodes WIT standard types `0x51` Acc, `0x52` Gyro, and `0x53`
Angle using signed little-endian values and the documented engineering-unit
scalings. A checksum-valid `0x54` Mag frame is known-but-ignored: it increments
`mag_frame_count`, preserves all supported state, and does not increment
`unsupported_frame_count`. A checksum-valid genuinely unknown type increments
`unsupported_frame_count` and also preserves supported state. The frame
contract follows the [official WIT standard protocol](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol);
the product page lists 9600 baud and the default output expectations.

USART3 diagnostics are available through the bring-up accessors and include
`rx_byte_count`, successful ring pushes, foreground pops, ring overflow/drop,
re-arm failures, aggregate and per-flag UART errors, parser header starts,
valid checksum frames, checksum errors, Mag-known-ignore frames, unsupported
types, per-domain frame counts, and the last valid-frame tick. If a callback
re-arm fails or USART3 reports ORE/FE/NE/PE/DMA/other errors, the ISR only
records the event and marks `needs_rearm`; `app_main_process` makes at most one
non-blocking re-arm attempt per poll. No parser work or retry loop runs in the
ISR.

Validation is intentionally separated:

| Gate | Status |
|---|---|
| Host Test | **PASS**: parser, transport mock, ring-buffer, and all current Firmware regressions |
| ARM Build | **PASS**: STM32 target build; 0 errors, 0 warnings; RAM 2680 B / 128 KB, FLASH 23260 B / 512 KB |
| Program Verify | **PASS**: DAP/OpenOCD programming flow completed and reported `Verified OK` |
| Hardware Verified | **Pending**: physical JY901S bench verification |
| USART3 physical RX | **Pending** |
| JY901S valid real frames | **Pending** |
| Acc/Gyro/Angle real data | **Pending** |
| Pending | Physical USART3 reception, wiring/electrical checks, and current sensor-configuration diagnosis |

On 2026-09-10, the hardware-verification checkout built
`RoboBeetleFirmware.elf` at
`D:\RoboBeetle\RoboBeetleFirmware\build\Debug\RoboBeetleFirmware.elf`
with 0 errors and 0 warnings. The recorded artifact had a last-write time of
2026-09-10 17:07:11. The DAP/OpenOCD flow used SWD 100 kHz, SYSRESETREQ,
halt, program, verify, and reset-run, and reported `Programming Finished`,
`Verify Started`, and `Verified OK`. These facts establish the ARM Build and
Program Verify gates only; they do not establish physical JY901S reception or
valid Acc/Gyro/Angle data.

If no legal frame appears on the bench, first inspect RX bytes, ring-buffer
activity, `0x55` headers, valid/checksum/error counters, and state updates in
that order. Do not add automatic JY901S configuration in response; a separate
configuration/init phase requires evidence that the physical UART is working
but the sensor's current persistent settings are not the expected ones.

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
  ├─ LeakStatus telemetry (after accepted Heartbeat ACK)
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
- **[Implemented]** `Core/Sensors/leak_sensor.c/.h` owns the HAL-independent UNKNOWN/DRY/WET mapping; `leak_sensor_stm32.c/.h` only reads the configured PA11 GPIO.
- **[Implemented]** `Core/Sensors/leak_telemetry_policy.c/.h` limits LeakStatus publication to first sample/state changes/500 ms refreshes. `Core/App/app_main.c` sends one-byte `0x20` telemetry only after a successful Heartbeat ACK; it does not connect leak state to Safety or Servo behavior.
- **[Hardware Verified]** `Core/Safety/safety_supervisor.c/.h` owns host liveness, the last valid Heartbeat timestamp, strict timeout evaluation, and one-shot timeout transition reporting. It has no HAL, Protocol, UART, or Servo dependency.
- **[Hardware Verified]** `Core/Communication/protocol_dispatcher.c/.h` owns decoded command payload validation, HostAlive gating, Servo service invocation/result mapping, Heartbeat semantics, and the one-entry successful-command cache. It has no HAL, UART, TIM3, or Console dependency.
- **[Hardware Verified]** `main.c` keeps the CubeMX entry/configuration, `app_main_init`/`app_main_process` calls, and a small UART callback transport delegate. Protocol, Safety, Servo, ACK, diagnostics, and RX-drain orchestration live in `Core/App/app_main.c`.

The Servo service/calibration/driver extraction is now **[Hardware Verified]** in PR #3. STM32CubeIDE target build passed, ST-LINK download completed with “Download verified successfully”, and the physical Servo regression passed for Connect + Heartbeat, Enable + ACK, Neutral, 0°, ±10°, ±45°, ±90°, Disable, Disable All, Disconnect, reconnect without automatic Enable, and manual Enable + ACK recovery. This confirms the behavior-preserving extraction on the target hardware.

The App/Main extraction in PR #6 is now **[Hardware Verified]**. STM32CubeIDE build PASS, ST-LINK download PASS, and full physical regression PASS covered cold boot/reset without automatic Enable, Connect + Heartbeat, Enable + ACK, Neutral, Set Angle 0/+10/-10/+45/-45/+90/-90 degrees, Set PWM 1520 us, Disable/Disable All, re-enable, Disconnect, the >500 ms safe-disable transition, reconnect without automatic Enable, manual Enable + ACK recovery, and repeated disconnect/reconnect.

## UART receive and transmit audit

- **[Implemented]** The USART1/APC220 `ring_buffer` storage is 128 bytes with `uint16_t` head/tail indices. JY901S uses a separate 256-byte storage instance.
- The empty/full distinction reserves one slot, so usable capacity is **127 bytes**.
- On the USART1/APC220 full buffer, `ring_buffer_push()` silently drops the new byte. That legacy transport has no overflow flag/counter and no host-visible error; the JY901S transport has independent overflow diagnostics.
- Head and tail remain volatile, with the same one-byte ISR producer / main-loop consumer model as the original implementation.
- The USART1/APC220 `uart_transport_stm32` calls `HAL_UART_Receive_IT()` at startup and re-arms it in the callback; no blocking receive remains.
- Return values from the legacy USART1/APC220 initial and callback receive-arm calls are ignored. USART3/JY901S records re-arm failures and retries once from foreground maintenance.
- The transport calls `HAL_UART_Transmit(..., 100U)` only while main-loop dispatch sends an ACK. It is blocking but not ISR-blocking. At 9600 8-N-1 a short ACK frame normally takes milliseconds, yet a stalled transmit can block the loop for up to 100 ms.

## Protocol V2

`rb_protocol_v2.c/.h` is already a HAL-independent C codec containing little-endian helpers, CRC-16/CCITT-FALSE, COBS encode/decode, logical header validation, and wire encoding. It should remain a pure protocol library.

Current constants:

- Magic `52 42`, version `02`
- Header 8 bytes, CRC 2 bytes, payload ≤64 bytes
- `WireFrame = COBS(LogicalFrame) + 00`
- Message IDs: Heartbeat `01`, ACK `02`, Error `03`, Servo Enable `10`, Servo Disable `11`, Set Servo PWM `12`, Set Servo Angle `13`, Neutral `14`, LeakStatus `20`

See `../RoboBeetleConsole/docs/protocol.md` for the detailed Console ↔ Firmware matrix. Important current behavior is:

- Heartbeat, ACK, semantic five-servo Enable/Disable, and per-descriptor Set PWM agree with the Console.
- Error is declared but never sent by Firmware.
- Neutral validates liveness/mask/enabled state, writes each descriptor's neutral pulse, and leaves the selected channels enabled.
- Set Angle is available only for the four calibrated SAVOX/GDW angle-capable descriptors; FrontAxis is PWM-only. Each angle is range-checked and mapped with `int32_t` intermediates.
- LeakStatus `0x20` is a one-byte, unacknowledged monitoring frame (`UNKNOWN=0`, `DRY=1`, `WET=2`). Firmware sends it only after an accepted Heartbeat and completed ACK transmission, on first sample/state change or a 500 ms refresh; it has an independent telemetry sequence and does not trigger Safety or Servo actions.
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
- There is no independent hardware watchdog, fault state, persisted reset reason, leak safety response, battery/current input, or emergency-stop message in this Phase 1 source. Leak D0 is polled into an internal state and exposed through monitoring-only LeakStatus telemetry; it does not change Servo behavior.
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

The project uses C11, Ninja, `arm-none-eabi-gcc`, and the generated STM32CubeMX CMake target. The generated CubeMX CMake remains untouched; the user-maintained top-level CMake lists the App, Communication, Servo, Safety, and Sensors modules and their include directories. The Firmware host gate currently compiles and runs all 11 executable test sources: `tests/protocol_golden_vectors.c`, `tests/ring_buffer_tests.c`, `tests/servo_descriptor_tests.c`, `tests/servo_calibration_tests.c`, `tests/servo_service_tests.c`, `tests/safety_supervisor_tests.c`, `tests/protocol_dispatcher_tests.c`, `tests/leak_sensor_tests.c`, `tests/servo_driver_stm32_tests.c`, `tests/jy901s_parser_tests.c`, and `tests/jy901s_transport_stm32_tests.c`. The leak sensor source is linked with `leak_telemetry_policy.c` because that existing test covers both behaviors. The HAL-adapter mapping test uses host stubs for PWM start/stop and explicitly verifies that HAL `TIM_CHANNEL_1 == 0` remains valid. `tests/app_main_jy901s_api_tests.c` is a separate compile-contract check; all are compiled manually with `-Wall -Wextra -Werror` plus the documented host HAL pointer-cast suppression where needed. These host checks complement, but do not replace, the real ARM target build.

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
├─ Sensors/
│  ├─ leak_sensor.c/h
│  └─ leak_sensor_stm32.c/h
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
- P1: JY901S overflow, RX re-arm, and UART error diagnostics are volatile/debug-visible only; they are not exposed through Protocol V2 telemetry.
- P1: blocking UART ACK transmit shares the watchdog/parser loop.
- P2: Error `0x03` remains reserved; command failures currently use the frozen ACK result enum.
- P2: diagnostics are volatile counters only and are not exposed as telemetry.
- P2: debug LED is configured but unused.
- P2: no integrated Firmware-native codec/dispatcher/safety/calibration CTest target; current pure-C checks are manually compiled.

PR #3 adds only the three `Core/Servo` modules, their user-maintained top-level CMake source/include entries, pure-C Servo regression tests, the `main.c` service/driver delegation points, and this documentation. `.ioc`, generated CubeMX CMake, pins, clocks, USART settings, base frame format, CRC, COBS rules, message IDs, heartbeat policy, and Servo calibration values were not changed. The prior UART/ring-buffer extraction remains hardware verified, and the Servo service/calibration/driver extraction is now hardware verified: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Servo regression PASS. No further code refactoring is included in this change.

PR #4 adds only the pure-C `Core/Safety` supervisor, its user-maintained top-level CMake source/include entries, the liveness regression test, the `main.c` time-injection/timeout delegation points, and this documentation. The strict `(now - last_heartbeat_rx_ms) > 500U` behavior, Protocol V2 parsing, ACKs, Servo behavior, UART, `.ioc`, and generated CubeMX CMake remain unchanged. The Safety Supervisor is now **[Hardware Verified]**: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Safety regression PASS, covering Heartbeat loss, the strict timeout safe-disable transition, reconnect without automatic Enable, manual Enable + ACK recovery, and repeated disconnect/reconnect behavior.

PR #5 adds only the pure-C `Core/Communication/protocol_dispatcher` module, its host regression test, the user-maintained CMake source entry, the `main.c` decoded-frame/outcome integration, and this documentation. Protocol V2 wire framing, ACK encoding/transmit, UART transport, Safety policy, Servo behavior/calibration, `.ioc`, and generated CubeMX CMake remain unchanged. The Protocol Dispatcher extraction is now **[Hardware Verified]**: STM32CubeIDE build PASS, ST-LINK download PASS, and physical Protocol regression PASS covering Connect + Heartbeat, Enable + ACK, Neutral, Set Angle 0/+10/-10 degrees, Set PWM 1520 us near Neutral, Disable/re-enable, Disconnect, the >500 ms safe-disable transition, reconnect without automatic Enable, and manual Enable + ACK recovery.

PR #6 adds only the `Core/App/app_main.c/.h` orchestration layer, its user-maintained CMake source/include entries, the `main.c` delegation points, and this documentation. Protocol V2 wire framing, ACK encoding/transmit, duplicate cache, UART transport, Safety policy, Servo behavior/calibration, `.ioc`, generated CubeMX CMake, and peripheral initialization values remain unchanged. App/Main extraction is now **[Hardware Verified]**: STM32CubeIDE Build PASS, ST-LINK Download PASS, and Full physical regression PASS.

PR #9 adds the pure-C leak-state telemetry policy, the Protocol V2 `LeakStatus (0x20)` frame, and a monitoring-only Qt indicator. Firmware samples PA11 and publishes after an accepted Heartbeat ACK on first/change/500 ms refresh opportunities; Console stale/disconnect/liveness loss returns the indicator to Unknown. Host Test, ARM Build, Program Verify, PA11 detection, real-link telemetry, Qt indication, and end-to-end Leak monitoring are now **[Hardware Verified]**. The acceptance also confirmed that rebuilding/programming/verifying the correct current ELF resolved the earlier persistent Unknown display; no numeric voltage or response-time claim is made. No leak state is connected to Servo or Safety actions.
