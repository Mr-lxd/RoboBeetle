# RoboBeetleConsole

RoboBeetleConsole is the Phase 1 Qt 6 / C++20 engineering console for the current direct Windows laptop → serial/APC220 → STM32 bring-up path. This document describes the implementation repaired and verified on 2026-09-10, including the PR #11 low-rate JY901S telemetry monitor; historical papers, slides, and legacy code are references only.

## Status labels

- **[Implemented]** Confirmed in the current source tree.
- **[Hardware Verified]** Reported by the current development record and handoff notes; this is stronger than a plan but is not derivable from source alone.
- **[Hardware Verified - Bench]** Hardware evidence collected on the current desktop bench; it does not imply distance, antenna-orientation, poolside, or outdoor RF characterization.
- **[Provisional]** Temporary bring-up values or incomplete interfaces that must not be treated as final calibration.
- **[Planned]** Intended future work that is not implemented.
- **[Historical Reference]** Information from old papers, slides, Simulink, CPG, or STM32 projects; it does not describe the current runtime unless independently reconfirmed.

## Current PR #8 five-servo semantic bring-up

The Qt Console now uses an independent descriptor table for the five semantic IDs. The supported mask is fixed at `0x001F`; a matching pure-C Firmware descriptor table is checked separately by tests so the C/C++ boundary stays explicit.

| ID / mask | Semantic actuator | Hardware | Qt capability |
|---:|---|---|---|
| `0` / `0x0001` | `FrontRight` | SAVOX SW-0250MG+, TIM3_CH1 / PA6 | PWM 1050–1950 μs; angle −45…+45° |
| `1` / `0x0002` | `FrontLeft` | SAVOX SW-0250MG+, TIM3_CH2 / PA7 | PWM 1050–1950 μs; angle −45…+45° |
| `2` / `0x0004` | `Depth` (`FrontAxis` internal ID) | HDKJ S3150D, TIM3_CH3 / PB0 | Electrical 500/1500/2500 μs; PWM command 500–2500 μs; **Calibration Pending**; Set Angle disabled |
| `3` / `0x0008` | `RearRight` | GDW IPX896HV, TIM4_CH1 / PD12 | PWM 1020–2020 μs; angle −45…+45° |
| `4` / `0x0010` | `RearLeft` | GDW IPX896HV, TIM4_CH2 / PD13 | PWM 1020–2020 μs; angle −45…+45° |

`ServoId::Servo1` remains only as a deprecated historical source-compatibility alias for `FrontRight`; there is deliberately no `Servo2` alias. User-facing panel labels are the exact ASCII names `FrontRight`, `FrontLeft`, `Depth`, `RearRight`, and `RearLeft`; the internal semantic identifier remains `FrontAxis`. FrontAxis's seller-provided electrical metadata is 500–2500 μs with a 1500 μs center candidate; these values describe electrical/absolute capability and do not by themselves establish a final mechanical safety range. Qt and Firmware now enforce the same provisional 500–2500 μs PWM-only endpoint-exploration window. This `500–2500 μs` window is **[Pending Hardware Verification]** and is not the final mechanically safe endpoint range. Its button is labelled `Center 1500 us — Provisional`; this reuses the Neutral protocol command without claiming a calibrated or Hardware Verified mechanical center. Only the supplied 1480/1500/1520 direction check is Hardware Verified; the expanded window and endpoints remain pending. The next unloaded exploration uses `1500,1400,1300,1200,1100,1000,900,800,700,600,500`, returns to 1500, then `1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500`, with smaller steps near resistance.

FrontAxis waterproof capability is **[Unverified]**. The seller parameter page says it is not waterproof, while the product photo/shell says “Water proof Robot Servo.” Until reliable IP/sealing evidence exists, the project must not describe the actuator as suitable for direct immersion.

Current supplied bring-up evidence keeps `FrontRight`, `FrontLeft`, and `RearLeft` **[Hardware Verified]**; the `RearRight` STM32/A12 PWM path is **[Hardware Verified]**, while the original RearRight actuator/lead is a hardware fault scheduled for replacement. Depth direction at 1480/1500/1520 μs is **[Hardware Verified]** only; the expanded 500–2500 μs window and endpoint calibration remain **[Pending Hardware Verification]**.

This is a hardware-layout compatibility break: the historical v0.4 Servo1/PA6 bring-up object was `RearLeft`, while PR #8 formally assigns PA6/ID0 to `FrontRight` and `RearLeft` to PD13/TIM4_CH2. Do not mix pre-PR8 Console/Firmware binaries with the PR8 five-servo wiring. PR #8 software verification is complete when the descriptor and controller tests pass; target hardware regression for the new layout remains pending.

## Current PR #9 leak-status telemetry — [Hardware Verified]

The Console accepts the monitoring-only Protocol V2 `LeakStatus` message
(`0x20`) with a one-byte payload: `0=UNKNOWN`, `1=DRY`, and `2=WET`. The
controller keeps this state separate from command/ACK scheduling; the frame is
unacknowledged and its Firmware sequence is independent of command ACK
matching. The global panel displays `Leak: Unknown`, `Leak: Dry`, or
`LEAK DETECTED`.

The Firmware publishes LeakStatus only after an accepted Heartbeat's normal
ACK has finished transmitting, on the first valid sample, a state change, or a
500 ms refresh. The Console treats the state as stale after 1500 ms (three
500 ms refresh opportunities; provisional), and returns to `Unknown` on
disconnect, APC220 liveness loss, invalid payload, or stale telemetry.
This indicator is monitoring-only: it does not disable Servos, alter Safety,
or auto-recover anything. The validation record is:

| Evidence | Status |
|---|---|
| Firmware pure-C / Protocol / Qt host tests | **[Host Test: PASS]** |
| STM32 ARM configure/build | **[ARM Build: PASS]** |
| Correct current ELF programmed and verified | **[Program Verify: PASS]** |
| PA11 leak detection | **[Hardware Verified]** |
| Protocol V2 `LeakStatus` real-link path | **[Hardware Verified]** |
| Qt Leak indicator | **[Hardware Verified]** |
| End-to-end Leak monitoring | **[Hardware Verified]** |

The earlier persistent `Leak: Unknown` result was traced to programming/build
artifact provenance. Rebuilding the correct current ELF and programming and
verifying that image restored the complete path. No numeric voltage or response
time values are asserted here because none were recorded in the acceptance
result. Leak monitoring remains independent of Servo and Safety behavior.

## Current PR #11 JY901S telemetry monitor — Review / Hardware Verification Ready

The Console receives the Firmware `ImuSnapshot` (`0x21`) frame through the
existing Protocol V2 decoder and keeps it separate from ACK matching and the
monitoring-only LeakStatus path. The payload is a fixed 56-byte schema with
explicit little-endian fixed-point Acc (mg), Gyro (0.1 dps), and Angle (0.01
degree) fields plus bring-up diagnostics counters.
Under the nominal accepted Heartbeat cadence, the effective ImuSnapshot refresh
is up to approximately 1 Hz; delayed ACK opportunities or pending LeakStatus
refreshes may reduce it. The Firmware does not use an independent IMU transmit
timer.

The `IMU — JY901S` panel is read-only and shows status, Acc, Gyro, Euler angle,
and diagnostics. It displays `Unknown` before data or after transport/APC
liveness loss, `Receiving` after a valid snapshot, `Stale` after 3500 ms without
one, and `Error` after an invalid snapshot. Stale, invalid, disconnected, and
liveness-lost states clear the values; invalid individual domains remain `--`.
There is no 3D model, plot/history, calibration, control action, raw JY901S
passthrough, or automatic JY901S configuration.

PR #11 verification is intentionally separate from PR #10's real target
programming evidence:

| Evidence | Status |
|---|---|
| Firmware host regressions, telemetry codec/scheduler tests, and Qt tests | **[Host Test: PASS]** |
| PR #11 STM32 target build | **[Pending]** until run on this branch |
| PR #11 program/verify | **[Pending]**; no hardware programming in this phase |
| JY901S physical telemetry / end-to-end IMU data | **[Pending]** |

The previously recorded PR #10 ARM Build and Program Verify PASS applies to
the listen-only bring-up image and does not claim that a physical JY901S is
transmitting or that this new telemetry/UI path has been hardware verified.

## Current scope

### [Implemented]

- Qt Widgets UI with serial-port discovery, editable port selection, configurable baud rate, and a default of 9600 baud.
- Connect, Disconnect, Refresh, per-servo Enable/Disable, Disable All, Neutral, and explicit Apply PWM controls.
- Protocol V2 with COBS framing, `0x00` delimiter, CRC-16/CCITT-FALSE, little-endian fields, and a 64-byte maximum payload.
- Explicit DirectUart and Apc220HalfDuplex link profiles. DirectUart retains the 100 ms heartbeat / 200 ms ACK timeout and multi-pending behavior; the APC220 stop-and-wait scheduler is **[Hardware Verified - Bench]**, while its 250 ms / 250 ms timing values remain **[Provisional]**.
- Up to three retransmissions after the original send, always reusing the original sequence and encoded frame.
- ACK/Error reception, request-sequence matching, TX/RX hex display, packet/CRC/timeout counters, ACK status, and an event log.
- Shared ACK result meanings `0..6`, with named rejection status in the monitor.
- `ITransport` abstraction with real `SerialTransport` and test-only `FakeTransport` implementations.
- Protocol codec/stream tests and controller behavior tests.
- Set Angle UI for the four angle-capable semantic servos with descriptor-specific ranges; the UI converts to centidegrees and angle-to-pulse conversion remains authoritative in Firmware. FrontAxis is explicitly PWM-only while calibration is pending, with a shared 500–2500 μs command envelope.
- Monitoring-only LeakStatus `0x20` indicator with Unknown/Dry/Wet states and stale/disconnect fail-to-Unknown behavior; the end-to-end path is **[Hardware Verified]**.
- Monitoring-only JY901S `ImuSnapshot` `0x21` panel with explicit fixed-point display, diagnostics, Unknown/Receiving/Stale/Error lifecycle, and stale/liveness value invalidation; this PR's physical IMU path remains **[Pending]**.

### [Planned]

- Raspberry Pi onboard service and `TcpTransport`.
- Camera and FOMO/ONNX result visualization.
- Depth, battery, curves, and 3D attitude views. The current PR #11 IMU monitor is intentionally not a 3D/history view.
- ROS 2, automatic control, CPG/PID integration, and telemetry models.
- Emergency Stop. The current button is intentionally disabled because Protocol V2 has no such Phase 1 message.

## Runtime architecture

```text
QApplication
  └─ main.cpp
      ├─ SerialTransport : ITransport
      ├─ RobotController
      │   ├─ PacketCodec / StreamDecoder / CRC16
      │   ├─ profile-aware heartbeat timer
      │   ├─ pending ACK/retry scheduler
      │   ├─ bounded APC220 command queue
      │   ├─ logical enabled-mask state
      │   ├─ LeakStatus state / stale policy
      │   └─ ImuSnapshot state / stale policy
      └─ MainWindow
          ├─ connection controls
          ├─ five descriptor-driven servo panels
          ├─ IMU — JY901S monitor
          └─ protocol monitor / event log
```

| Component | Current responsibility |
|---|---|
| `main.cpp` | Creates the application, `SerialTransport`, `RobotController`, and `MainWindow`; injects serial-port discovery. |
| `MainWindow` | Converts UI actions into controller calls and displays controller signals. It does not access `QSerialPort` or construct packets. |
| `RobotController` | Owns command payload construction, sequence allocation, profile-aware heartbeat/ACK scheduling, APC220 queue state, logical servo enable state, range gates, monitoring-only LeakStatus and ImuSnapshot state/staleness, and monitor data. IMU frames never enter pending ACK state. |
| `ITransport` | Byte-stream open/close/write contract plus received-byte, state, and error signals. |
| `SerialTransport` | Qt SerialPort adapter: port scan, 8-N-1, no flow control, async receive, buffered writes, and close-time flush attempt. |
| `FakeTransport` | Deterministic byte transport used by controller tests. It is not a simulator of STM32 behavior. |
| `PacketCodec` | Logical frame encode/decode, COBS, length/type checks, and CRC verification. |
| `StreamDecoder` | Split/sticky-frame accumulation, `0x00` framing, oversized-frame discard, and delimiter resynchronization. |
| `CRC16` | Pure CRC-16/CCITT-FALSE calculation. |
| `RobotControllerConfig` | Runtime timing and provisional PWM limits. |
| `ProtocolMonitor` | Last TX/RX hex is emitted separately; this structure stores packet, CRC, timeout, and ACK status counters. |

## UI capability audit

| Capability | Status | Actual behavior |
|---|---|---|
| COM scan | [Implemented] | `QSerialPortInfo::availablePorts()` returns sorted port names; Refresh reruns discovery. |
| Baud rate | [Implemented] | Editable 1200–3,000,000; default 9600; connection uses 8 data bits, no parity, 1 stop bit, no flow control. |
| Connect / Disconnect | [Implemented] | Connect opens the selected serial port. Disconnect first sends unacknowledged Disable All, then flushes/closes the port. |
| Heartbeat | [Hardware Verified - Bench] / target [Provisional] | DirectUart sends every 100 ms. Apc220HalfDuplex sends a fresh heartbeat immediately after `Connected`, gates user commands until a matching successful ACK, then targets a 250 ms cadence when the stop-and-wait slot is free; due ticks coalesce while a heartbeat is in flight. |
| ACK / retry / timeout | [Hardware Verified - Bench] / timing [Provisional] | DirectUart tracks multiple requests; Apc220HalfDuplex allows one ACK-requiring request in flight, queues up to `kApc220CommandQueueCapacity` user commands, prioritizes due heartbeat over ordinary command retry, and retries the identical frame at the profile timeout. |
| Servo Enable / Disable | [Implemented] | UI logical enable changes only after a matching successful ACK. |
| Disable All | [Implemented] | Sends the current supported mask `0x001F` for all five semantic channels. |
| Neutral | [Implemented] | Sends `0x14` with the selected semantic servo mask after Enable ACK and with no pending Disable; FrontAxis exposes this as `Center 1500 us — Provisional`, not calibrated Neutral. |
| Apply PWM | [Implemented] | Explicit button; slider movement alone does not transmit. Requires successful Enable ACK, no pending Disable, and descriptor command-range validation. |
| Set Angle | [Implemented] | Angle-capable semantic servos use descriptor-specific input ranges and 0.1° steps; Qt converts to signed cdeg and calls `RobotController::setServoAngle()`. The control requires connection, support, Enable ACK, and no pending Disable request. FrontAxis is disabled. |
| Leak status | [Hardware Verified] | Displays `Leak: Unknown`, `Leak: Dry`, or `LEAK DETECTED` from Protocol V2 `0x20`; disconnect, APC liveness loss, invalid payload, and stale telemetry return it to Unknown. Monitoring-only; no Servo/Safety action. |
| JY901S IMU monitor | [Implemented] / physical data [Pending] | Displays `IMU — JY901S` status, valid fixed-point Acc/Gyro/Angle domains, and diagnostics from Protocol V2 `0x21`; Unknown/invalid/Stale/liveness loss clear values. Read-only; no 3D/history/control/configuration. |
| Protocol monitor | [Implemented] | Displays latest TX/RX chunks, packet counts, CRC errors, timeouts, latest matching-ACK RTT, ACK state, and up to 1000 log blocks. |

## Historical Servo1 hardware acceptance (2026-09-06)

This evidence belongs to the pre-PR #8 Servo1/PA6 layout and remains valid only for that historical wiring.

The merged Servo1 Set Angle path is **[Hardware Verified]** on the current bring-up hardware:

- Actuator: GDW IPX896HV on `TIM3_CH1 / PA6`, approximately 333 Hz.
- Protocol V2 Set Angle `0x13`: 0°, +10°, 0°, −10°, 0°, ±45°, and ±90° all passed the controlled acceptance.
- Observed provisional correspondence: −90° ≈ 520 μs, 0° = 1520 μs, +90° ≈ 2520 μs.
- Safety/UI acceptance passed: Disable blocks Set Angle; Enable before ACK does not restore it; Enable + ACK restores it; Disable All and Disconnect block it; Reconnect does not auto-enable; manual Enable + ACK restores it.

## Legacy Servo1-only model and calibration status (Historical Reference)

The following Servo1/Servo2 notes describe the pre-PR #8 Console and are retained for traceability. They do not override the current five-servo descriptor table above.

- Historical names were `Servo1 = 0` and `Servo2 = 1`; current source uses semantic IDs and retains only `Servo1` as a deprecated alias for `FrontRight`.
- The current supported mask is exactly `0x001F`; the five semantic masks are documented in the PR #8 section above.
- **[Provisional]** Current Console limits are 520–2520 μs with neutral 1520 μs.
- **[Provisional]** Servo1 angle command range is −9000…+9000 cdeg (−90.0…+90.0° in the Qt input). The Console transmits this physical unit without converting it to PWM.
- **[Hardware Verified]** The current Servo1 hardware path is GDW IPX896HV on `TIM3_CH1 / PA6` at approximately 333 Hz; the piecewise mapping is still an approximate bring-up calibration: −9000→520 μs, 0→1520 μs, +9000→2520 μs.
- Protocol V2 Set Angle `0x13`, its Controller path, and the Qt UI are **[Hardware Verified]** for the acceptance values above. There is still no persisted or multi-servo calibration model.
- `MainWindow` now builds five descriptor-driven panels. FrontAxis is supported for bounded PWM bring-up but remains angle-disabled while calibration is pending.

The angle controls are implemented for the four angle-capable semantic servos and are enabled only when the transport is connected, the descriptor is supported, the servo has a successful Enable ACK, and no Disable request is pending. Disable, Disable All, and disconnect immediately disable the angle controls. FrontAxis cannot send angle commands until calibration is completed and verified.

**Neutral — [Hardware Verified]**: the current development record confirms that Neutral returns Servo1 to mechanical zero near 1520 μs. The PWM input currently represents the user's debug input value; it is not guaranteed to mirror the last hardware-confirmed position after Neutral or another command.

**Set Angle real servo motion — [Hardware Verified]**: the controlled Servo1 acceptance passed at 0°, ±10°, ±45°, and ±90°. The mapping remains provisional rather than a final precision calibration.

## Safety behavior and limitations

- Startup never enables a servo.
- PWM and Neutral are rejected locally until Servo Enable has received a matching result-0 ACK.
- Semantic angle entry and Set Angle are enabled only while connected, supported, angle-capable, after the matching Servo Enable result-0 ACK, and while no Disable request is pending; Disable/Disable All/disconnect close that UI gate immediately.
- Losing the transport or receiving an APC220 transport error clears in-flight requests, queued commands, heartbeat intent, decoder state, and the Console's logical enable mask, then stops the scheduler. Reconnect starts from a disabled state with one fresh heartbeat and never auto-enables a servo.
- Disconnect/application close attempts Disable All, but deliberately does not wait for its ACK before closing. A successful local serial write is not proof that STM32 acted on it.
- Unexpected link loss can only log that Disable All could not be delivered. The STM32 watchdog is the actual link-loss safety boundary.
- Retries reuse the same sequence and frame. Firmware caches the most recent successful non-Heartbeat request by sequence and type and replays its ACK without repeating the Servo action; Heartbeats refresh liveness without evicting that action cache.
- Firmware invalidates the duplicate cache on heartbeat watchdog timeout, preserving the requirement for a new explicit Enable after reconnect.
- The Console and Firmware validate the frozen five-bit descriptor mask; unknown IDs/bits are rejected without partial action, and FrontAxis Set Angle is rejected as PWM-only.

## Fit for Laptop ↔ Raspberry Pi ↔ STM32

The byte-oriented `ITransport`, protocol codec, decoder, and controller separation are reusable. A future `TcpTransport` can carry the same Protocol V2 wire frames without moving packet construction into the UI.

The following boundaries need deliberate adjustment before the Pi path is added:

- `TransportConfiguration` currently has serial-specific `baudRate`; future endpoint configuration should be transport-specific or represented as a tagged configuration.
- Port discovery is injected into `RobotController`, and `MainWindow` presents COM/baud controls directly. Network discovery/address/port settings should not be forced into the serial UI model.
- Link ownership, reconnect behavior, command origin, telemetry routing, and ACK timeout budgets must be defined across two hops.
- The Pi should forward or terminate Protocol V2 explicitly; it must not ambiguously rewrite sequence numbers or ACK ownership.
- Safety and servo calibration should remain authoritative on STM32, not on the laptop or Pi.

Recommended future flow:

```text
Laptop Qt Console
  ↕ TCP transport
Raspberry Pi Onboard service
  ↕ UART transport
STM32 Protocol V2 dispatcher
  → safety supervisor
  → servo service/calibration
  → TIM HAL driver
```

## Link profiles and APC220 scheduling

`RobotControllerConfig::bringUpProvisional()` remains the DirectUart baseline used by existing controller tests and direct serial integrations. The application selects `RobotControllerConfig::apc220Provisional()` so the physical laptop → APC220 → STM32 path uses the **[Hardware Verified - Bench]** stop-and-wait scheduler. Its 250 ms heartbeat target and 250 ms ACK timeout remain **[Provisional]** link parameters.

In Apc220HalfDuplex mode, only one ACK-requiring frame is active. Servo user commands that arrive while it is active wait in the bounded `kApc220CommandQueueCapacity` queue. Heartbeat timer ticks set a single due flag; they never accumulate. When a heartbeat is due alongside a command retry, the heartbeat goes first and the command retry retains its original sequence and encoded frame. Any transport reset clears the in-flight request, queue, heartbeat due flag, and logical enable/pending state.

The monitor exposes `ProtocolMonitor::lastAckRttMs`, and the UI displays the latest matching-ACK RTT for diagnosing the high-latency link. APC220 matching heartbeat rejection/mismatch/timeout keeps the liveness gate closed until a later heartbeat succeeds.

The bring-up evidence and link-isolation lessons are recorded in the root [engineering lessons](../docs/engineering-lessons.md).

## APC220 scheduler hardware acceptance (2026-09-08)

**APC220 Half-Duplex Scheduler: [Hardware Verified - Bench]**

The user-accepted desktop-bench regression covered the 440 MHz two-module link and the complete Qt → APC220 → STM32 → ACK → APC220 → Qt path:

- 60 s idle Heartbeat, Servo1 Enable + ACK, Neutral, 0° → +10° → 0° → −10° → 0°, ±45°, ±90°.
- Rapid repeated Set Angle traffic exercising the bounded scheduler queue.
- Disable / Disable All safety priority.
- Robot-side APC220 disconnect, Heartbeat loss/retry, Firmware watchdog safe-disable, and APC220 recovery.
- Recovery never auto-rearmed Servo1; a fresh user Enable plus matching ACK was required.
- CRC errors remained 0 during the normal link run. Normal desktop ACK RTT was approximately 160–170 ms (approximately 160–173 ms across the recorded observations).
- Retry/Timeout events injected deliberately for fault isolation were expected safety behavior and are not normal-link timeout statistics.

The following remain **[Provisional]** pending poolside/lab distance, antenna-orientation, and outdoor RF characterization:

- APC220 heartbeat target: 250 ms.
- ACK timeout: 250 ms.
- Console host-side/local safety admission budget: 490 ms.

This bench result verifies the scheduler behavior on the tested setup; it is not a Windows-plus-RF hard-real-time guarantee.

## Protocol

See [docs/protocol.md](docs/protocol.md) for the byte layout, payloads, Console ↔ Firmware consistency table, reliability behavior, and golden vectors.

## Build on the audited Windows environment

The current Qt installation is:

- Qt 6.11.2 MinGW 64-bit: `D:\Qt\6.11.2\mingw_64`
- MinGW-w64 GCC/G++ 13.1.0: `D:\Qt\Tools\mingw1310_64\bin`
- CMake 3.30.5: `D:\Qt\Tools\CMake_64\bin\cmake.exe`
- Ninja 1.12.1: `D:\Qt\Tools\Ninja\ninja.exe`

Use a process-local PATH so Anaconda Qt 5.9.7 cannot be selected:

```powershell
$env:PATH = "D:\Qt\Tools\mingw1310_64\bin;D:\Qt\Tools\Ninja;D:\Qt\6.11.2\mingw_64\bin;$env:PATH"

& 'D:\Qt\Tools\CMake_64\bin\cmake.exe' `
  -S . `
  -B build\mingw-debug `
  -G Ninja `
  -DBUILD_TESTING=ON `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_PREFIX_PATH='D:\Qt\6.11.2\mingw_64'

& 'D:\Qt\Tools\CMake_64\bin\cmake.exe' --build build\mingw-debug
& 'D:\Qt\Tools\CMake_64\bin\ctest.exe' --test-dir build\mingw-debug --output-on-failure
```

Do not mix the MinGW Qt libraries with MSVC, LLVM-MinGW, the separately installed WinLibs toolchain, or Anaconda Qt.

## Software verification status (2026-09-10 PR #11 follow-up)

- A fresh MinGW/Qt CMake configure and build succeeds without changing the generated project structure; the host CTest set includes protocol, controller, descriptor, IMU lifecycle, and MainWindow IMU panel tests.
- `protocol_tests`: **PASS**, including CRC/COBS regression, result enum values, Neutral, and −9000/0/+9000 cdeg golden vectors.
- `robot_controller_tests`: **PASS**, including semantic five-servo descriptor boundaries, PWM boundaries (including FrontAxis 500/2500 acceptance and 499/2501 rejection), angle-capability/range gates, FrontAxis rejection, pending-Disable PWM/Neutral/Angle barriers across APC Error/timeout, Neutral ACK, ACK match/mismatch, identical-frame retry, APC220 first-heartbeat ACK gate, heartbeat coalescing and retry priority, bounded queue release, heartbeat rejection/timeout liveness, Error type validation, error-only/write-failure reset, DirectUart multi-pending regression, and IMU-frame isolation from ACK/Leak state.
- `imu_monitor_tests` and `main_window_tests`: **PASS**, including fixed 56-byte Protocol V2 golden vectors, schema/flag/range validation, partial validity, Receiving/Stale/Error/Unknown lifecycle, fixed-point display, and clearing stale values.
- Firmware was separately clean-built with the STM32 GCC toolchain. PR #7 user hardware regression passed on the desktop APC220 bench; the timing values remain a Console-side adaptation and the Firmware watchdog remains unchanged.

## Historical Servo1 hardware milestones (pre-PR #8)

The pre-PR #8 development record marks the following as **[Hardware Verified]** for the historical Servo1/PA6 layout: Qt 6 Console startup; SerialTransport on COM10; USART1 bidirectional traffic; interrupt RX plus ring buffer; Protocol V2 COBS/CRC; heartbeat; STM32 ACK reception in Qt; normal TX/RX packets with CRC error count remaining zero during the recorded run; Servo Enable/Disable; heartbeat watchdog; Set Servo PWM updating TIM3 CCR; TIM3 PWM driving Servo1; Neutral near 1520 μs; Protocol V2 Set Angle `0x13` and the Qt Set Angle UI at 0°, ±10°, ±45°, and ±90°; the corresponding Disable/Enable/ACK/Disable All/Disconnect/Reconnect safety-state behavior; and real GDW IPX896HV motion. These records do not verify the PR #8 five-servo rewiring.

These milestones are recorded from the development/handoff record, not inferred from source. The current provisional correspondence is approximately −90°=520 μs, 0°=1520 μs, +90°=2520 μs; it remains an approximate bring-up calibration, not final precision calibration.
