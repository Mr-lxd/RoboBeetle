# RoboBeetleConsole

RoboBeetleConsole is the Phase 1 Qt 6 / C++20 engineering console for the current direct Windows laptop → serial/APC220 → STM32 bring-up path. This document describes the implementation repaired and verified on 2026-09-08; historical papers, slides, and legacy code are references only.

## Status labels

- **[Implemented]** Confirmed in the current source tree.
- **[Hardware Verified]** Reported by the current development record and handoff notes; this is stronger than a plan but is not derivable from source alone.
- **[Provisional]** Temporary bring-up values or incomplete interfaces that must not be treated as final calibration.
- **[Planned]** Intended future work that is not implemented.
- **[Historical Reference]** Information from old papers, slides, Simulink, CPG, or STM32 projects; it does not describe the current runtime unless independently reconfirmed.

## Current scope

### [Implemented]

- Qt Widgets UI with serial-port discovery, editable port selection, configurable baud rate, and a default of 9600 baud.
- Connect, Disconnect, Refresh, per-servo Enable/Disable, Disable All, Neutral, and explicit Apply PWM controls.
- Protocol V2 with COBS framing, `0x00` delimiter, CRC-16/CCITT-FALSE, little-endian fields, and a 64-byte maximum payload.
- Explicit DirectUart and Apc220HalfDuplex link profiles. DirectUart retains the 100 ms heartbeat / 200 ms ACK timeout and multi-pending behavior; Apc220HalfDuplex uses **[Provisional]** 250 ms / 250 ms stop-and-wait scheduling for the APC220 path.
- Up to three retransmissions after the original send, always reusing the original sequence and encoded frame.
- ACK/Error reception, request-sequence matching, TX/RX hex display, packet/CRC/timeout counters, ACK status, and an event log.
- Shared ACK result meanings `0..6`, with named rejection status in the monitor.
- `ITransport` abstraction with real `SerialTransport` and test-only `FakeTransport` implementations.
- Protocol codec/stream tests and controller behavior tests.
- Servo1 Set Angle UI with a −90.0…+90.0° `QDoubleSpinBox` at 0.1° steps; the UI converts to centidegrees and angle-to-pulse conversion remains authoritative in Firmware.

### [Planned]

- Raspberry Pi onboard service and `TcpTransport`.
- Camera and FOMO/ONNX result visualization.
- IMU, depth, leak, battery, curves, and 3D attitude views.
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
      │   └─ logical enabled-mask state
      └─ MainWindow
          ├─ connection controls
          ├─ two fixed servo panels
          └─ protocol monitor / event log
```

| Component | Current responsibility |
|---|---|
| `main.cpp` | Creates the application, `SerialTransport`, `RobotController`, and `MainWindow`; injects serial-port discovery. |
| `MainWindow` | Converts UI actions into controller calls and displays controller signals. It does not access `QSerialPort` or construct packets. |
| `RobotController` | Owns command payload construction, sequence allocation, profile-aware heartbeat/ACK scheduling, APC220 queue state, logical servo enable state, range gates, and monitor data. |
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
| Heartbeat | [Implemented] / APC target [Provisional] | DirectUart sends every 100 ms. Apc220HalfDuplex sends a fresh heartbeat immediately after `Connected`, gates user commands until a matching successful ACK, then targets a 250 ms cadence when the stop-and-wait slot is free; due ticks coalesce while a heartbeat is in flight. |
| ACK / retry / timeout | [Implemented] | DirectUart tracks multiple requests; Apc220HalfDuplex allows one ACK-requiring request in flight, queues up to `kApc220CommandQueueCapacity` user commands, prioritizes due heartbeat over ordinary command retry, and retries the identical frame at the profile timeout. |
| Servo Enable / Disable | [Implemented] | UI logical enable changes only after a matching successful ACK. |
| Disable All | [Implemented] | Sends the current supported mask `0x0001`; it does not include the unimplemented Servo2 bit. |
| Neutral | [Hardware Verified] | Sends `0x14` with Servo1 mask `0x0001` after Enable ACK; the development record confirms Servo1 returns to mechanical zero near 1520 μs, while Firmware keeps PWM enabled. |
| Apply PWM | [Implemented] | Explicit button; slider movement alone does not transmit. Requires successful Enable ACK and range validation. |
| Set Angle | [Hardware Verified] | Servo1 uses a −90.0…+90.0° input with 0.1° steps; the Qt UI converts to signed cdeg and calls `RobotController::setServoAngle()`. The control requires connection, Servo1 support, Enable ACK, and no pending Disable request. |
| Protocol monitor | [Implemented] | Displays latest TX/RX chunks, packet counts, CRC errors, timeouts, latest matching-ACK RTT, ACK state, and up to 1000 log blocks. |

## Servo1 hardware acceptance (2026-09-06)

The merged Servo1 Set Angle path is **[Hardware Verified]** on the current bring-up hardware:

- Actuator: GDW IPX896HV on `TIM3_CH1 / PA6`, approximately 333 Hz.
- Protocol V2 Set Angle `0x13`: 0°, +10°, 0°, −10°, 0°, ±45°, and ±90° all passed the controlled acceptance.
- Observed provisional correspondence: −90° ≈ 520 μs, 0° = 1520 μs, +90° ≈ 2520 μs.
- Safety/UI acceptance passed: Disable blocks Set Angle; Enable before ACK does not restore it; Enable + ACK restores it; Disable All and Disconnect block it; Reconnect does not auto-enable; manual Enable + ACK restores it.

## Servo model and calibration status

- `ServoId::Servo1 = 0`, `ServoId::Servo2 = 1`.
- Masks are `1 << ServoId`: Servo1=`0x0001`; Servo2=`0x0002` is reserved but unsupported. The current supported mask is exactly `0x0001`.
- **[Provisional]** Current Console limits are 520–2520 μs with neutral 1520 μs.
- **[Provisional]** Servo1 angle command range is −9000…+9000 cdeg (−90.0…+90.0° in the Qt input). The Console transmits this physical unit without converting it to PWM.
- **[Hardware Verified]** The current Servo1 hardware path is GDW IPX896HV on `TIM3_CH1 / PA6` at approximately 333 Hz; the piecewise mapping is still an approximate bring-up calibration: −9000→520 μs, 0→1520 μs, +9000→2520 μs.
- Protocol V2 Set Angle `0x13`, its Controller path, and the Qt UI are **[Hardware Verified]** for the acceptance values above. There is still no persisted or multi-servo calibration model.
- `MainWindow` is hard-coded to two servo panels through fixed-size arrays and index checks. This is adequate for Phase 1 but is not a scalable actuator model.
- Servo2 is visibly labelled **Unsupported / Planned**, and its enable control remains disabled.

The Servo1 **Set Angle** control is implemented and is enabled only when the transport is connected, Servo1 is supported, Servo1 has a successful Enable ACK, and no Disable request is pending. Disable, Disable All, and disconnect immediately disable the angle controls. Servo2 remains visibly **Unsupported / Planned** and cannot send angle commands.

**Neutral — [Hardware Verified]**: the current development record confirms that Neutral returns Servo1 to mechanical zero near 1520 μs. The PWM input currently represents the user's debug input value; it is not guaranteed to mirror the last hardware-confirmed position after Neutral or another command.

**Set Angle real servo motion — [Hardware Verified]**: the controlled Servo1 acceptance passed at 0°, ±10°, ±45°, and ±90°. The mapping remains provisional rather than a final precision calibration.

## Safety behavior and limitations

- Startup never enables a servo.
- PWM and Neutral are rejected locally until Servo Enable has received a matching result-0 ACK.
- Servo1 angle entry and Set Angle are enabled only while connected, supported, after the matching Servo Enable result-0 ACK, and while no Disable request is pending; Disable/Disable All/disconnect close that UI gate immediately.
- Losing the transport or receiving an APC220 transport error clears in-flight requests, queued commands, heartbeat intent, decoder state, and the Console's logical enable mask, then stops the scheduler. Reconnect starts from a disabled state with one fresh heartbeat and never auto-enables a servo.
- Disconnect/application close attempts Disable All, but deliberately does not wait for its ACK before closing. A successful local serial write is not proof that STM32 acted on it.
- Unexpected link loss can only log that Disable All could not be delivered. The STM32 watchdog is the actual link-loss safety boundary.
- Retries reuse the same sequence and frame. Firmware caches the most recent successful non-Heartbeat request by sequence and type and replays its ACK without repeating the Servo action; Heartbeats refresh liveness without evicting that action cache.
- Firmware invalidates the duplicate cache on heartbeat watchdog timeout, preserving the requirement for a new explicit Enable after reconnect.
- The Console rejects Servo2 locally, and Firmware rejects any Servo2/unknown mask bit without partial action.

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

`RobotControllerConfig::bringUpProvisional()` remains the DirectUart baseline used by existing controller tests and direct serial integrations. The application selects `RobotControllerConfig::apc220Provisional()` so the physical laptop → APC220 → STM32 path uses a 250 ms heartbeat and 250 ms ACK timeout.

In Apc220HalfDuplex mode, only one ACK-requiring frame is active. Servo user commands that arrive while it is active wait in the bounded `kApc220CommandQueueCapacity` queue. Heartbeat timer ticks set a single due flag; they never accumulate. When a heartbeat is due alongside a command retry, the heartbeat goes first and the command retry retains its original sequence and encoded frame. Any transport reset clears the in-flight request, queue, heartbeat due flag, and logical enable/pending state.

The monitor exposes `ProtocolMonitor::lastAckRttMs`, and the UI displays the latest matching-ACK RTT for diagnosing the high-latency link. APC220 matching heartbeat rejection/mismatch/timeout keeps the liveness gate closed until a later heartbeat succeeds.

The bring-up evidence and link-isolation lessons are recorded in the root [engineering lessons](../docs/engineering-lessons.md).

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

## Software verification status (2026-09-08 follow-up)

- A fresh MinGW/Qt CMake configure and build succeeds without changing the project CMake structure.
- `protocol_tests`: **PASS**, including CRC/COBS regression, result enum values, Neutral, and −9000/0/+9000 cdeg golden vectors.
- `robot_controller_tests`: **PASS**, including −90/−45/0/+45/+90° to cdeg conversion, PWM boundaries, invalid PWM, Set Angle payload/range, Servo1 enable, Servo2 rejection, Neutral ACK, ACK match/mismatch, identical-frame retry, APC220 first-heartbeat ACK gate, heartbeat coalescing and retry priority, bounded queue release, heartbeat rejection/timeout liveness, Error type validation, error-only/write-failure reset, and DirectUart multi-pending regression.
- Firmware was separately clean-built with the STM32 GCC toolchain. No serial port, MCU flashing, PWM output, or physical Servo movement was performed in this round. APC220 timing values are a Console-side adaptation; the Firmware watchdog remains unchanged.

## Hardware milestones

The current development record marks the following as **[Hardware Verified]**: Qt 6 Console startup; SerialTransport on COM10; USART1 bidirectional traffic; interrupt RX plus ring buffer; Protocol V2 COBS/CRC; heartbeat; STM32 ACK reception in Qt; normal TX/RX packets with CRC error count remaining zero during the recorded run; Servo Enable/Disable; heartbeat watchdog; Set Servo PWM updating TIM3 CCR; TIM3 PWM driving Servo1; Neutral near 1520 μs; Protocol V2 Set Angle `0x13` and the Qt Set Angle UI at 0°, ±10°, ±45°, and ±90°; the corresponding Disable/Enable/ACK/Disable All/Disconnect/Reconnect safety-state behavior; and real GDW IPX896HV motion.

These milestones are recorded from the development/handoff record, not inferred from source. The current provisional correspondence is approximately −90°=520 μs, 0°=1520 μs, +90°=2520 μs; it remains an approximate bring-up calibration, not final precision calibration.
