# RoboBeetleConsole

RoboBeetleConsole is the Phase 1 Qt 6 / C++20 engineering console for the current direct Windows laptop → serial/APC220 → STM32 bring-up path. This document describes the implementation repaired and verified on 2026-09-05; historical papers, slides, and legacy code are references only.

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
- A 100 ms heartbeat, 200 ms ACK timeout, and up to three retransmissions after the original send.
- ACK/Error reception, request-sequence matching, TX/RX hex display, packet/CRC/timeout counters, ACK status, and an event log.
- Shared ACK result meanings `0..6`, with named rejection status in the monitor.
- `ITransport` abstraction with real `SerialTransport` and test-only `FakeTransport` implementations.
- Protocol codec/stream tests and controller behavior tests.
- Servo1 Set Angle packet construction in centidegrees; angle-to-pulse conversion remains authoritative in Firmware.

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
      │   ├─ heartbeat timer
      │   ├─ pending ACK/retry table
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
| `RobotController` | Owns command payload construction, sequence allocation, heartbeats, ACK/retry state, logical servo enable state, range gates, and monitor data. |
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
| Heartbeat | [Implemented] | Starts only after transport state becomes Connected; sends every 100 ms and expects ACK. |
| ACK / retry / timeout | [Implemented] | Tracks by request sequence and type; retries the identical frame after 200 ms, at most three times. |
| Servo Enable / Disable | [Implemented] | UI logical enable changes only after a matching successful ACK. |
| Disable All | [Implemented] | Sends the current supported mask `0x0001`; it does not include the unimplemented Servo2 bit. |
| Neutral | [Implemented] | Sends `0x14` with Servo1 mask `0x0001` after Enable ACK; Firmware maps this semantic request to Servo1 calibration neutral without disabling it. |
| Apply PWM | [Implemented] | Explicit button; slider movement alone does not transmit. Requires successful Enable ACK and range validation. |
| Set Angle | [Implemented in controller / UI disabled] | `RobotController::setServoAngle()` sends count 1, Servo1 ID, and signed `angle_cdeg` in little-endian. The interactive UI button remains disabled pending hardware verification. |
| Protocol monitor | [Implemented] | Displays latest TX/RX chunks, packet counts, CRC errors, timeouts, ACK state, and up to 1000 log blocks. |

## Servo model and calibration status

- `ServoId::Servo1 = 0`, `ServoId::Servo2 = 1`.
- Masks are `1 << ServoId`: Servo1=`0x0001`; Servo2=`0x0002` is reserved but unsupported. The current supported mask is exactly `0x0001`.
- **[Provisional]** Current Console limits are 520–2520 μs with neutral 1520 μs.
- **[Provisional]** Servo1 angle command range is −9000…+9000 cdeg. The Console transmits this physical unit without converting it to PWM.
- Firmware owns the current piecewise linear mapping −9000→520 μs, 0→1520 μs, +9000→2520 μs. There is still no persisted or multi-servo calibration model.
- `MainWindow` is hard-coded to two servo panels through fixed-size arrays and index checks. This is adequate for Phase 1 but is not a scalable actuator model.
- Servo2 is visibly labelled **Unsupported / Planned**, and its enable control remains disabled.

The disabled **Set Angle — UI Not Enabled** button is intentional. The command path and provisional Firmware calibration now exist, but no physical angle-path verification was performed in this software-only round.

## Safety behavior and limitations

- Startup never enables a servo.
- PWM and Neutral are rejected locally until Servo Enable has received a matching result-0 ACK.
- Losing the transport clears pending requests and the Console's logical enable mask.
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

## Repair verification status (2026-09-05)

- A fresh MinGW/Qt CMake configure and build succeeds without changing the project CMake structure.
- `protocol_tests`: **PASS**, including CRC/COBS regression, result enum values, Neutral, and −9000/0/+9000 cdeg golden vectors.
- `robot_controller_tests`: **PASS**, including PWM boundaries, invalid PWM, Set Angle encoding/range, Servo1 enable, Servo2 rejection, Neutral ACK, ACK match/mismatch, and identical-frame retry.
- Firmware was separately clean-built with the STM32 GCC toolchain. No serial port, MCU flashing, PWM output, or physical Servo movement was performed in this round.

## Hardware milestones

The current development record marks the following as **[Hardware Verified]**: Qt 6 Console startup; SerialTransport on COM10; USART1 bidirectional traffic; interrupt RX plus ring buffer; Protocol V2 COBS/CRC; heartbeat; STM32 ACK reception in Qt; normal TX/RX packets with CRC error count remaining zero during the recorded run; Servo Enable/Disable; heartbeat watchdog; Set Servo PWM updating TIM3 CCR; TIM3 PWM driving Servo1; real GDW IPX896HV motion; and mechanical centering near 1520 μs.

These milestones are recorded from the development/handoff record, not inferred from source. The current provisional correspondence is approximately −90°=520 μs, 0°=1520 μs, +90°=2520 μs; it is not final precision calibration.
