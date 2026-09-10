# JY901S IMU Bring-Up Design

## Goal

Add the first JY901S bring-up path on the STM32F407 without changing the existing USART1 host-link, Protocol V2, Servo, Leak, Safety, or Console behavior. The current host-link evidence uses DAP UART/COM13; the APC220 profile is an earlier/legacy transport record and is not current hardware evidence:

```text
JY901S
  -> USART3 PB11 RX
  -> one-byte interrupt receive
  -> independent 256-byte RX ring buffer
  -> HAL-independent JY901S stream parser
  -> Acc/Gyro/Angle internal state
```

This is a listen-only startup. The application configures USART3 for the wiring and expected serial framing, starts reception, and never sends a JY901S configuration, save, restart, or calibration command.

## Protocol contract

The implementation follows the WIT standard protocol documentation:

- Every normal data frame is 11 bytes.
- Byte 0 is the frame header `0x55`.
- Byte 1 is the frame type.
- Bytes 2–9 contain four signed 16-bit little-endian values.
- Byte 10 is the low 8 bits of the sum of bytes 0–9.
- `0x51` is acceleration, `0x52` is angular velocity, and `0x53` is Euler angle output.
- `0x54` is magnetic-field output. It is a known valid frame type for this bring-up, but it is intentionally not decoded into internal state.
- Acceleration is `raw / 32768 * 16 g`.
- Angular velocity is `raw / 32768 * 2000 deg/s`.
- Roll, pitch, and yaw are `raw / 32768 * 180 deg`.

The vendor product information identifies UART as the digital interface, gives 9600 baud as the default, and specifies 3.3–5 V module supply compatibility. The accessible vendor protocol material identifies the persistent defaults as output mask `0x001E` and rate `10 Hz`: acceleration, gyro, angle, and magnetic-field frames are expected by default. The STM32 USART3 setup will use 8 data bits, no parity, one stop bit, and no flow control, matching the existing USART1 framing and the vendor SDK UART examples. No runtime command will attempt to verify or alter those sensor settings in this phase.

Sources:

- https://wit-motion.cn/proztmz/22.html
- https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol

## Hardware and HAL integration

The active CubeMX configuration will gain only the USART3 resources required by the schematic:

| Resource | Assignment |
|---|---|
| PB10 | USART3_TX; JY901S RX input path, configured but unused by application code |
| PB11 | USART3_RX; JY901S TX input path |
| USART3 | 9600 baud, 8-N-1, TX/RX mode, no hardware flow control |
| USART3 IRQ | Enabled at the same priority as USART1 |

`main.c` will own the generated `huart3`, call a generated `MX_USART3_UART_Init`, and pass both `huart1` and `huart3` to `app_main_init`. `stm32f4xx_hal_msp.c` will configure the USART3 clock, PB10/PB11 alternate function 7, and `USART3_IRQn`. `stm32f4xx_it.c` will add `USART3_IRQHandler` calling `HAL_UART_IRQHandler(&huart3)`.

There will remain exactly one global `HAL_UART_RxCpltCallback`. It will delegate to both transport modules; each module will accept only its own USART instance. USART1 continues to own the host-link ring and transmit path (the current bench uses DAP UART/COM13; the source profile name for its conservative scheduler is retained separately). USART3 will have no transmit wrapper. If the generated project has no existing UART error callback, there will also be one global `HAL_UART_ErrorCallback` that dispatches only USART3 errors to the JY901S transport; USART1 error behavior is otherwise unchanged.

The original design sketch called for the callback to re-arm directly. That
pre-implementation sketch is superseded by the final deferred-rearm design;
it is retained only as checkpoint history. In the final implementation the
USART3 callback performs only these operations in order:

1. Count the received byte.
2. Attempt to push the staging byte into the dedicated ring buffer.
3. Count a successful push or count an overflow/drop.
4. Mark the receive as needing foreground re-arm.

It will not parse frames, update IMU state, send commands, or loop/retry from
interrupt context. `jy901s_transport_stm32_poll()` makes at most one
non-blocking re-arm attempt per foreground call; `HAL_BUSY` is deferred and
does not increment the hard-failure counter.

The transport tracks whether the one-byte receive is armed and whether it needs a foreground re-arm. RX and error callbacks only record the event and mark the pending state. `jy901s_transport_stm32_poll()` is called from `app_main_process`; when recovery is pending, it makes at most one non-blocking `HAL_UART_Receive_IT(..., 1U)` attempt. `HAL_BUSY` is deferred work and does not increment the hard `rx_rearm_failure_count`; `HAL_ERROR` and other non-success statuses do. A successful attempt clears the pending state and restores the armed state, while a failed attempt remains retryable for the next foreground poll. A generation re-check protects a newer callback/error event from being cleared by a stale foreground success transition.

The single global `HAL_UART_ErrorCallback` performs no parsing or recovery loop. For USART3 it records an aggregate UART error count plus diagnostics for the HAL error flags (including overrun, framing, noise, parity, and other/error flags), marks the receive as needs-rearm, and returns. Foreground polling uses the same one-attempt recovery path. USART1 is not routed through this new recovery state.

## Ring buffer design

The existing pure-C ring buffer will be minimally generalized to use caller-provided storage. It will retain the current empty-slot convention, FIFO ordering, non-blocking push/pop behavior, and full-buffer rejection semantics. No dynamic allocation is introduced.

- USART1 supplies the existing 128-byte storage, retaining its 127-byte effective capacity and silent drop behavior at the transport boundary.
- USART3 supplies independent 256-byte storage, giving 255 bytes of effective capacity because one slot is reserved to distinguish full from empty.

At 9600 baud with 8-N-1 framing, the line can deliver at most 960 bytes/s. With the documented default output mask and rate, four 11-byte frames at 10 Hz require approximately 440 bytes/s. The 255-byte USART3 effective capacity therefore represents approximately 0.58 seconds of nominal buffering. This is a bring-up estimate, not a permanent limit: a different persistent output mask or rate must be evaluated using observed RX byte rate, valid-frame rate, and overflow count.

The USART3 transport exposes diagnostics for RX bytes, successful ring pushes, bytes popped by the foreground, overflow/drop count, hard receive re-arm failures, deferred `HAL_BUSY` attempts, and UART error flags. These remain internal/debug-visible and do not become Protocol V2 telemetry.

## Parser design

`jy901s_parser` is a pure C module with no HAL dependency. It accepts one byte per call and owns a fixed 11-byte candidate frame, decoded internal state, and parser counters.

When idle, the parser ignores bytes until `0x55`. Once a candidate starts, it collects exactly 11 bytes. A complete candidate with a bad checksum increments the checksum-error counter and scans the collected bytes after the original header for another `0x55` candidate so a dropped byte cannot permanently desynchronize the stream. A checksum-valid supported frame updates only its corresponding state vector and valid flag. A checksum-valid `0x54` Mag frame is accepted as a known-but-ignored frame: it increments `mag_frame_count` (and the overall valid-frame count), leaves Acc/Gyro/Angle state unchanged, and returns to normal synchronization. A checksum-valid frame with a genuinely unknown type increments `unsupported_frame_count` (and the overall valid-frame count), leaves the supported state unchanged, and returns to idle synchronization. A valid frame immediately following a bad frame is accepted when its header is encountered.

The parser records:

- header/candidate start count;
- valid checksum frame count, including known-ignored Mag and genuinely unknown types;
- checksum error count;
- known-ignored Mag frame count;
- unsupported valid-frame count for types other than `0x51`–`0x54`;
- per-domain update counts where useful;
- valid-frame events that let the application record the last-valid-frame timestamp.

The state contains:

```text
acc_valid,  acc_x_g,   acc_y_g,   acc_z_g
gyro_valid, gyro_x_dps, gyro_y_dps, gyro_z_dps
angle_valid, roll_deg, pitch_deg, yaw_deg
```

The parser will not swap axes, invert signs, apply robot body-frame transforms, fuse sensors, calibrate mounting, or expose telemetry.

## Application data flow

`app_main_init` initializes the parser and USART3 transport after the generated USART3 peripheral initialization. Each `app_main_process` call independently drains the existing USART1 transport and the USART3 transport. USART1 bytes continue through the existing Protocol V2 accumulator. USART3 bytes are fed one at a time into `jy901s_parser`.

The latest internal IMU state and diagnostic structures are available through narrow read-only accessors/debug-visible module state for bring-up inspection. `app_main` records the timestamp of each valid parser event with the existing `HAL_GetTick()` time base; no new clock architecture is introduced. These values are not serialized onto USART1 and do not affect Servo, Leak, Safety, or Protocol V2 behavior.

## Error handling and recovery

- A full USART3 ring buffer drops only the newest byte, records the drop, and keeps the interrupt receiver armed.
- A failed USART3 re-arm marks the transport as needing re-arm and returns. `HAL_BUSY` is deferred rather than counted as a hard failure; `HAL_ERROR` and other non-success statuses remain hard failures. The foreground transport poll retries once per call until it succeeds; no parser, retry loop, or blocking operation is attempted from the ISR.
- USART3 HAL UART errors are recorded, mark the transport as needing re-arm, and are recovered by that same foreground poll path. Error handling does not change USART1 behavior.
- Leading garbage is ignored until the next header.
- Bad checksums do not update state and do not permanently stop parsing.
- Dropped bytes and corrupt frames are recovered through fixed-length candidate rejection plus header resynchronization.
- Unsupported but checksum-valid types are diagnostic events, not stream failures.
- No automatic sensor configuration is triggered by missing frames or any diagnostic counter.

## Verification contract

The pure parser and generalized ring buffer will be verified with host GCC tests before production implementation follows each test-first cycle. Coverage includes single-byte input, full frames, split frames, concatenated frames, leading garbage, bad checksums, dropped bytes, corruption recovery, immediately-following valid frames, valid known-ignored `0x54` Mag frames, genuinely unknown valid frames, all three engineering-unit decoders, and effective-capacity/overflow behavior. A host/mock transport test covers failed USART3 re-arm, needs-rearm state, successful foreground recovery, and reception after recovery; HAL error-flag recovery is additionally covered by HAL syntax/ARM Build evidence if host mocking does not exercise the generated HAL error path.

The existing Firmware regressions and Console CTest suite must remain passing. The ARM target build is a separate evidence category. No Program Verify or physical JY901S result is inferred from host tests or source inspection.

The following was the pre-bench verification contract; the current recorded
status is superseded by the closeout addendum below:

- **Host Test:** parser/ring-buffer behavior proven on the host.
- **ARM Build:** USART3/CubeMX integration proven by `arm-none-eabi-gcc` if the toolchain is available.
- **Program Verify:** STM32 image programming and verification, reported separately from host/physical evidence.
- **Hardware Verified:** physical USART3 reception and real JY901S values, reported only from explicit bench evidence.
- **Pending:** configuration mismatch diagnosis, physical-link quality, body-frame/magnetic calibration, and any future sensor configuration phase.

## Explicit non-goals

This change will not add IMU Protocol V2 telemetry, Qt IMU visualization, depth pressure sensing, Safety actions, IMU-based servo control, stabilization, EKF/sensor fusion, robot-frame transforms, mounting calibration, or automatic JY901S configuration.

## Closeout status addendum — 2026-09-10

The matching PR #11 Firmware + Qt run verified the listen-only JY901S path
through PB11/USART3, the independent ring, the 11-byte parser, Acc/Gyro/Angle
state, and known-but-ignored Mag frames. The host telemetry path was Qt Console
→ Windows COM13 → DAP UART/USB serial bridge → STM32 USART1 → Protocol V2;
APC220 was not used. Post-fix Run A and Run B both reported hard re-arm failures
0 with overflow 0, closing the re-arm follow-up. UART/checksum physical-link
quality, final body-frame mapping, and magnetic/yaw calibration remain Pending.
The one isolated Qt invalid-length event remains an observation because framing
tests pass and Protocol CRC/timeouts remained zero. No standalone PR #11 Program
Verify record is included in this closeout, so that gate remains Pending.
