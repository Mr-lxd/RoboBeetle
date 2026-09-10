# JY901S low-rate telemetry and Qt bring-up monitor

## Scope and evidence boundary

This stacked phase starts from the reviewed PR #10 HEAD. It consumes the
read-only JY901S state already produced by
`USART3/PB11 -> one-byte interrupt RX -> 256-byte ring -> 11-byte parser` and
publishes a low-rate Protocol V2 snapshot over the existing USART1/APC220
link. It adds no JY901S configuration command, no raw UART passthrough, no
servo/safety linkage, no body-frame transform, no EKF, no depth work, and no
3D/history UI.

`RoboBeetleFirmware/Core/Inc/rb_protocol_v2.h` currently defines message IDs
`0x01`, `0x02`, `0x03`, `0x10`–`0x14`, and `0x20`. A repository-wide source and
protocol-document audit found no `0x21` use or reservation, so this phase
assigns `0x21` to `ImuSnapshot` and records that assignment in the protocol
documentation and both endpoint enums.

The existing Protocol V2 frame remains unchanged: `RB 02`, one-byte message
type, little-endian sequence and payload length, CRC-16/CCITT-FALSE, COBS, and
the trailing zero delimiter. `RBP2_MAX_PAYLOAD` remains 64 bytes and the
generated CubeMX CMake file and `.ioc` remain unchanged in this phase.

## Fixed ImuSnapshot payload

`ImuSnapshot` is an unacknowledged telemetry frame. Its sequence uses the
existing Firmware telemetry sequence, which is separate from the command/ACK
sequence. The payload is exactly 56 bytes; no C/C++ struct is copied to the
wire.

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | schema version | `uint8`, currently `0x01` |
| 1 | 1 | validity flags | bit 0 Acc, bit 1 Gyro, bit 2 Angle; bits 3–7 zero |
| 2 | 6 | Acc X/Y/Z | signed `int16 LE`, mg, nearest integer, ties away from zero |
| 8 | 6 | Gyro X/Y/Z | signed `int16 LE`, 0.1 dps, nearest integer, ties away from zero |
| 14 | 6 | Roll/Pitch/Yaw | signed `int16 LE`, 0.01 degree, nearest integer, ties away from zero |
| 20 | 4 | USART3 RX bytes | `uint32 LE` |
| 24 | 4 | parser header count | `uint32 LE` |
| 28 | 4 | valid frame count | `uint32 LE` |
| 32 | 4 | checksum error count | `uint32 LE` |
| 36 | 4 | ring overflow count | `uint32 LE` |
| 40 | 4 | RX re-arm failure count | `uint32 LE` |
| 44 | 4 | UART error count | `uint32 LE` |
| 48 | 4 | known Mag frame count | `uint32 LE` |
| 52 | 4 | unsupported valid frame count | `uint32 LE` |

If a domain validity bit is clear, all three corresponding fixed-point values
are encoded as zero and the Console displays `--`. Firmware clamps a valid
domain to the representable `int16` range; the parser's normal ranges are
approximately Acc ±16 g, Gyro ±2000 dps, and Angle ±180 degrees. The Console
rejects reserved flags, nonzero values for an invalid domain, and values beyond
those documented engineering ranges. NaN is not a valid live value and is
encoded as zero with the domain value treated as invalid by the producer.

The 56-byte payload produces a 66-byte logical frame, at most a 67-byte COBS
body, and at most a 68-byte wire frame including the delimiter. At 9600 8-N-1,
one 1 Hz IMU frame consumes at most 68 of the approximately 960 serial bytes
per second. With the nominal 250 ms APC220 Heartbeat opportunity, four
Heartbeat+ACK exchanges, up to two 500 ms LeakStatus refreshes, and one IMU
snapshot per second consume at most approximately 222 bytes per second using
the endpoint worst-case frame sizes, before RF turnaround and application
overhead. This is a budget estimate, not a hardware throughput claim.

## Firmware scheduling

Firmware evaluates the existing LeakStatus policy and the new one-second IMU
policy only after an accepted Heartbeat has had its normal ACK transmit return
successfully. A pure-C selector returns exactly one slot:

1. LeakStatus when its first/change/refresh policy is due;
2. ImuSnapshot when LeakStatus is not due and the one-second IMU interval is due;
3. no optional frame otherwise.

Thus a Heartbeat can cause zero or one non-ACK telemetry frame, never both.
The leak policy is marked published only after successful LeakStatus transmit;
the IMU policy is marked only after successful snapshot transmit. The first
valid Heartbeat can publish the first due frame. A later Heartbeat opportunity
allows a still-due IMU frame after a LeakStatus priority publication, so the
existing LeakStatus refresh is not starved. No independent IMU timer or TX
path is introduced, and no ACK sequence or pending-request state is touched.

## Console monitor

The Console decodes `ImuSnapshot` separately from ACK and LeakStatus handling.
`ImuMonitor` owns only the latest validated snapshot and its status:
`Unknown` before data or after disconnect/liveness loss, `Receiving` after a
validated snapshot, `Stale` after 3500 ms without a new snapshot, and `Error`
after a malformed snapshot. Invalid, stale, disconnected, and liveness-lost
states clear the live snapshot so old values cannot be presented as current.
Valid snapshots may carry partial domain validity; each invalid domain remains
`--` while diagnostics are still visible.

The Qt panel is titled `IMU — JY901S` and shows status, Acc, Gyro, Euler angle,
and the selected bring-up counters. It intentionally has no 3D model, plot,
history, control action, or calibration UI. Incoming IMU frames never create,
release, retry, or reorder ACK-pending commands, and existing LeakStatus and
USART1/APC220 behavior remains unchanged.
