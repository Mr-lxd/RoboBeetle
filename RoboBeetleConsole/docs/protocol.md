# RoboBeetle Protocol V2 — Phase 1 Baseline

This document describes the Console and Firmware sources repaired and clean-built on 2026-09-06. **[Implemented]** refers to code presence and software verification; it does not imply hardware coverage.

Evidence labels used across the project are **[Implemented]** (current source), **[Hardware Verified]** (development-record hardware evidence), **[Provisional]** (bring-up value/incomplete contract), **[Planned]** (future work), and **[Historical Reference]** (old papers/code only). This protocol document relies primarily on Implemented evidence; hardware milestones and historical context are kept in the project READMEs and root handoff.

## Logical and wire framing

All multi-byte integers are little-endian in both implementations.

```text
LogicalFrame:
offset  size  field
0       2     Magic          uint8[2] = 52 42  ("RB")
2       1     Version        uint8    = 02
3       1     MessageType    uint8
4       2     Sequence       uint16 LE
6       2     PayloadLength  uint16 LE
8       N     Payload        uint8[N], N <= 64
8+N     2     CRC16          uint16 LE

WireFrame = COBS(LogicalFrame) + 00
```

CRC is CRC-16/CCITT-FALSE:

- Polynomial `0x1021`
- Initial value `0xFFFF`
- RefIn/RefOut false
- XorOut `0x0000`
- Check value for ASCII `123456789`: `0x29B1`
- Coverage from the first Magic byte through the final Payload byte

The trailing `0x00` is a delimiter; it is not part of the COBS body or CRC input. The Console caps an accumulated encoded frame at 96 bytes. Firmware derives a 76-byte maximum wire buffer from its 64-byte payload limit.

## Payload definitions

### Heartbeat — `0x01`

```text
host_uptime_ms  uint32 LE
```

The Console sends the low 32 bits of its wall-clock millisecond value every 100 ms. Firmware stores it only for diagnostics and uses local `HAL_GetTick()` arrival time for liveness.

### ACK — `0x02`

```text
request_sequence  uint16 LE
request_type      uint8
result            uint8
```

The ACK frame has its own independent Firmware TX sequence. The Console matches `request_sequence` and `request_type` to a pending request; result 0 is accepted.

Console and Firmware use the same frozen result values:

| Result | Name | Meaning |
|---:|---|---|
| `0` | `OK` | Accepted |
| `1` | `InvalidPayload` | Invalid payload/count, zero mask, unsupported/default message, or generic rejection |
| `2` | `HostNotAlive` | Host heartbeat not alive |
| `3` | `UnsupportedServo` | Unsupported servo ID or any unsupported mask bit |
| `4` | `ServoNotEnabled` | Servo not enabled |
| `5` | `OutOfRange` | PWM/angle outside provisional range |
| `6` | `HardwareFailure` | `HAL_TIM_PWM_Start()` failed |

### Error — `0x03`

```text
request_sequence  uint16 LE
request_type      uint8
error_code        uint16 LE
```

The Console can parse payloads of at least five bytes, removes the matching pending request, and displays the numeric code. Firmware declares `0x03` but does not emit or handle Error frames; it reports command failures as nonzero ACK results and silently counts framing failures.

### Servo Enable / Servo Disable / Neutral — `0x10`, `0x11`, `0x14`

```text
servo_mask  uint16 LE
```

Mask assignments are:

- Bit 0 / `0x0001`: Servo1 (`ServoId=0`)
- Bit 1 / `0x0002`: reserved Servo2 (`ServoId=1`), not implemented
- Current `SUPPORTED_SERVO_MASK = 0x0001`

Firmware and Console currently permit only Servo1/bit 0. A zero mask returns `InvalidPayload`; any mask containing an unsupported bit, including `0x0002`, `0x0003`, or `0xFFFF`, returns `UnsupportedServo` and performs no partial action. Neutral requires a live host and enabled Servo1, writes the Firmware calibration neutral 1520 μs, keeps PWM enabled, and returns `OK`.

### Set Servo PWM — `0x12`

```text
count       uint8
items[count]:
    servo_id  uint8
    pulse_us  uint16 LE
```

The wire schema is variable-length, but both current command paths use exactly one item (`payload_length=4`, `count=1`). Console can encode ServoId 0 or 1; Firmware accepts only ServoId 0. Both currently gate Servo1 PWM to the **[Provisional]** range 520–2520 μs. Console additionally requires its local Enable ACK state; Firmware requires both a live heartbeat and its own enabled bit.

### Set Servo Angle — `0x13`

```text
count       uint8
items[count]:
    servo_id   uint8
    angle_cdeg int16 LE
```

The type and schema are implemented on both sides:

- Console/controller: `setServoAngle()` requires supported/enabled Servo1, validates −9000…+9000 cdeg, and sends the signed `int16` value directly. It does not convert angle to PWM. The Qt UI provides a Servo1 `QDoubleSpinBox` from −90.0° to +90.0° in 0.1° steps, converts the value to cdeg, and is enabled only after connection plus a successful Enable ACK.
- Firmware: validates `count=1`, live heartbeat, Servo1 ID, enabled state, and the same range. Out-of-range values return `OutOfRange`; they are not clamped.
- Firmware performs a piecewise linear calibration with `int32_t` intermediates: −9000→520 μs, 0→1520 μs, +9000→2520 μs, then updates TIM3 CCR1.

## Console ↔ Firmware consistency matrix

| Message | ID | Console encode / behavior | Firmware decode / behavior | Payload | ACK | Current status / notes |
|---|---:|---|---|---|---|---|
| Heartbeat | `0x01` | Sends uint32 LE every 100 ms; expects ACK | Requires length 4; records host value and local arrival; sets `host_alive` | `uint32` | Result 0/1 | **Consistent and [Implemented]** |
| ACK | `0x02` | Does not originate; requires exactly 4 payload bytes and matches request sequence/type | Encodes four-byte payload; TX sequence starts at 0 | `uint16,uint8,uint8` | N/A | **Consistent in active direction** |
| Error | `0x03` | Parses length ≥5; uses request sequence and code | Enum only; no producer/handler | `uint16,uint8,uint16` | N/A | **Incomplete Firmware side** |
| Servo Enable | `0x10` | Sends Servo1 mask; marks it enabled after matching result 0 | Requires heartbeat/length 2/valid mask; starts Servo1 at 1520 μs | `uint16 mask` | Yes | **Consistent; unsupported bits fail atomically** |
| Servo Disable | `0x11` | Sends only supported mask; clears it after matching result 0 | Requires length 2/valid mask; stops and clears Servo1 | `uint16 mask` | Yes | **Consistent; unsupported bits fail atomically** |
| Set Servo PWM | `0x12` | Sends count 1, ServoId 0, pulse LE; local 520–2520 gate | Accepts exactly count 1 and ServoId 0; host-alive/enabled/range gates; writes TIM3 CCR1 | `uint8,uint8,uint16` | Yes | **Consistent for Servo1; Servo2 rejected** |
| Set Servo Angle | `0x13` | Controller and Qt UI send count 1, ServoId 0, signed cdeg LE; UI is gated by connection and Enable ACK | Maps −9000/0/+9000 cdeg to 520/1520/2520 μs with `int32_t` arithmetic | `uint8,uint8,int16` | Yes | **Protocol/controller/UI implemented; real motion not hardware verified** |
| Neutral | `0x14` | Sends Servo1 mask after local enable | Requires live host, valid mask, and enabled Servo1; writes 1520 μs without disabling | `uint16 mask` | Yes | **Consistent and [Hardware Verified]** |

## Sequence, ACK, retry, and duplicate behavior

- Console request sequences start at 1 and increment as `uint16`; wraparound is implicit.
- Firmware ACK frame sequences start at 0 and increment independently.
- Console keeps every ACK-requiring request in a sequence-keyed table, including heartbeats.
- ACK timeout is 200 ms. The original frame is retransmitted unchanged up to three times; the same sequence is retained.
- Firmware caches the most recent successful non-Heartbeat request using `Sequence + MessageType`. An immediate retry of that request replays the cached result-0 ACK and returns before dispatch, so Servo Enable/PWM/Angle/Neutral/Disable are not executed twice.
- Valid Heartbeats always refresh `last_heartbeat_rx_ms` and host liveness, and do not evict the action cache. This matters because the 100 ms heartbeat interval is shorter than the 200 ms ACK timeout.
- Heartbeat watchdog timeout clears host liveness, the enabled mask, PWM output, and the duplicate cache. A later connection cannot replay a stale successful action to bypass explicit re-enable.
- This is intentionally a one-entry Phase 1 cache, not a sequence window. A different successful non-Heartbeat request replaces it. With only the last entry retained, 16-bit wrap does not collide with an ancient request after intervening successful commands.
- Console removes a pending request on a matching ACK even if type/result are invalid; a type mismatch or rejection is displayed and not retried.
- Firmware does not acknowledge frames that fail COBS, size, Magic, Version, or CRC validation because a trustworthy request identity is unavailable.

Duplicate handling is the most important protocol-level safety gap before adding a lossy two-hop Laptop ↔ Pi ↔ STM32 path.

## Decoder and resynchronization behavior

### Console

- Accepts split frames and multiple sticky frames in one receive chunk.
- Rejects malformed COBS, Magic, Version, length, unknown MessageType, and CRC separately.
- Counts only CRC mismatches in the UI CRC counter; other decode failures are logged but not counted there.
- If encoded input exceeds 96 bytes, emits an invalid-length event and discards until the next delimiter.
- Resets partial-frame state when the transport disconnects or errors.

### Firmware

- Feeds bytes popped from the UART ring buffer into a delimiter-based frame accumulator.
- Counts good decoded frames and bad decode/overflow events.
- Drops an oversized frame until the next `0x00`, then resynchronizes.
- The codec validates framing fields and CRC but does not reject unknown MessageType values; the dispatcher returns result 1 for unsupported types.
- Invalid command payloads usually produce nonzero ACKs but, except for malformed Heartbeat, do not increment `protocol_bad_frames`.

## Timing and safety coupling

- Console heartbeat period: 100 ms.
- Console ACK timeout: 200 ms; three retries after the original transmission.
- Firmware host watchdog: greater than 500 ms since the last valid Heartbeat.
- Watchdog timeout clears `host_alive`, stops Servo1 PWM if enabled, and clears the enabled mask.
- After timeout/reconnect, a new valid Heartbeat and a new Servo Enable are required before PWM/Angle commands can succeed.
- PWM is not started at boot. TIM3 CCR is initialized to 1520, but the waveform starts only on accepted Servo Enable.

## Golden vectors

Complete wire frames below include the trailing `00` delimiter.

| Command | Sequence | Payload | Wire frame |
|---|---:|---|---|
| Heartbeat | 1 | `78 56 34 12` | `06 52 42 02 01 01 02 04 07 78 56 34 12 44 28 00` |
| Servo Enable | 2 | `01 00` | `06 52 42 02 10 02 02 02 02 01 03 45 AD 00` |
| Servo Disable | 3 | `01 00` | `06 52 42 02 11 03 02 02 02 01 03 84 50 00` |
| Set Servo PWM | 4 | `02 00 DC 05 01 40 06` | `06 52 42 02 12 04 02 07 02 02 08 DC 05 01 40 06 21 DB 00` |
| Neutral | 5 | `01 00` | `06 52 42 02 14 05 02 02 02 01 03 C2 A4 00` |
| Set Angle −90° | 6 | `01 00 D8 DC` | `06 52 42 02 13 06 02 04 02 01 05 D8 DC F4 4A 00` |
| Set Angle 0° | 7 | `01 00 00 00` | `06 52 42 02 13 07 02 04 02 01 01 01 03 58 9B 00` |
| Set Angle +90° | 8 | `01 00 28 23` | `06 52 42 02 13 08 02 04 02 01 05 28 23 D4 D9 00` |

The Set Servo PWM vector contains two codec items (Servo0=1500 μs, Servo1=1600 μs). It verifies only the generic codec. Neither current Console command construction nor Firmware dispatch accepts `count=2`.

## Verified software baseline and remaining limits

1. Console clean configure/build and both test executables pass with Qt 6.11.2 / MinGW 13.1.0.
2. Firmware clean configure/build passes with STM32 GCC 14.3.1; the standalone pure-C codec golden-vector test passes with warnings treated as errors.
3. Protocol failures in this baseline use the frozen nonzero ACK results above. `Error (0x03)` remains reserved and is not emitted by Firmware.
4. Duplicate suppression is implemented in the HAL-coupled dispatcher and source-reviewed, but there is still no Firmware host unit-test framework for dispatcher/CCR side effects.
5. Neutral is **[Hardware Verified]** near the mechanical 1520 μs center according to the development record. Set Angle protocol/controller/UI are **[Implemented]** and software-tested, but real Set Angle motion remains **[Not yet hardware verified]** until a controlled Servo1 bench test confirms it.

Magic, Version, base frame layout, CRC, COBS, baud rate, `.ioc`, and CMake structure were not changed.
