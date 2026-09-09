# RoboBeetle Protocol V2 — Phase 1 Baseline

This document describes the Console and Firmware sources repaired and clean-built on 2026-09-08, plus the Console APC220 scheduler adaptation. **[Implemented]** refers to code presence and software verification; the pre-PR8 Servo1 hardware acceptance recorded below is explicitly marked **[Hardware Verified]** for its historical layout. PR #7 scheduler behavior is **[Hardware Verified - Bench]** on the tested desktop setup; its timing parameters remain **[Provisional]**. The PR #8 five-servo implementation is software-verified, while its target hardware regression remains pending.

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

## Current PR #8 semantic descriptor contract

PR #8 does not change the Protocol V2 wire format, message IDs, CRC, or COBS rules. It freezes the semantic servo IDs and supported mask independently in the C++ Console and pure-C Firmware descriptor tables. Separate descriptor tests assert the same IDs, masks, capabilities, calibration envelopes, and timer/channel assignments so drift is detected without sharing a C/C++ header.

| ID / mask | Semantic name | Hardware / STM32 output | Capability and command envelope |
|---:|---|---|---|
| `0` / `0x0001` | `FrontRight` / 前足右 | SAVOX SW-0250MG+, TIM3_CH1 / PA6 | PWM 1050–1950 μs; angle −45…+45° |
| `1` / `0x0002` | `FrontLeft` / 前足左 | SAVOX SW-0250MG+, TIM3_CH2 / PA7 | PWM 1050–1950 μs; angle −45…+45° |
| `2` / `0x0004` | `FrontAxis` / 升潜前足轴 | HDKJ S3150D, TIM3_CH3 / PB0 | Electrical 500/1500/2500 μs; command 1200–1800 μs; Set Angle rejected; calibration pending |
| `3` / `0x0008` | `RearRight` / 后足右 | GDW IPX896HV, TIM4_CH1 / PD12 | PWM 1020–2020 μs; angle −45…+45° |
| `4` / `0x0010` | `RearLeft` / 后足左 | GDW IPX896HV, TIM4_CH2 / PD13 | PWM 1020–2020 μs; angle −45…+45° |

`SUPPORTED_SERVO_MASK` is exactly `0x001F`. SAVOX electrical limits are 1000/1500/2000 μs with an accepted command envelope of 1050–1950 μs; GDW electrical limits are 520/1520/2520 μs with an accepted command envelope of 1020–2020 μs. FrontAxis seller metadata records electrical 500/1500/2500 μs, 4.8–7.4 V, 0–270° travel, and 4 μs dead band; those electrical values are capability metadata, not a user command safety range. The accepted FrontAxis command envelope is the matching provisional 1200–1800 μs PWM-only exploration window in both Console and Firmware. The 1500 μs value is a provisional bring-up center candidate, not a calibrated or Hardware Verified mechanical center. The supplied 1480/1500/1520 direction result is Hardware Verified; full-travel endpoints, safe min/max, true center, and angle mapping remain pending. Waterproof capability is **[Unverified]** because the seller parameter page says “not waterproof” while the product photo/shell says “Water proof Robot Servo”; no direct-immersion suitability may be claimed without reliable IP/sealing evidence.

This semantic map is a hardware-layout compatibility break. Historical v0.4 `Servo1`/PA6 bring-up referred to `RearLeft`; PR #8 assigns PA6/ID0 to `FrontRight` and PD13/TIM4_CH2 to `RearLeft`. Do not mix pre-PR8 Console/Firmware binaries with PR8 five-servo wiring. Qt's `ServoId::Servo1` is only a deprecated source alias for `FrontRight`; there is no `Servo2` alias, and new UI/logs/docs use semantic names.

## Payload definitions

### Heartbeat — `0x01`

```text
host_uptime_ms  uint32 LE
```

The Console sends the low 32 bits of its wall-clock millisecond value on the configured heartbeat cadence. The DirectUart baseline uses a 100 ms heartbeat and keeps its original multi-pending behavior. The APC220 scheduler is **[Hardware Verified - Bench]** on the tested desktop setup, while its timing values remain **[Provisional]**: 250 ms is a soft target, and an independent 490 ms **Console host-side/local safety admission budget** is anchored to each heartbeat dispatch/send time. That local scheduler budget intentionally reserves 10 ms below the Firmware's strict greater-than-500 ms watchdog boundary; it is not a Windows-plus-RF hard-real-time guarantee. Before starting ordinary work, the scheduler accounts for the configured ACK timeout plus one retry-timer polling interval; if that worst-case exchange would cross the local budget, it dispatches a heartbeat first. A matching ACK confirms liveness and records RTT, but never moves either deadline forward. Once a deadline is due, no new ordinary user command starts; an already in-flight exchange may finish, then the heartbeat is dispatched before retry or queued work. APC220 sends one fresh heartbeat immediately after `Connected`; user Servo commands may queue but cannot reach the wire until that heartbeat receives a matching successful ACK. Firmware stores the host value only for diagnostics and uses local `HAL_GetTick()` arrival time for liveness.

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

The Console can parse payloads of at least five bytes, validates both the request sequence and `request_type`, removes only a matching pending request, and displays the numeric code. A type-mismatched Error leaves the pending request and any Disable-pending state untouched. Firmware declares `0x03` but does not emit or handle Error frames; it reports command failures as nonzero ACK results and silently counts framing failures.

### Servo Enable / Servo Disable / Neutral — `0x10`, `0x11`, `0x14`

```text
servo_mask  uint16 LE
```

Mask assignments are the frozen five semantic bits from the PR #8 descriptor contract above. Firmware and Console use `SUPPORTED_SERVO_MASK = 0x001F`. A zero mask returns `InvalidPayload`; any bit outside `0x001F` returns `UnsupportedServo` and performs no partial action. Multi-bit Enable is all-or-nothing: requested channels that were already enabled are skipped without pulse write/start/stop, and if a newly requested channel fails to start, only channels newly started by that call are rolled back while the pre-call logical and physical state is preserved. Disable and Disable All retain fail-closed/best-effort stop semantics. While a Disable is pending for a servo, the Console Controller rejects PWM, Neutral, and Set Angle before wire encoding or APC queueing. Neutral otherwise requires a live host and enabled selected channels, writes each descriptor's center/neutral pulse, and returns `OK`; FrontAxis remains PWM-only with calibration pending and its 1500 μs action is provisional.

### Set Servo PWM — `0x12`

```text
count       uint8
items[count]:
    servo_id  uint8
    pulse_us  uint16 LE
```

The wire schema is variable-length, but both current command paths use exactly one item (`payload_length=4`, `count=1`). Console and Firmware accept the five semantic IDs and apply the descriptor-specific command envelopes: SAVOX 1050–1950 μs, FrontAxis 1200–1800 μs, and GDW 1020–2020 μs. FrontAxis's 500–2500 μs electrical metadata does not widen this command envelope. Console additionally requires its local Enable ACK state and no pending Disable for that ID; Firmware requires both a live heartbeat and its own enabled bit.

### Set Servo Angle — `0x13`

```text
count       uint8
items[count]:
    servo_id   uint8
    angle_cdeg int16 LE
```

The type and schema are implemented on both sides:

- Console/controller: PWM, Neutral, and `setServoAngle()` require a supported, enabled semantic servo with no pending Disable; Set Angle also requires angle capability and validates the descriptor range (−45…+45° for current SAVOX/GDW channels). Rejected commands are neither written nor placed in the APC220 queue. Angle is sent as signed `int16` cdeg; Qt does not convert angle to PWM.
- Firmware: validates `count=1`, live heartbeat, an angle-capable semantic ID, enabled state, and the descriptor range. FrontAxis is rejected as unsupported for Set Angle; out-of-range values return `OutOfRange` and are not clamped.
- Firmware performs the descriptor-specific piecewise linear calibration with `int32_t` intermediates (SAVOX electrical 1000/1500/2000 μs; GDW 520/1520/2520 μs), then updates the mapped STM32 timer channel.

### Historical Servo1 Set Angle hardware acceptance — [Hardware Verified] (2026-09-06)

This evidence belongs to the pre-PR #8 Servo1/PA6 layout and is not a hardware verification of the new five-servo wiring.

- Hardware: GDW IPX896HV, driven by `TIM3_CH1 / PA6` at approximately 333 Hz.
- Protocol V2 Set Angle `0x13` passed at 0°, +10°, 0°, −10°, 0°, ±45°, and ±90°.
- Observed correspondence remains approximate bring-up calibration: −90° ≈ 520 μs, 0° = 1520 μs, +90° ≈ 2520 μs.
- Safety/UI behavior passed: Disable blocks Set Angle; Enable before ACK remains unavailable; Enable + ACK restores it; Disable All and Disconnect block it; Reconnect does not auto-enable; manual Enable + ACK restores it.

## Console ↔ Firmware consistency matrix

| Message | ID | Console encode / behavior | Firmware decode / behavior | Payload | ACK | Current status / notes |
|---|---:|---|---|---|---|---|
| Heartbeat | `0x01` | DirectUart sends uint32 LE every 100 ms with multi-pending behavior; APC220 is **[Provisional]**, sends one immediately after connect, then targets 250 ms when the stop-and-wait slot is free; user commands are gated until a matching successful ACK | Requires length 4; records host value and local arrival; sets `host_alive` | `uint32` | Result 0/1 | **Consistent and [Implemented]**; APC220 cadence is a provisional link budget |
| ACK | `0x02` | Does not originate; requires exactly 4 payload bytes and matches request sequence/type | Encodes four-byte payload; TX sequence starts at 0 | `uint16,uint8,uint8` | N/A | **Consistent in active direction** |
| Error | `0x03` | Parses length ≥5; uses request sequence and code | Enum only; no producer/handler | `uint16,uint8,uint16` | N/A | **Incomplete Firmware side** |
| Servo Enable | `0x10` | Sends a supported semantic mask; marks requested IDs enabled after matching result 0 | Requires heartbeat/length 2/valid mask; starts each requested channel at its descriptor neutral; multi-bit start is atomic with rollback | `uint16 mask` | Yes | **Consistent for `0x001F`; unsupported bits fail atomically** |
| Servo Disable | `0x11` | Sends a supported semantic mask; clears requested IDs after matching result 0 | Requires length 2/valid mask; stops each requested channel | `uint16 mask` | Yes | **Consistent for `0x001F`; fail-closed/best-effort stop** |
| Set Servo PWM | `0x12` | Sends count 1, semantic ID, pulse LE; applies descriptor command envelope | Accepts exactly count 1 and a supported semantic ID; host-alive/enabled/range gates; writes mapped timer CCR | `uint8,uint8,uint16` | Yes | **Consistent for all five IDs; FrontAxis is PWM-only at 1200–1800 μs, endpoint verification pending** |
| Set Servo Angle | `0x13` | Controller and Qt UI send count 1, angle-capable semantic ID, signed cdeg LE; UI is gated by connection, Enable ACK, and no pending Disable | Maps each accepted descriptor angle with `int32_t` arithmetic; FrontAxis is rejected | `uint8,uint8,int16` | Yes | **Implemented; new five-servo hardware verification pending** |
| Neutral | `0x14` | Sends the selected semantic mask after local enable | Requires live host, valid mask, and enabled selected channels; writes descriptor neutral pulses without disabling | `uint16 mask` | Yes | **Implemented; FrontAxis neutral remains provisional** |

## Sequence, ACK, retry, and duplicate behavior

- Console request sequences start at 1 and increment as `uint16`; wraparound is implicit.
- Firmware ACK frame sequences start at 0 and increment independently.
- DirectUart keeps every ACK-requiring request in a sequence-keyed table, including heartbeats; this preserves the original multi-pending behavior.
- Apc220HalfDuplex permits one ACK-requiring request in flight. User servo commands wait in a bounded queue of `kApc220CommandQueueCapacity` entries, and heartbeat ticks collapse into one pending/due bit rather than a queue.
- APC220 Servo Disable/Disable All requests use a safety-priority queue. A new Disable clears unsent Servo Enable, Set Servo PWM, Set Servo Angle, and Neutral work for the affected mask (and any deferred retry), then runs after the uncancellable in-flight exchange and any due heartbeat but before ordinary retry/queue work.
- ACK timeout is 200 ms for DirectUart and 250 ms for Apc220HalfDuplex. Retries reuse the original encoded frame and sequence, up to three retransmissions after the original send when the transport accepts the write; DirectUart retains its legacy handling of failed retry writes for regression compatibility.
- When an APC220 heartbeat is due, or when the remaining safety budget cannot contain an ordinary exchange, the heartbeat is dispatched first. The command retry remains deferred with its original sequence/frame and is sent after the heartbeat exchange.
- Firmware caches the most recent successful non-Heartbeat request using `Sequence + MessageType`. An immediate retry of that request replays the cached result-0 ACK and returns before dispatch, so Servo Enable/PWM/Angle/Neutral/Disable are not executed twice.
- Valid Heartbeats always refresh `last_heartbeat_rx_ms` and host liveness, and do not evict the action cache. In DirectUart this matters because the 100 ms heartbeat interval is shorter than the 200 ms ACK timeout; APC220 stop-and-wait scheduling avoids overlapping those exchanges.
- Heartbeat watchdog timeout clears host liveness, the enabled mask, PWM output, and the duplicate cache. A later connection cannot replay a stale successful action to bypass explicit re-enable.
- On the first APC220 heartbeat ACK timeout, the Console converges actuator state fail-closed with the Firmware watchdog's impending state: it clears logical enabled/Disable-pending state, drops queued and deferred actuator work, and keeps heartbeat retry bookkeeping independent. Recovery Heartbeats restore link liveness only; they never replay outage-era Enable, PWM, Angle, or Neutral requests. A new user Servo Enable with a matching ACK is required after recovery. A later terminal timeout remains observable as a transport timeout but is not required for local actuator fail-close.
- This is intentionally a one-entry Phase 1 cache, not a sequence window. A different successful non-Heartbeat request replaces it. With only the last entry retained, 16-bit wrap does not collide with an ancient request after intervening successful commands.
- Console removes a pending request on a matching ACK even if the result is invalid; an ACK type mismatch or rejection is displayed and not retried. A mismatched Error is not allowed to release a request. In APC220 mode a released non-Heartbeat request opens the single in-flight slot, while a heartbeat type mismatch/rejection keeps liveness false and schedules the next heartbeat without dispatching queued user commands.
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

- Console link profiles: DirectUart = 100 ms heartbeat / 200 ms ACK timeout with multiple pending requests; Apc220HalfDuplex scheduler behavior is **[Hardware Verified - Bench]** at a 250 ms soft heartbeat target / 250 ms ACK timeout with one ACK-requiring request in flight. The timing values remain **[Provisional]**. APC also has a 490 ms **Console host-side/local safety admission budget**, measured from each heartbeat dispatch (not from its ACK), with a 10 ms margin below the Firmware watchdog boundary. This is a local admission policy, not a Windows-plus-RF hard-real-time guarantee; RF jitter and host scheduling still require hardware regression.
- The APC220 scheduler's bounded user-command queue is `kApc220CommandQueueCapacity` entries. A full queue rejects new user commands locally; a matching ACK, Error, timeout, or reset releases/clears the associated state.
- APC220 emits a first heartbeat immediately after `Connected`. Until its matching successful ACK, Servo Enable/PWM/Angle/Neutral/Disable requests remain queued. Heartbeat ACK rejection, type mismatch, timeout, or matching Error keeps liveness false and marks the next heartbeat due; it never turns a queued command into a `HostNotAlive` wire request. After a heartbeat timeout fail-closes the actuator state, a new Servo Enable is rejected locally while liveness is recovering and is never queued; heartbeat recovery restores link liveness only, so a fresh user Enable and matching ACK are required. Timer ticks while a heartbeat is in flight are ignored/coalesced, and the next available slot is used for the deferred retry or queued command after the heartbeat exchange. A deadline that becomes due while a command is in flight remains due after that command's ACK, so a second ordinary command cannot slip ahead of the heartbeat.
- Firmware host watchdog: greater than 500 ms since the last valid Heartbeat.
- Watchdog timeout clears `host_alive`, stops all enabled descriptor PWM channels, and clears the enabled mask.
- The measured APC220 RTT of approximately 167–173 ms is an input to test scenarios, not a formal guarantee: 250 + 170 = 420 ms is only an illustrative nominal observation. The local admission boundary is the 490 ms dispatch-anchored **Console host-side/local safety admission budget**; ordinary work is admitted only when its configured worst-case timeout plus retry polling still fits, leaving 10 ms below the 500 ms watchdog boundary. This policy is not a Windows-plus-RF hard-real-time guarantee, and actual radio/host jitter still requires hardware validation.
- On the first missed APC220 Heartbeat ACK, the Console clears its logical enabled state and stale actuator queue before Firmware can diverge at its watchdog boundary. Heartbeat retries may continue, but recovery does not replay old actions; after recovery a new valid Heartbeat and a new Servo Enable ACK are required before PWM/Angle commands can succeed.
- PWM is not started at boot. TIM3/TIM4 CCRs are initialized from the descriptor table, but waveforms start only on accepted Servo Enable.

The APC220 250 ms/250 ms values are a **[Provisional]** link adaptation target, not a Firmware watchdog change. The Console uses a stop-and-wait exchange to avoid overlapping frames on the half-duplex/high-latency path; heartbeat intent is coalesced while a command or retry is active and the actual wire cadence depends on ACK turnaround. Every matching ACK records the latest measured round-trip time in `ProtocolMonitor::lastAckRttMs` and the Qt monitor.

## PR #7 APC220 bench hardware verification

The APC220 Half-Duplex Scheduler is **[Hardware Verified - Bench]** for the user-tested desktop setup. The regression covered the 440 MHz two-module link, the complete Qt → APC220 → STM32 → ACK → APC220 → Qt path, 60 s idle Heartbeat, Servo1 Enable/ACK, Neutral, 0°/±10°/±45°/±90°, rapid queued Set Angle traffic, Disable/Disable All priority, robot-side disconnect, Heartbeat retry/watchdog safe-disable, and recovery without automatic re-arm. CRC errors were 0 during the normal run, and normal desktop ACK RTT was approximately 160–170 ms (approximately 160–173 ms across recorded observations). Deliberately injected Retry/Timeout events are expected fault-injection behavior and are not normal-link timeout statistics.

The 250 ms Heartbeat target, 250 ms ACK timeout, and 490 ms Console host-side/local safety admission budget remain **[Provisional]** pending lab/poolside distance, antenna-orientation, and outdoor RF characterization. The 490 ms value is a host-side/local admission policy, not a Windows-plus-RF hard-real-time guarantee.

## Golden vectors (wire-codec examples)

These frames remain codec regression fixtures. They are not a statement that every payload is inside the current descriptor command envelope (for example, the legacy ID 0 ±90° examples are outside the PR #8 FrontRight ±45° command range).

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

## Historical software baseline and remaining limits (Historical Reference)

The following snapshot predates the PR #8 five-servo layout and is retained for traceability; current descriptor semantics and verification status are defined above.

1. Console clean configure/build and both test executables pass with Qt 6.11.2 / MinGW 13.1.0.
2. Firmware clean configure/build passes with STM32 GCC 14.3.1; the standalone pure-C codec golden-vector test passes with warnings treated as errors.
3. Protocol failures in this baseline use the frozen nonzero ACK results above. `Error (0x03)` remains reserved and is not emitted by Firmware.
4. Duplicate suppression is implemented in the HAL-coupled dispatcher and source-reviewed, but there is still no Firmware host unit-test framework for dispatcher/CCR side effects.
5. Historical Neutral is **[Hardware Verified]** near the mechanical 1520 μs center. Historical Protocol V2 Set Angle `0x13`, its Controller/Qt path, and real Servo1 motion were **[Hardware Verified]** at 0°, ±10°, ±45°, and ±90° on the pre-PR8 GDW IPX896HV (`TIM3_CH1 / PA6`); this does not verify the PR8 five-servo wiring.

Magic, Version, base frame layout, CRC, COBS, baud rate, `.ioc`, and CMake structure were not changed.
