# RoboBeetle Protocol V2 — Phase 1 Baseline

This document describes the Console and Firmware sources repaired and clean-built on 2026-09-11, plus the conservative host-link scheduler adaptation, PR #9 leak-status telemetry, PR #11 low-rate JY901S telemetry, PR #12 ROVMAKER depth telemetry, the PR #13 final five-servo calibration contract, and the bench-provisional Motion / SimpleGait foundation. **[Implemented]** refers to code presence and software verification; the pre-PR8 Servo1 hardware acceptance recorded below is explicitly marked **[Hardware Verified]** for its historical layout. The current host-link evidence uses Qt Console → Windows COM13 → DAP UART/USB serial bridge → STM32 USART1 → Protocol V2; APC220 is an earlier/legacy transport record and was not used in the recent runs. PR #7 scheduler behavior remains **[Hardware Verified - Bench]** only for that historical APC220 setup; its timing parameters remain **[Provisional]**. The PR #13 feature-image Hardware Verification for the final five-servo calibration is **PASS**; PR #9 LeakStatus, PR #11 physical JY901S telemetry, and PR #12's stable-connection DepthSnapshot path are **[Hardware Verified]** in their recorded boundaries, while Motion ARM/hardware exercise, connector robustness, and ROVMAKER depth calibration remain pending. The full Motion contract is in [`../../docs/motion-simple-gait.md`](../../docs/motion-simple-gait.md).

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

## Current PR #13 final five-servo semantic descriptor contract

PR #13 does not change the Protocol V2 wire format, message IDs, CRC, or COBS rules. It records the final semantic servo IDs, supported mask, calibration tuples, and independent numeric PWM command bounds already shared by the C++ Console and pure-C Firmware descriptor tables. Separate descriptor tests assert the same IDs, masks, capabilities, calibration envelopes, and timer/channel assignments so drift is detected without sharing a C/C++ header.

| ID / mask | Semantic name | Hardware / STM32 output | Capability and command envelope |
|---:|---|---|---|
| `0` / `0x0001` | `FrontRight` | SAVOX SW-0250MG+, TIM3_CH1 / PA6 | calibration −45/0/+45° = 1000/1450/1900 μs; PWM 1000–1900 μs |
| `1` / `0x0002` | `FrontLeft` | SAVOX SW-0250MG+, TIM3_CH2 / PA7 | calibration −45/0/+45° = 2020/1580/1140 μs; PWM 1140–2020 μs |
| `2` / `0x0004` | `Depth` (`FrontAxis` internal ID) | HDKJ S3150D, TIM3_CH3 / PB0 | calibration −90/0/+90° = 1060/1745/2430 μs; PWM 1060–2430 μs |
| `3` / `0x0008` | `RearRight` | GDW IPX896HV, TIM4_CH1 / PD12 | calibration −45/0/+45° = 1110/1570/2030 μs; PWM 1110–2030 μs |
| `4` / `0x0010` | `RearLeft` | GDW IPX896HV, TIM4_CH2 / PD13 | calibration −45/0/+45° = 1940/1450/960 μs; PWM 960–1940 μs |

`SUPPORTED_SERVO_MASK` is exactly `0x001F`. The final raw PWM command bounds are independent ascending numeric ranges: `FrontRight 1000–1900 μs`, `FrontLeft 1140–2020 μs`, `FrontAxis 1060–2430 μs`, `RearRight 1110–2030 μs`, and `RearLeft 960–1940 μs`. This ascending-bound representation is intentional: the `FrontLeft` and `RearLeft` calibration endpoints descend as logical angle increases, while raw PWM validation still uses numeric min/max order. The four paddle servos use the common logical convention `0° = mechanical neutral`, `+45° = backward paddle stroke / forward propulsion direction`, and `−45° = the opposite direction`; the calibration layer owns neutral differences and mirrored PWM direction. Evidence for the PR #13 feature-image Hardware Verification is kept distinct: `FrontRight` 1450/1900 μs are **[Bench Measured]** and 1000 μs is **[Symmetry-Derived / User Accepted]**; `FrontLeft` 1580/1140 μs are **[Bench Measured]** and 2020 μs is **[Symmetry-Derived / User Accepted]**; all three `RearRight` and `RearLeft` points are **[Bench Hardware Verified]**; the final `FrontAxis` actuator calibration is retained at 1060/1745/2430 μs with feature-image Hardware Verification **PASS**. The derived FrontRight/FrontLeft endpoints are not independently Hardware Verified endpoints. User-facing labels are exact ASCII (`FrontRight`, `FrontLeft`, `Depth`, `RearRight`, `RearLeft`); `FrontAxis` remains the internal identifier and is distinct from the ROVMAKER depth sensor. Waterproof capability is **[Unverified]** because the seller parameter page says “not waterproof” while the product photo/shell says “Water proof Robot Servo”; no direct-immersion suitability may be claimed without reliable IP/sealing evidence.

This semantic map is a hardware-layout compatibility break. Historical v0.4 `Servo1`/PA6 bring-up referred to `RearLeft`; PR #8 assigns PA6/ID0 to `FrontRight` and PD13/TIM4_CH2 to `RearLeft`. Do not mix pre-PR8 Console/Firmware binaries with PR8 five-servo wiring. Qt's `ServoId::Servo1` is only a deprecated source alias for `FrontRight`; there is no `Servo2` alias, and new UI/logs/docs use semantic names.

## Payload definitions

### Heartbeat — `0x01`

```text
host_uptime_ms  uint32 LE
```

The Console sends the low 32 bits of its wall-clock millisecond value on the configured heartbeat cadence. The DirectUart baseline uses a 100 ms heartbeat and keeps its original multi-pending behavior. The legacy-named Apc220HalfDuplex scheduler supplies the conservative host-link policy used by the current wired COM13/DAP/USART1 bench path; its 250 ms timing values remain **[Provisional]** profile parameters. An independent 490 ms **Console host-side/local safety admission budget** is anchored to each heartbeat dispatch/send time. That local scheduler budget intentionally reserves 10 ms below the Firmware's strict greater-than-500 ms watchdog boundary; it is not a Windows-plus-RF hard-real-time guarantee. Before starting ordinary work, the scheduler accounts for the configured ACK timeout plus one retry-timer polling interval; if that worst-case exchange would cross the local budget, it dispatches a heartbeat first. A matching ACK confirms liveness and records RTT, but never moves either deadline forward. Once a deadline is due, no new ordinary user command starts; an already in-flight exchange may finish, then the heartbeat is dispatched before retry or queued work. The profile sends one fresh heartbeat immediately after `Connected`; user Servo commands may queue but cannot reach the wire until that heartbeat receives a matching successful ACK. Firmware stores the host value only for diagnostics and uses local `HAL_GetTick()` arrival time for liveness.

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
| `5` | `OutOfRange` | PWM/angle outside the descriptor command range |
| `6` | `HardwareFailure` | `HAL_TIM_PWM_Start()` failed |
| `7` | `Busy` | Motion ownership or an in-progress Motion transition rejects the request |

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

Mask assignments are the frozen five semantic bits from the PR #13 descriptor contract above. Firmware and Console use `SUPPORTED_SERVO_MASK = 0x001F`. A zero mask returns `InvalidPayload`; any bit outside `0x001F` returns `UnsupportedServo` and performs no partial action. Multi-bit Enable is all-or-nothing: requested channels that were already enabled are skipped without pulse write/start/stop, and if a newly requested channel fails to start, only channels newly started by that call are rolled back while the pre-call logical and physical state is preserved. Disable and Disable All retain fail-closed/best-effort stop semantics, clear logical ownership immediately, and never insert a Neutral write. While a channel's physical safe-stop is pending, the Console Controller rejects PWM, Neutral, Set Angle, and Enable before wire encoding or host-link queueing. Neutral otherwise requires a live host and enabled selected channels, writes each descriptor's final calibrated neutral pulse (`FrontRight 1450`, `FrontLeft 1580`, `FrontAxis 1745`, `RearRight 1570`, `RearLeft 1450 μs`), and returns `OK`.

#### PWM Disable safe-stop

TIM3/TIM4 use PWM mode 1, active-high, up-counting output compare. For a
channel-local Disable, `CNT >= CCR` means the falling edge has already occurred
and the HAL PWM channel may be stopped immediately. If `CNT < CCR`, Firmware
clears the stale `CCxIF`, records a per-channel pending-stop bit, enables only
that channel's `CCxIE`, keeps `CCxE` active, and rechecks `CNT`/`CCR`/`CCxIF`
after arming. The existing HAL timer IRQ clears the compare flag and invokes
the callback after the falling edge; the callback then finalizes that channel
through `HAL_TIM_PWM_Stop()` so HAL channel state remains consistent. A race
may emit one additional complete legal pulse, but never a truncated pulse.

Pending-stop ownership rejects SetAngle, ApplyPWM, Motion writes, and Enable
with `Busy=7`; repeated Disable is idempotent. TIM3/TIM4 channels are finalized
independently. This HAL only disables the shared timer counter after all of its
output channels are disabled, so stopping one channel does not stop another.
With the current `PSC=15`, `ARR=3002`, and 1 μs timer tick, the PWM period is
3003 ticks (about 3 ms / 333 Hz); the physical shutdown bound is at most one
PWM frame plus ISR latency, not the separate 750 ms graceful Motion STOP.
Safety ownership is cleared immediately and SafetySupervisor never waits for
the graceful ramp.

Host tests cover the decision/state policy only. Physical no-jump Disable
remains `Pending Hardware Re-verification`; optional logic-analyzer or
oscilloscope verification must show a final complete legal pulse or a stop
after entry into the LOW window, with no runt pulse.

### LeakStatus — `0x20`

```text
state  uint8
```

The state is `0=UNKNOWN`, `1=DRY`, or `2=WET`. This is an
unacknowledged, monitoring-only telemetry frame; its Firmware sequence is
independent of command/ACK sequence matching. Firmware samples the PA11 leak
input in the main loop and publishes LeakStatus only after an accepted
Heartbeat has had its normal ACK fully transmitted, on the first valid sample,
on a state change, or at most once per 500 ms refresh interval. No independent
telemetry timer or Safety/Servo action is introduced. The Console accepts only
the one-byte values, records the last telemetry time, and returns its display to
`Unknown` on disconnect, host-link liveness loss, invalid payload, or after 1500 ms
(three 500 ms refresh opportunities; provisional) without a valid update.
The PA11 leak detection path, Protocol V2 real-link exchange, and Qt indicator
were **[Hardware Verified]** in the recorded end-to-end acceptance. The prior
persistent `Leak: Unknown` result was traced to programming/build artifact
provenance; rebuilding the correct current ELF and programming and verifying it
restored the complete path. No numeric voltage or response-time values are
asserted here because none were recorded in that acceptance.

### ImuSnapshot — `0x21`

`ImuSnapshot` is an unacknowledged, monitoring-only frame published through the
existing Protocol V2 → STM32 USART1 → DAP UART/USB serial bridge → Windows COM13
host path. Its Firmware sequence comes from the
independent telemetry sequence space; it never enters command pending/ACK
matching and never triggers Servo or Safety behavior.

The payload is exactly 56 bytes. All multi-byte fields are little-endian, and
no C/C++ struct is copied directly to the wire:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| `0` | 1 | schema version | `uint8`, currently `0x01` |
| `1` | 1 | validity flags | bit 0 Acc, bit 1 Gyro, bit 2 Angle; bits 3–7 zero |
| `2` | 6 | Acc X/Y/Z | signed `int16 LE`, mg, nearest integer, ties away from zero |
| `8` | 6 | Gyro X/Y/Z | signed `int16 LE`, 0.1 dps, nearest integer, ties away from zero |
| `14` | 6 | Roll/Pitch/Yaw | signed `int16 LE`, 0.01 degree, nearest integer, ties away from zero |
| `20` | 4 | USART3 RX bytes | `uint32 LE` |
| `24` | 4 | parser header count | `uint32 LE` |
| `28` | 4 | valid frame count | `uint32 LE` |
| `32` | 4 | checksum error count | `uint32 LE` |
| `36` | 4 | ring overflow count | `uint32 LE` |
| `40` | 4 | RX re-arm failure count | `uint32 LE` |
| `44` | 4 | UART error count | `uint32 LE` |
| `48` | 4 | known Mag frame count | `uint32 LE` |
| `52` | 4 | unsupported valid frame count | `uint32 LE` |

The Firmware producer clamps a valid fixed-point value to the signed `int16`
representation. The documented Console ranges are Acc ±16000 mg, Gyro ±20000
deci-dps (±2000 dps), and Angle ±18000 centidegrees (±180 degrees). If a
validity bit is clear, the three corresponding values are encoded as zero and
the Console displays `--`; partial-valid snapshots are allowed. The Console
rejects unknown schema/flags, nonzero values in an invalid domain, and values
outside these ranges. NaN is not a live value and is encoded as zero by the
Firmware producer.

The 56-byte payload produces a 66-byte logical frame, at most a 67-byte COBS
body, and at most a 68-byte wire frame including the delimiter. At 9600 8-N-1,
one 1 Hz snapshot uses at most 68 serial bytes per second. Combining the
nominal four Heartbeat+ACK opportunities per second, up to two LeakStatus
refreshes per second, and one IMU snapshot per second is approximately 222
wire bytes per second using endpoint worst-case sizes, before host-link and
application overhead. This is a conservative DAP/USART1 host-link budget
estimate, not an APC220 throughput claim.
The effective ImuSnapshot refresh is therefore up to approximately 1 Hz under
the nominal accepted Heartbeat cadence, and may be lower when ACK opportunities
are delayed or consumed by pending LeakStatus refreshes; there is no independent
IMU transmit timer.

Firmware evaluates the one-second IMU policy only after an accepted Heartbeat
has completed its normal ACK transmission. It selects at most one optional
frame per opportunity. A due LeakStatus `0x20` always wins the shared due
opportunity. When LeakStatus is not due, the scheduler fairly rotates the due
ImuSnapshot `0x21` and DepthSnapshot `0x22` slots. A failed optional transmit is
not marked published, so the pending policy remains retryable. There is no
independent IMU or depth transmit timer.

### DepthSnapshot — `0x22`

`DepthSnapshot` is an unacknowledged, monitoring-only frame from the
listen-only ROVMAKER decoder-board path. Firmware publishes it only after an
accepted Heartbeat ACK completes, at most one optional telemetry frame per
opportunity, using a separate telemetry sequence. It never enters command/ACK
matching or Servo/Safety behavior.

The payload is exactly 38 bytes, schema version `0x01`; all multi-byte fields
are little-endian and no C/C++ struct is copied directly to the wire:

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| `0` | 1 | schema version | `uint8`, currently `0x01` |
| `1` | 1 | validity flags | bit 0 depth valid, bit 1 temperature valid; bits 2–7 zero |
| `2` | 4 | depth | signed `int32 LE`, millimetres |
| `6` | 2 | temperature | signed `int16 LE`, centi-°C |
| `8` | 2 | sample age | `uint16 LE`, milliseconds; `0xffff` means no valid sample or saturation |
| `10` | 4 | RX bytes | `uint32 LE` |
| `14` | 4 | valid lines | `uint32 LE` |
| `18` | 4 | parse errors | `uint32 LE` |
| `22` | 4 | overlong lines | `uint32 LE` |
| `26` | 4 | RX buffer overflows | `uint32 LE` |
| `30` | 4 | hard RX re-arm failures | `uint32 LE` |
| `34` | 4 | UART errors | `uint32 LE` |

The PR #12 stable-connection hardware run verified the functional path from the
ROVMAKER decoder through USART6/PC7, Firmware, DepthSnapshot, USART1/DAP/COM13,
and the Qt monitor. It showed continuously updating plausible depth/temperature
values, refreshed sample age, increasing RX/valid-line counters, negligible
parse errors, and zero overflow/hard re-arm failures. Disturbing the
sensor-to-decoder cable/connector caused invalid readings or a temporary Stale
state until reseating; connector retention, strain relief, wiring stability,
and post-assembly continuity testing remain pending. This observation is not
classified as a Firmware defect. Absolute zero, installed reference point,
fresh/seawater density, installed offset, and pool accuracy also remain pending.

When a validity bit is clear, its numeric field is encoded as zero and the
receiver must ignore it. If no valid depth sample has ever been received, age is
`0xffff`; otherwise the latest valid depth sample age is explicitly saturated
to the `uint16` range and must not wrap around to appear fresh. The Qt monitor uses
local packet arrival and the existing telemetry lifecycle for liveness; it does
not use `sample_age_ms` as its sole stale decision. The Console rejects an
unknown schema, reserved flags, nonzero invalid numeric fields, or any payload
whose length is not exactly 38 bytes.

The Firmware parser accepts the canonical vendor line
`Depth:XX.XXm Temp:XX.XXC\r\n` and the vendor's documented example form
`Depth:XX.XXm Temp=XX.XXC\r\n`. These are exact bounded grammars: the
implementation does not accept guessed compact `T=...D=...`, arbitrary
separators, substring matches, bare LF, or trailing data. The decoder path is
listen-only and sends no configuration, save, restart, calibration, or other
command. See the official [decoder-board manual](https://docs.rovmaker.cn/产品手册/水深传感器产品手册/深度传感器解算板V1.0.html)
for the canonical format, example, and surface-power/air-zero instruction.

Firmware separately applies a provisional 3000 ms sensor-sample freshness
bound. At expiry it clears depth/temperature validity and zeroes those values
in the snapshot while retaining all transport/parser diagnostics. This is
separate from the one-second publication policy and Qt's 3500 ms host-packet
stale timeout; Qt still uses telemetry lifecycle rather than sample age alone.

### Set Servo PWM — `0x12`

```text
count       uint8
items[count]:
    servo_id  uint8
    pulse_us  uint16 LE
```

The wire schema is variable-length, but both current command paths use exactly one item (`payload_length=4`, `count=1`). Console and Firmware accept the five semantic IDs and apply the final descriptor-specific numeric command bounds: `FrontRight 1000–1900 μs`, `FrontLeft 1140–2020 μs`, `FrontAxis 1060–2430 μs`, `RearRight 1110–2030 μs`, and `RearLeft 960–1940 μs`. The bounds remain ascending numeric ranges even where the signed calibration mapping is inverted. Console additionally requires its local Enable ACK state and no pending Disable for that ID; Firmware requires both a live heartbeat and its own enabled bit.

### Set Servo Angle — `0x13`

```text
count       uint8
items[count]:
    servo_id   uint8
    angle_cdeg int16 LE
```

The type and schema are implemented on both sides:

- Console/controller: PWM, Neutral, and `setServoAngle()` require a supported, enabled semantic servo with no pending Disable; Set Angle also requires angle capability and validates the descriptor range (−45…+45° for the four paddle channels and −90…+90° for `FrontAxis`). Rejected commands are neither written nor placed in the host-link command queue. Angle is sent as signed `int16` cdeg; Qt does not convert angle to PWM.
- Firmware: validates `count=1`, live heartbeat, an angle-capable semantic ID, enabled state, and the descriptor range for all five final descriptors. Out-of-range values return `OutOfRange` and are not clamped.
- Firmware performs the descriptor-specific signed-delta piecewise linear calibration with `int32_t` intermediates and then updates the mapped STM32 timer channel. The final tuples are `FrontRight 1000/1450/1900 μs`, `FrontLeft 2020/1580/1140 μs`, `FrontAxis 1060/1745/2430 μs`, `RearRight 1110/1570/2030 μs`, and `RearLeft 1940/1450/960 μs` for negative endpoint / neutral / positive endpoint.

### Motion / SimpleGait — `0x15`

```text
schema  uint8 = 1
mode    uint8
action  uint8
```

The stable mode order is `STOP=0`, `FORWARD=1`, `BACKWARD=2`, `TURN_LEFT=3`,
`TURN_RIGHT=4`, `ASCEND=5`, `DESCEND=6`; `COUNT=7` is a sentinel and is not
sent. `START=1` is currently accepted for `FORWARD`, `TURN_LEFT`, `TURN_RIGHT`,
`ASCEND`, and `DESCEND`. `BACKWARD` remains enum/protocol-compatible but is
Pending bench verification: the generator and Qt UI reject it as a startable
mode. Ordinary STOP uses `mode=STOP, action=STOP=0` and is accepted without
waiting for the neutral transition.

The Firmware ACK is acceptance-level: successful START enters `RUNNING`, while
successful STOP enters `MOTION_STOPPING` and retains Motion ownership. A
centralized 750 ms **[Provisional]** cooperative wall-time ramp drives
amplitude/bias and logical targets to zero before releasing ownership and
entering STOPPED. During STOPPING, manual Servo Enable/SetPWM/SetAngle/Neutral
requests map to `Busy=7`. Disable All and SafetySupervisor host-liveness loss
immediately abort Motion and disable/stop actuators; they never wait for the
ramp. An explicit single-channel Disable aborts Motion only when its validated
mask intersects active Motion ownership; a non-intersecting Disable remains
allowed. A successful `0x15` request participates in the existing one-entry
successful duplicate cache, so same sequence/type retry replays the ACK without
repeating the transition.

The Controller also treats unresolved in-flight, queued, or deferred Motion
work as active for local arbitration. STOP cancels stale queued/deferred Motion
requests and marks an in-flight Motion request cancelled before submitting the
canonical STOP. DirectUart can transmit STOP immediately alongside the
cancelled request; Apc220HalfDuplex retains one-flight ordering and gives STOP
a dedicated lane above ordinary work but below the safety-priority Disable
lane. A late START or mode-change ACK is ignored and cannot resurrect Running
state. ServoService's logical-pose contract is explicit: Enable, SetAngle,
Neutral, and Motion angle writes establish known pose; raw SetPWM marks pose
unknown; Motion START rejects an unknown required pose using the existing
`HardwareFailure=6` result and rejects a known rear pose outside the
`-3000…+4500 cdeg` operational envelope before Motion ownership is acquired.
During the initial START ramp or a mode cross-fade, only a repeated START for
the current transition target is idempotently accepted; a different mode maps
to `Busy=7` and cannot overwrite the active transition. STOPPING re-applies the
same rear envelope guard after interpolation.

The Qt Controller displays `Running`, acceptance-time `Stopping`, timer-based
`Stopped`, or `Faulted`; the UI timer is not actuator confirmation. Reconnect
does not auto-resume an interrupted Motion. The current PA11 leak path remains
monitoring-only and has no leak-to-Safety trip; any future leak trip must use
the same immediate fail-safe path.

### Historical Servo1 Set Angle hardware acceptance — [Hardware Verified] (2026-09-06)

This evidence belongs to the pre-PR #8 Servo1/PA6 layout and is not a hardware verification of the new five-servo wiring.

- Hardware: GDW IPX896HV, driven by `TIM3_CH1 / PA6` at approximately 333 Hz.
- Protocol V2 Set Angle `0x13` passed at 0°, +10°, 0°, −10°, 0°, ±45°, and ±90°.
- Observed correspondence remains approximate bring-up calibration: −90° ≈ 520 μs, 0° = 1520 μs, +90° ≈ 2520 μs.
- Safety/UI behavior passed: Disable blocks Set Angle; Enable before ACK remains unavailable; Enable + ACK restores it; Disable All and Disconnect block it; Reconnect does not auto-enable; manual Enable + ACK restores it.

## Console ↔ Firmware consistency matrix

| Message | ID | Console encode / behavior | Firmware decode / behavior | Payload | ACK | Current status / notes |
|---|---:|---|---|---|---|---|
| Heartbeat | `0x01` | DirectUart sends uint32 LE every 100 ms with multi-pending behavior; the legacy-named Apc220HalfDuplex policy sends one immediately after connect, then targets 250 ms when the stop-and-wait slot is free on the current DAP/COM13/USART1 host link; user commands are gated until a matching successful ACK | Requires length 4; records host value and local arrival; sets `host_alive` | `uint32` | Result 0/1 | **Consistent and [Implemented]** on the current DAP host link; 250 ms remains a provisional policy parameter, not APC220 hardware evidence |
| ACK | `0x02` | Does not originate; requires exactly 4 payload bytes and matches request sequence/type | Encodes four-byte payload; TX sequence starts at 0 | `uint16,uint8,uint8` | N/A | **Consistent in active direction** |
| Error | `0x03` | Parses length ≥5; uses request sequence and code | Enum only; no producer/handler | `uint16,uint8,uint16` | N/A | **Incomplete Firmware side** |
| Servo Enable | `0x10` | Sends a supported semantic mask; marks requested IDs enabled after matching result 0 | Requires heartbeat/length 2/valid mask; starts each requested channel at its descriptor neutral; multi-bit start is atomic with rollback | `uint16 mask` | Yes | **Consistent for `0x001F`; unsupported bits fail atomically** |
| Servo Disable | `0x11` | Sends a supported semantic mask; clears requested IDs after matching result 0 | Requires length 2/valid mask; stops each requested channel | `uint16 mask` | Yes | **Consistent for `0x001F`; fail-closed/best-effort stop** |
| Set Servo PWM | `0x12` | Sends count 1, semantic ID, pulse LE; applies descriptor command envelope | Accepts exactly count 1 and a supported semantic ID; host-alive/enabled/range gates; writes mapped timer CCR | `uint8,uint8,uint16` | Yes | **Consistent for all five IDs; final numeric bounds are recorded above, including inverted left-side calibration** |
| Set Servo Angle | `0x13` | Controller and Qt UI send count 1, angle-capable semantic ID, signed cdeg LE; UI is gated by connection, Enable ACK, and no pending Disable | Maps each accepted descriptor angle with `int32_t` arithmetic, including FrontAxis −90…+90° | `uint8,uint8,int16` | Yes | **Implemented; PR #13 feature-image Hardware Verification: PASS** |
| Neutral | `0x14` | Sends the selected semantic mask after local enable | Requires live host, valid mask, and enabled selected channels; writes descriptor neutral pulses without disabling | `uint16 mask` | Yes | **Implemented; final five-servo neutral values recorded above; PR #13 feature-image Hardware Verification: PASS** |
| SetMotionMode | `0x15` | Sends `schema=1, mode, action`; successful STOP ACK displays `Stopping` and starts the provisional local transition timer; STOP supersedes unresolved Motion work | Validates exact three-byte payload, HostAlive for START, mode/action relation, ownership, and returns acceptance-level ACK; ordinary STOP enters `MOTION_STOPPING` and ramps targets to neutral over actual elapsed 750 ms while retaining Motion ownership; common Motion output guard enforces the rear envelope | `uint8,uint8,uint8` | Yes | **Implemented / Host Test: PASS; gait profile and ARM/hardware exercise [Pending]** |
| LeakStatus | `0x20` | Receives one-byte monitoring telemetry and updates Unknown/Dry/Wet indicator; never creates an ACK pending entry | Samples PA11 and emits after accepted Heartbeat ACK, first/change/500 ms refresh; no ACK and no Servo/Safety action | `uint8 state` | No | **Implemented; end-to-end monitoring [Hardware Verified]** |
| ImuSnapshot | `0x21` | Decodes fixed 56-byte monitoring telemetry into `ImuMonitor`; never creates or releases an ACK pending entry; displays Unknown/Receiving/Stale/Error and clears invalid/stale values | Encodes current JY901S state and diagnostics after accepted Heartbeat ACK, at most one optional frame per opportunity, with due LeakStatus priority and fair rotation against a due DepthSnapshot when LeakStatus is not due | 56-byte fixed schema | No | **Implemented / Host Test: PASS; physical JY901S telemetry [Hardware Verified]** |
| DepthSnapshot | `0x22` | Decodes fixed 38-byte monitoring telemetry into `DepthMonitor`; validates schema/flags/length and never creates or releases an ACK pending entry; sensor-invalid snapshots are Stale with values hidden while diagnostics remain visible | Parses the listen-only ROVMAKER decoder line, applies the provisional 3000 ms sensor freshness bound, and encodes validity-gated fixed-point fields and seven diagnostics counters after accepted Heartbeat ACK, at most one optional frame per opportunity | 38-byte fixed schema | No | **Implemented / Host Test: PASS; physical decoder path [Pending Hardware Verification]** |

## Sequence, ACK, retry, and duplicate behavior

- Console request sequences start at 1 and increment as `uint16`; wraparound is implicit.
- Firmware ACK frame sequences start at 0 and increment independently.
- DirectUart keeps every ACK-requiring request in a sequence-keyed table, including heartbeats; this preserves the original multi-pending behavior.
- Apc220HalfDuplex permits one ACK-requiring request in flight. User servo commands wait in a bounded queue of `kApc220CommandQueueCapacity` entries, and heartbeat ticks collapse into one pending/due bit rather than a queue. The profile name is retained for compatibility; the current hardware path is DAP UART/COM13/USART1.
- Servo Disable/Disable All requests in this single-flight profile use a safety-priority queue. A new Disable clears unsent Servo Enable, Set Servo PWM, Set Servo Angle, and Neutral work for the affected mask (and any deferred retry), then runs after the uncancellable in-flight exchange and any due heartbeat but before ordinary retry/queue work.
- `SetMotionMode` is an ACK-requiring actuator command and uses the same DirectUart multi-pending or Apc220HalfDuplex bounded-queue bookkeeping. STOP cancels queued/deferred Motion work and marks stale in-flight Motion requests cancelled; APC schedules the canonical STOP above ordinary work while keeping one ACK-requiring exchange in flight. Disable All clears all graceful-stop work and keeps only the safety-priority Disable request. A successful Motion STOP ACK is cached at acceptance; the later 750 ms neutral completion is a local/Firmware state transition, not a second ACK.
- ACK timeout is 200 ms for DirectUart and 250 ms for Apc220HalfDuplex. Retries reuse the original encoded frame and sequence, up to three retransmissions after the original send when the transport accepts the write; DirectUart retains its legacy handling of failed retry writes for regression compatibility.
- When the profile heartbeat is due, or when the remaining safety budget cannot contain an ordinary exchange, the heartbeat is dispatched first. The command retry remains deferred with its original sequence/frame and is sent after the heartbeat exchange.
- Firmware caches the most recent successful non-Heartbeat request using `Sequence + MessageType`. An immediate retry of that request replays the cached result-0 ACK and returns before dispatch, so Servo Enable/PWM/Angle/Neutral/Disable are not executed twice.
- Valid Heartbeats always refresh `last_heartbeat_rx_ms` and host liveness, and do not evict the action cache. In DirectUart this matters because the 100 ms heartbeat interval is shorter than the 200 ms ACK timeout; the single-flight policy avoids overlapping those exchanges.
- Heartbeat watchdog timeout clears host liveness, the enabled mask, PWM output, and the duplicate cache. A later connection cannot replay a stale successful action to bypass explicit re-enable.
- On the first single-flight heartbeat ACK timeout, the Console converges actuator state fail-closed with the Firmware watchdog's impending state: it clears logical enabled/Disable-pending state, drops queued and deferred actuator work, and keeps heartbeat retry bookkeeping independent. Recovery Heartbeats restore link liveness only; they never replay outage-era Enable, PWM, Angle, or Neutral requests. A new user Servo Enable with a matching ACK is required after recovery. A later terminal timeout remains observable as a transport timeout but is not required for local actuator fail-close.
- This is intentionally a one-entry Phase 1 cache, not a sequence window. A different successful non-Heartbeat request replaces it. With only the last entry retained, 16-bit wrap does not collide with an ancient request after intervening successful commands.
- Console removes a pending request on a matching ACK even if the result is invalid; an ACK type mismatch or rejection is displayed and not retried. A mismatched Error is not allowed to release a request. In the single-flight mode a released non-Heartbeat request opens the in-flight slot, while a heartbeat type mismatch/rejection keeps liveness false and schedules the next heartbeat without dispatching queued user commands.
- Firmware does not acknowledge frames that fail COBS, size, Magic, Version, or CRC validation because a trustworthy request identity is unavailable.
- ImuSnapshot is intentionally unacknowledged. Its sequence is telemetry-only; receiving it cannot satisfy, release, retry, reorder, or mutate any command/ACK pending request. The Console additionally ignores it while host-link heartbeat liveness is not ready.

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

- Console link profiles: DirectUart = 100 ms heartbeat / 200 ms ACK timeout with multiple pending requests; the legacy-named Apc220HalfDuplex policy is **[Hardware Verified]** as software behavior on the current DAP UART/COM13/USART1 host link, with a 250 ms soft heartbeat target / 250 ms ACK timeout and one ACK-requiring request in flight. The timing values remain **[Provisional]**. The policy also has a 490 ms **Console host-side/local safety admission budget**, measured from each heartbeat dispatch (not from its ACK), with a 10 ms margin below the Firmware watchdog boundary. This is a local admission policy, not evidence of APC220 hardware or a Windows-plus-RF hard-real-time guarantee.
- The conservative host-link scheduler's bounded user-command queue is `kApc220CommandQueueCapacity` entries. A full queue rejects new user commands locally; a matching ACK, Error, timeout, or reset releases/clears the associated state.
- The single-flight profile emits a first heartbeat immediately after `Connected`. Until its matching successful ACK, Servo Enable/PWM/Angle/Neutral/Disable requests remain queued. Heartbeat ACK rejection, type mismatch, timeout, or matching Error keeps liveness false and marks the next heartbeat due; it never turns a queued command into a `HostNotAlive` wire request. After a heartbeat timeout fail-closes the actuator state, a new Servo Enable is rejected locally while liveness is recovering and is never queued; heartbeat recovery restores link liveness only, so a fresh user Enable and matching ACK are required. Timer ticks while a heartbeat is in flight are ignored/coalesced, and the next available slot is used for the deferred retry or queued command after the heartbeat exchange. A deadline that becomes due while a command is in flight remains due after that command's ACK, so a second ordinary command cannot slip ahead of the heartbeat.
- Firmware host watchdog: greater than 500 ms since the last valid Heartbeat.
- Watchdog timeout clears `host_alive`, stops all enabled descriptor PWM channels, and clears the enabled mask.
- The historical APC220 RTT of approximately 167–173 ms is an input to scheduler test scenarios, not a current DAP/COM13 measurement or formal guarantee: 250 + 170 = 420 ms is only an illustrative nominal observation. The local admission boundary is the 490 ms dispatch-anchored **Console host-side/local safety admission budget**; ordinary work is admitted only when its configured worst-case timeout plus retry polling still fits, leaving 10 ms below the 500 ms watchdog boundary. Host scheduling and future APC220 radio jitter require separate validation.
- On the first missed host-link Heartbeat ACK, the Console clears its logical enabled state and stale actuator queue before Firmware can diverge at its watchdog boundary. Heartbeat retries may continue, but recovery does not replay old actions; after recovery a new valid Heartbeat and a new Servo Enable ACK are required before PWM/Angle commands can succeed.
- PWM is not started at boot. TIM3/TIM4 CCRs are initialized from the descriptor table, but waveforms start only on accepted Servo Enable.
- The low-rate JY901S ImuSnapshot and depth policies run only in an existing accepted-Heartbeat opportunity. A due LeakStatus always preempts the optional IMU/Depth slots; when LeakStatus is not due, the scheduler fairly rotates due IMU and Depth publications. At most one non-ACK telemetry frame is sent after each completed Heartbeat ACK. Firmware marks a sensor sample unusable after the provisional 3000 ms depth freshness bound, while the Console marks host telemetry stale after 3500 ms without a valid live packet; both are monitoring policies, not control or calibration paths.

The 250 ms/250 ms values are a **[Provisional]** conservative host-link policy target, not a Firmware watchdog change. The current DAP UART/COM13/USART1 path uses a stop-and-wait exchange to avoid overlapping frames; heartbeat intent is coalesced while a command or retry is active and the actual wire cadence depends on ACK turnaround. The Apc220HalfDuplex name is retained for source compatibility and historical scheduler context; it is not evidence of a current APC220 radio. Every matching ACK records the latest measured round-trip time in `ProtocolMonitor::lastAckRttMs` and the Qt monitor.

## [Historical Reference] PR #7 APC220 bench hardware verification

The APC220 Half-Duplex Scheduler is **[Hardware Verified - Bench]** for the
user-tested historical desktop setup. This section is not the current DAP
UART/COM13/USART1 evidence and does not make APC220 part of the currently
enabled hardware path. The regression covered the 440 MHz two-module link, the
complete Qt → APC220 → STM32 → ACK → APC220 → Qt path, 60 s idle Heartbeat,
Servo1 Enable/ACK, Neutral, 0°/±10°/±45°/±90°, rapid queued Set Angle traffic,
Disable/Disable All priority, robot-side disconnect, Heartbeat retry/watchdog
safe-disable, and recovery without automatic re-arm. CRC errors were 0 during
the normal run, and normal desktop ACK RTT was approximately 160–170 ms
(approximately 160–173 ms across recorded observations). Deliberately injected
Retry/Timeout events are expected fault-injection behavior and are not
normal-link timeout statistics.

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
