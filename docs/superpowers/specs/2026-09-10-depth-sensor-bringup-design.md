# Depth sensor and decoder-board bring-up design

## Status and scope

This design is approved for implementation after the `DepthSnapshot` wire
contract was frozen by the user. It starts from the merged `main` baseline and
adds the smallest listen-only path:

```text
ROVMAKER MS5837
  -> decoder-board ASCII output
  -> STM32 USART6 / PC7 RX
  -> dedicated one-byte interrupt receive
  -> independent ring buffer
  -> bounded foreground line parser
  -> internal depth state
  -> Protocol V2 DepthSnapshot
  -> existing USART1/DAP/COM13 link
  -> Qt read-only Depth monitor
```

This phase does not send decoder-board configuration, save, reset, calibration,
or density commands. It does not change Servo, Leak, Safety, JY901S parser or
transport behavior, Protocol V2 framing, or the existing command/ACK path.
Body-frame calibration, seawater configuration, and depth-sensor electrical
level verification remain outside software verification.

## Vendor format boundary

The parser accepts exactly these two documented input grammars, both terminated
by `\\r\\n`:

```text
Depth:<signed-decimal-with-two-fractional-digits>m Temp:<signed-decimal-with-two-fractional-digits>C\\r\\n
Depth:<signed-decimal-with-two-fractional-digits>m Temp=<signed-decimal-with-two-fractional-digits>C\\r\\n
```

The first is the decoder-board manual's canonical output format. The second is
the exact `Depth:1.21m Temp=25.27C` example recorded in the same official
decoder-board manual. The parser does not accept an inferred compact
`T=...D=...` form. No whitespace, substring, arbitrary separator, bare-LF, or
trailing-data variant is accepted. The manual also records 115200 8-N-1 and
surface power/air-zero guidance; see the [official decoder-board manual](https://docs.rovmaker.cn/产品手册/水深传感器产品手册/深度传感器解算板V1.0.html).

Signs are optional and both `+` and `-` are accepted. Negative depth is not
rejected merely because the board is above water; it can represent zeroing,
offset, or pressure-drift behavior. Numeric conversion is deterministic fixed
point: depth metres become signed millimetres and temperature degrees Celsius
become signed centi-degrees. Values must fit their destination types and use
exactly two fractional digits; no `sscanf`, `strtof`, `atof`, or permissive
prefix parsing is used.

The line collector tolerates input split at any byte boundary and multiple
back-to-back lines. A malformed complete line increments `parse_error_count`.
An overlong line is discarded through its terminator and increments only
`overlong_line_count`, keeping the two diagnostics distinct. The parser then
accepts the next complete valid line.

## STM32 receive transport

USART6 is configured as 115200 8-N-1 with PC6 TX / PC7 RX, AF8, and a matching
IRQ/MSP path. TX is physically configured for the board interface, but no
application code sends decoder commands.

The receive path uses one-byte `HAL_UART_Receive_IT` and a dedicated 512-byte
ring buffer (511-byte effective capacity). The interrupt callback only records
the received byte, increments byte/overflow diagnostics, and attempts the
minimum receive-state transition. It never parses a line, blocks, or loops on
re-arm.

If a completion callback cannot immediately re-arm, it marks the transport as
needing foreground maintenance. `HAL_BUSY` is a deferred state and is not a
hard failure; the foreground poll retries once when called. A genuine
`HAL_ERROR` increments `hard_rearm_failure_count` and remains observable while
the same foreground path continues attempting recovery. UART errors are
recorded, including aggregate and subtype counters where the HAL exposes the
error code, and also mark the receive path for foreground re-arm. USART1 and
USART3 callback routing remains instance-gated and unchanged.

The foreground application drains the ring into the line parser. A valid line
updates the latest sample and its tick timestamp. Transport counters and parser
counters are copied into the telemetry snapshot without sharing storage with
USART1 or USART3.

## Internal state and freshness

The depth state contains:

- `depth_valid` and `temperature_valid`;
- `depth_mm` as signed `int32_t`;
- `temperature_centi_c` as signed `int16_t`;
- `last_valid_sample_ms`;
- parser, ring, re-arm, and UART diagnostics.

Every accepted line supplies both measurements. The separate validity bits are
retained for an explicit wire contract and future sensor-side partial validity.
The firmware does not apply a zero offset or density conversion of its own.

`sample_age_ms` is calculated from the current tick and the latest valid depth
sample. It is `0xFFFF` when no valid depth sample exists and otherwise saturates
at `65535`; it never wraps into a value that makes an old sample look new.
Firmware additionally treats a sample as current only while its wrap-safe age is
less than the provisional `DEPTH_TELEMETRY_SENSOR_FRESHNESS_TIMEOUT_MS` of
3000 ms. This is separate from the one-second publication interval and Qt's
host-packet stale timeout. At expiry the published validity bits and numeric
values are cleared, while diagnostics remain available.

## Frozen Protocol V2 contract

`DepthSnapshot` uses message ID `0x22`, schema version `1`, little-endian
fields, and an exactly 38-byte payload. It is an unacknowledged telemetry
frame and does not change the base Protocol V2 framing.

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | schema | `uint8`, always `1` |
| 1 | 1 | flags | bit 0 depth valid, bit 1 temperature valid; other bits zero |
| 2 | 4 | depth | signed `int32 LE`, millimetres |
| 6 | 2 | temperature | signed `int16 LE`, centi-degrees Celsius |
| 8 | 2 | sample age | `uint16 LE`, milliseconds, `0xFFFF` unknown/saturated |
| 10 | 4 | RX byte count | `uint32 LE` |
| 14 | 4 | valid line count | `uint32 LE` |
| 18 | 4 | parse error count | `uint32 LE` |
| 22 | 4 | overlong line count | `uint32 LE` |
| 26 | 4 | RX ring overflow count | `uint32 LE` |
| 30 | 4 | hard re-arm failure count | `uint32 LE` |
| 34 | 4 | UART error count | `uint32 LE` |

When a validity bit is clear, its numeric field is encoded as zero and must be
ignored by receivers. With no valid sample, age is `0xFFFF`. The Qt monitor
uses local telemetry arrival/liveness for its lifecycle and does not use age as
its sole stale decision.

The 38-byte payload yields a 48-byte logical frame before COBS, at most a
50-byte wire frame including the delimiter. The firmware publishes at a
provisional one-second policy interval, giving an approximately 1 Hz host
telemetry opportunity rather than promising the decoder's undocumented sensor
cadence.

## Telemetry scheduling

After a Heartbeat is accepted and its ACK transmit succeeds, firmware emits at
most one optional telemetry frame. A due LeakStatus is always selected first;
when LeakStatus is not due, a pure-C cursor fairly rotates due IMU and Depth.
The preference order when no LeakStatus is due is:

| Last successful slot | Preference order when LeakStatus is not due |
|---|---|
| none | IMU, Depth |
| Leak | IMU, Depth |
| IMU | Depth, IMU |
| Depth | IMU, Depth |

If `leak_due` is true, the selector immediately returns LeakStatus. Otherwise
the cursor scans only the due IMU and Depth slots in fair order. A failed send
does not advance the cursor or mark the policy published, and at most one
optional frame is emitted per opportunity.

## Qt monitor

Qt adds `DepthSnapshot` decoding and a read-only `DepthMonitor`, routed before
ACK processing just like the existing telemetry monitors. It validates exact
payload length, schema, reserved flags, signed fields, and validity/zero rules.

The panel displays `Unknown`, `Receiving`, `Stale`, or `Error`, depth in metres,
temperature in degrees Celsius, sample age, and the frozen diagnostics. Invalid
or sensor-stale fields display `--` while received diagnostics remain visible;
disconnected and host-liveness-lost states reset to `Unknown`. The panel adds no
plot, command, calibration, or control action.

## Verification boundary

Each implementation slice has a test-first red/green checkpoint. The final
gates discover and run every existing Firmware regression, all Console CTests,
new depth tests, direct-standard-header checks, `.ioc`/generated-HAL syntax
checks, and `git diff --check`. No hardware is programmed by Codex.

The initial implementation phase kept these labels separate:

```text
Host Test: evidence from host tests
ARM Build: only PASS with an actual ARM toolchain/build
Program Verify: Pending until the user programs and checks the board
Hardware Verified: Pending until the user supplies real depth hardware evidence
```

The user-supplied hardware result and the updated closeout labels are recorded
in the closeout validation section below; the implementation boundary itself
still forbids Codex from programming hardware.

Electrical signal level, physical sealed mounting, the vendor surface-zero
procedure, actual decoder cadence, seawater density selection, and body-frame
mapping are explicitly documented as hardware/future verification items. The
local Raspberry Pi `ms5837.py` source is sensor-level direct-I2C reference only;
it does not justify a second Firmware I2C implementation. A future
laptop/network-or-tether → onboard Raspberry Pi/ROS 2 → local serial → STM32
split remains architecture documentation, not implementation in this phase.

## Closeout validation — 2026-09-11

The user-supplied real-hardware result for PR #12 records ARM Build **PASS**
(STM32CubeIDE/CMake Debug, 0 errors / 0 warnings) and Program Verify **PASS**
(`Programming Finished`, `Verify Started`, `Verified OK`). The stable functional
path from the ROVMAKER decoder through USART6/PC7, Firmware, DepthSnapshot
`0x22`, USART1/DAP/COM13, and the Qt monitor is **[Hardware Verified]**.

The stable run showed `Receiving`, continuously updating plausible values,
temperature near 24 °C, refreshed sample age, increasing RX/valid-line counts,
negligible parse errors, zero overflow, and zero hard re-arm failures. A loose
or disturbed sensor-to-decoder connector caused invalid readings or a temporary
Stale state until reseating. Connector retention, strain relief, wiring
inspection, sealing as applicable, post-assembly continuity/stability, absolute
zero, installed reference point, freshwater/seawater density, installed offset,
and pool accuracy remain **[Pending]**. This observation is not classified as a
Firmware bug and does not establish production-ready connector integrity.

The verified current host path is Qt Console → Windows COM13 → DAP UART/USB
serial bridge → STM32 USART1; APC220 remains legacy/inactive. No decoder
configuration command, direct-I2C Firmware path, or calibration offset was
added during closeout.
