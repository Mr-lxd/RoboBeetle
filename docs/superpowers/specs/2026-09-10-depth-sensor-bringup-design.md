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
T=<signed-decimal-with-two-fractional-digits>D=<signed-decimal-with-two-fractional-digits>\\r\\n
```

The first is the decoder-board manual's canonical output format. The second is
the exact compact `T=...D=...` form recorded in the vendor material. The
manual's isolated `Temp=` example is treated as an internal documentation
inconsistency and is not generalized into a third grammar. No whitespace,
substring, arbitrary separator, bare-LF, or trailing-data variant is accepted.

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
most one optional telemetry frame. Leak, IMU, and Depth are selected by one
pure-C fair cursor. Preference order is:

| Last successful slot | Preference order |
|---|---|
| none | Leak, IMU, Depth |
| Leak | IMU, Depth, Leak |
| IMU | Depth, Leak, IMU |
| Depth | Leak, IMU, Depth |

The selector scans the preference order for a due policy. A failed send does
not advance the cursor or mark the policy published. With Depth not due, the
existing Leak/IMU behavior remains equivalent; with all three due, the cursor
prevents starvation without emitting a burst.

## Qt monitor

Qt adds `DepthSnapshot` decoding and a read-only `DepthMonitor`, routed before
ACK processing just like the existing telemetry monitors. It validates exact
payload length, schema, reserved flags, signed fields, and validity/zero rules.

The panel displays `Unknown`, `Receiving`, `Stale`, or `Error`, depth in metres,
temperature in degrees Celsius, sample age, and the frozen diagnostics. Invalid
fields display `--`; stale, disconnected, and liveness-lost states clear old
measurement values. The panel adds no plot, command, calibration, or control
action.

## Verification boundary

Each implementation slice has a test-first red/green checkpoint. The final
gates discover and run every existing Firmware regression, all Console CTests,
new depth tests, direct-standard-header checks, `.ioc`/generated-HAL syntax
checks, and `git diff --check`. No hardware is programmed by Codex.

Final labels are kept separate:

```text
Host Test: evidence from host tests
ARM Build: only PASS with an actual ARM toolchain/build
Program Verify: Pending until the user programs and checks the board
Hardware Verified: Pending until the user supplies real depth hardware evidence
```

Electrical signal level, surface-zero procedure, actual decoder cadence,
seawater density selection, and body-frame mapping are explicitly documented
as hardware/future verification items.
