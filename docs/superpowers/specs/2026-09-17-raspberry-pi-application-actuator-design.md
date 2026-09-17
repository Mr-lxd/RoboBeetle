# Raspberry Pi Slice 6 Application / Actuator Integration Design

Status: implementation baseline approved by the user on 2026-09-17.

## Goal

Add the smallest typed robot-application facade above the frozen Linux
`LinkRuntime`. The facade composes Protocol V2 command payloads, submits them
through the existing runtime, and translates received telemetry frames into
typed application events. It must not become another link state machine,
transport, scheduler, or robot-state authority.

## Frozen layering

```text
future TCP / ROS2 / Qt
          |
          v
OnboardApplication (Slice 6)
          |
          v
LinkRuntime (Slice 5)
          |
          v
SerialSession -> PosixSerialTransport -> /dev/serial0 -> STM32 USART2
          |
          v
Protocol V2 -> ServoService / MotionManager / SimpleGait / CPG / SafetySupervisor
```

`OnboardApplication` owns no native descriptor, `poll(2)` call, monotonic
clock, heartbeat, ACK deadline, retry, sequence allocator, COBS decoder,
liveness state, quiet interval, raw resynchronization, teardown, or reconnect
policy. It calls only the public `LinkRuntime::open`, `run_once`, typed-through-
codec `submit_request`, and `abort` seams. Reopen remains an explicit caller
decision, and no command is retained or replayed after loss.

## Portable robot types and command codec

`RoboBeetlePi/application/include/robobeetle/application/robot_types.hpp`
contains the C++17 enums and value structures shared by portable codec tests
and the Linux facade:

```cpp
enum class ServoId : std::uint8_t {
    FrontRight = 0, FrontLeft = 1, FrontAxis = 2,
    RearRight = 3, RearLeft = 4,
};

enum class MotionMode : std::uint8_t {
    Stop = 0, Forward = 1, Backward = 2, TurnLeft = 3,
    TurnRight = 4, Ascend = 5, Descend = 6,
};

enum class GaitBackend : std::uint8_t { SimpleGait = 0, CPG = 1 };
```

The supported servo mask is exactly `0x001f`. `robot_codec` produces only
payload bytes; it never encodes a full frame and never performs angle-to-PWM
conversion. Mask commands are two little-endian bytes. Angle and maintenance
PWM commands are `[1, servo_id, value_lo, value_hi]`, with angle interpreted
as signed int16 centidegrees and PWM as unsigned int16 microseconds. Motion
commands are `[1, mode, action]` where action is `0` for STOP and `1` for
START. Gait selection is one byte. Invalid enum values, masks, or command
combinations return a typed codec error.

`start_motion(MotionMode::Backward)` is rejected by the production facade with
`PendingQualification`; the codec still understands wire value `2` for source
parity and tests. `stop_motion()` is the only normal way to send the STOP
combination. `set_servo_pwm_maintenance` is explicitly documented as a
bring-up/maintenance path.

## Submission and application events

The facade returns admission separately from execution:

```cpp
enum class CommandSubmitStatus {
    Submitted, InvalidArgument, PendingQualification, NotActive,
    PayloadTooLarge, QueueFull, TransportRejected,
};

struct CommandSubmitResult {
    CommandSubmitStatus status{CommandSubmitStatus::NotActive};
    std::optional<std::uint16_t> sequence;
};
```

`Submitted` means that the frozen `LinkCore::submit_request` accepted and
allocated a sequence; it does not mean that an actuator executed. Later raw
`LinkEvent` values remain visible and carry `RequestAccepted`,
`RequestRejected`, `RequestOutcomeUnknown`, or `RequestCancelled`.

`ApplicationEvent` is a `std::variant` containing the unmodified underlying
`link_core::LinkEvent`, typed `LeakTelemetry`, `ImuTelemetry`, and
`DepthTelemetry` notices, and `TelemetryMalformed`. A telemetry frame keeps its
raw LinkEvent and adds one typed notice or one malformed notice. Malformed
telemetry never aborts the session, completes an ACK, or changes LinkCore
state.

`ApplicationRunResult` preserves `runtime::RuntimeStatus` and errno while
returning the translated event vector. `abort()` translates the runtime's
completion events without inventing success. The facade stores no authoritative
servo-enabled, motion, or backend state.

## Firmware-parity telemetry decoding

The codec reads the current Firmware encoders directly and uses little-endian
fixed-width fields. It does not implement stale/freshness policy.

### LeakStatus (0x20)

Payload length is one byte: `0=Unknown`, `1=Dry`, `2=Wet`. Any other value is
`TelemetryMalformed`.

### ImuSnapshot (0x21)

Payload length is 56 and schema byte zero is `1`. Byte one uses validity bits
`0x01` acc, `0x02` gyro, and `0x04` angle; other bits are malformed. The
three signed int16 little-endian triples begin at offsets 2, 8, and 14 and
represent mg, 0.1 dps, and 0.01 degree respectively. Nine uint32 little-endian
diagnostic counters occupy offsets 20, 24, 28, 32, 36, 40, 44, 48, and 52:
RX bytes, parser headers, valid frames, checksum errors, RX overflow, RX
re-arm failures, UART errors, magnetometer frames, and unsupported frames.
When a validity bit is clear, Firmware encodes that domain as all zero; the
decoder rejects non-zero bytes in an invalid domain. No additional physical
range or stale rule is added.

### DepthSnapshot (0x22)

Payload length is 38 and schema byte zero is `1`. Byte one uses `0x01` depth
valid and `0x02` temperature valid; reserved bits are malformed. Signed depth
int32 is at offset 2 (mm), signed temperature int16 at offset 6 (0.01 C), and
uint16 sample age at offset 8. When depth is invalid, Firmware requires age
`0xffff` and zeroes invalid values; the decoder enforces those wire
invariants. Seven uint32 diagnostic counters occupy offsets 10, 14, 18, 22,
26, 30, and 34. Sample age is exposed as received; no application stale
classification is performed.

## Linux facade and smoke tool

`OnboardApplication` is Linux-only because it owns a `LinkRuntime` member.
Its public methods are `open`, `run_once`, the typed Servo/Motion/Gait
commands, maintenance PWM, and `abort`. It has no fd accessor. The Linux
`robobeetle_pi_smoke` executable accepts one device path and then one explicit
line command at a time (`link status`, enable/disable/neutral, angle, gait,
motion, and `telemetry display`). It does not daemonize, reconnect, run a
batch, or automatically move actuators. `backward` prints the qualification
state and sends nothing.

## CMake and test boundary

The portable `rbp2_robot_codec` target and codec tests build on Windows and
Linux without POSIX headers. Linux-only `rbp2_onboard_application`, its PTY
integration tests, and the smoke tool link against `rbp2_link_runtime`.
Existing Protocol, LinkCore, FrameTxQueue, POSIX transport, SerialSession,
and LinkRuntime production sources remain unchanged. Firmware and
`RoboBeetleConsole` remain untouched.

Tests cover command bytes and validation, exact telemetry fixtures and all
malformed cases, admission versus ACK outcomes, PTY open/quiet/resync/heartbeat
activation, typed command traffic, telemetry/ACK interleave, loss outcomes,
explicit reopen, and the no-replay invariant. Linux tests use the existing
runtime/session seams and real PTYs; no hardware or water result is implied.

## Safety and evidence boundary

No automatic actuator replay or reconnect is present. A request that becomes
`OutcomeUnknown` is not stored. The application never claims physical actuator
state from admission or ACK alone. Build and PTY results are software evidence
only; hardware acceptance remains `PENDING USER VERIFICATION`.
