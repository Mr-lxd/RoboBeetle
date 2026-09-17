# Raspberry Pi Slice 7 Remote Control Gateway Core + TCP Adapter Design

**Status:** DESIGN FROZEN  
**Repository:** `Mr-lxd/RoboBeetle`  
**Authoritative baseline:** `main @ 9bc66130467ebb4810d49c2a509c81f838855c6e`  
**Slice:** 7  
**Design owner / reviewer:** ChatGPT  
**Implementation owner:** Codex  
**Hardware acceptance owner:** User  

This document is the authoritative Slice 7 design. It supersedes the earlier local design draft and incorporates the external architecture reviews. The behavior below is frozen for implementation unless a concrete contradiction with the actual repository is found. In that case Codex must stop and report the conflict rather than redesign the architecture.

---

## 1. Context

Slice 6 established the Raspberry Pi typed onboard application boundary:

```text
OnboardApplication
    -> LinkRuntime
    -> SerialSession
    -> PosixSerialTransport
    -> /dev/serial0
    -> STM32 USART2
```

The existing lower layers already own:

- Protocol V2 framing;
- STM32 sequence allocation;
- Heartbeat generation;
- ACK correlation and timeout handling;
- liveness tracking;
- `SafetyQuiet` / raw resynchronization / reopen lifecycle;
- transport teardown;
- no-replay semantics;
- STM32-side physical safety through loss of host heartbeat.

Slice 7 must add a remote-control boundary **above** `OnboardApplication` without moving or duplicating any of those responsibilities.

Current timing contracts are preserved:

| Contract | Value |
|---|---:|
| Protocol V2 heartbeat interval | 100 ms |
| Protocol V2 ACK timeout | 200 ms |
| Protocol V2 liveness timeout | 450 ms |
| SerialSession safety quiet | 575 ms |
| Slice 7 remote ControlHeartbeat interval | 250 ms |
| Slice 7 authority lease timeout | 1000 ms |

The 250/1000 ms remote values are engineering defaults and require later hardware qualification.

---

## 2. Collaboration and phase rules

The project workflow remains:

```text
Design -> Implementation -> Test -> Review -> Hardware Acceptance -> PR
```

Roles are fixed:

- **ChatGPT**: project lead, architecture owner, technical mentor, Reviewer.
- **Codex**: implementation engineer; implements the frozen design, writes tests, uses Git/PR, and updates documentation.
- **User**: wiring, flashing, mechanical operation, physical safety validation, and final decisions.

Slice 7 implementation must not silently change this design. If an implementation conflict appears, stop and escalate it for review.

---

## 3. Goals

Slice 7 shall provide:

1. A transport-independent `ControlGatewayCore`.
2. A Linux TCP adapter using a small binary protocol: **RBRP v1**.
3. Exactly one `OnboardApplication` / UART owner.
4. Explicit remote control acquisition.
5. A remote authority lease independent of TCP connection state.
6. Typed remote actuator commands only.
7. Strict separation between remote `request_id` and STM32 sequence.
8. Two-stage command semantics: admission first, final outcome later.
9. Bounded queues and bounded request correlation.
10. Latest-value telemetry coalescing.
11. One consistent authority-loss -> `abort()` -> STM32 safety path.
12. Explicit reconnect and no automatic replay.
13. A future ROS2 adapter seam without adding ROS2 in Slice 7.
14. Portable codec/Core tests on non-Linux hosts and real TCP/PTY integration tests on Linux.

---

## 4. Non-goals

Slice 7 does **not** implement:

- `RoboBeetleConsole` / Qt TCP client;
- Qt UI changes;
- ROS2 dependencies or packages;
- SLAM, vision, autonomy, navigation, camera, video, FOMO;
- systemd or daemonization;
- TLS or authentication;
- public-Internet deployment;
- multiple simultaneous TCP controllers;
- observer clients;
- controller arbitration;
- automatic reconnect;
- automatic command retry;
- command replay;
- Backward qualification;
- remote raw PWM;
- remote raw Protocol V2 frames;
- Firmware changes;
- Protocol V2 changes;
- LinkCore changes;
- Transport changes;
- SerialSession changes;
- LinkRuntime changes;
- Slice 1-6 queue-capacity changes.

---

## 5. Target architecture

```text
                     Windows Qt / test client
                              |
                              | TCP / RBRP v1
                              v
                 +---------------------------+
                 | Linux TCP I/O worker      |
                 |                           |
                 | listener                  |
                 | current client fd         |
                 | RBRP stream decoder       |
                 | partial TX state          |
                 | poll(listener,client,     |
                 |      owner_eventfd)       |
                 +-------------+-------------+
                               |
                  bounded inbound envelopes
                  + source-addressed signals
                               |
                               v
                 +---------------------------+
                 | Gateway owner thread      |
                 |                           |
                 | ControlGatewayCore        |
                 |         |                 |
                 | GatewayApplicationPort    |
                 +---------+-----------------+
                           |
                           | portable DTO boundary
                           v
                 LinuxOnboardApplicationPort
                           |
                    one OnboardApplication
                           |
                       LinkRuntime
                           |
                      SerialSession
                           |
                  PosixSerialTransport
                           |
                    /dev/serial0
                           |
                          STM32
```

Future ROS2 integration:

```text
TcpAdapter -------\
                   \
                    -> ControlGatewayCore -> one GatewayApplicationPort
                   /
Ros2Adapter ------/
```

TCP and ROS2 are adapters. Neither may own a second UART/application chain.

---

## 6. Frozen ownership model

### 6.1 Single application owner

Within one gateway process there is exactly:

```text
one ControlGatewayCore
one LinuxOnboardApplicationPort
one OnboardApplication
one LinkRuntime
one SerialSession
one PosixSerialTransport
one STM32 sequence authority
```

Only the **Gateway owner thread** may call:

- `open`;
- `run_once`;
- `abort`;
- typed application command methods;
- `session_state`;
- `link_state`.

The TCP worker never calls `OnboardApplication`, `LinkRuntime`, `SerialSession`, or `LinkCore`.

### 6.2 No actuator shadow state

The gateway may store communication/control metadata:

- connected source identity;
- Hello state;
- authority state;
- lease deadline;
- request correlations;
- recent request IDs;
- queue state;
- pending network output;
- latest telemetry waiting for transmission.

It must not claim authoritative robot state such as:

- servo enabled;
- servo physical angle;
- motion currently executing;
- gait currently physically active;
- actual actuator position.

An ACK is not proof that a physical actuator reached a target.

---

## 7. Portable / Linux boundary

### 7.1 Portable layer

The portable layer contains only gateway-owned types and standard-library dependencies:

```text
RoboBeetlePi/gateway/
  include/robobeetle/gateway/
    gateway_types.hpp
    gateway_application_port.hpp
    control_gateway_core.hpp
    rbrp_codec.hpp

  src/
    control_gateway_core.cpp
    rbrp_codec.cpp
```

These portable files must not include:

- `onboard_application.hpp`;
- `link_runtime.hpp`;
- `serial_session.hpp`;
- POSIX socket/poll/eventfd/termios headers;
- Qt;
- ROS2.

### 7.2 Linux layer

Linux-only implementation:

```text
RoboBeetlePi/gateway/linux/
  include/robobeetle/gateway/
    linux_onboard_application_port.hpp
    tcp_adapter.hpp
    gateway_owner.hpp

  src/
    linux_onboard_application_port.cpp
    tcp_adapter.cpp
    gateway_owner.cpp
```

The Linux application port is the **only** place where Slice 7 maps gateway DTOs to/from Slice 6 application types.

### 7.3 Future executable

```text
RoboBeetlePi/tools/
  robobeetle_pi_gateway.cpp
```

Defaults:

```yaml
device: /dev/serial0
bind: 127.0.0.1
port: 47000
```

A non-loopback bind must be explicitly requested.

When binding a non-loopback address, print:

```text
WARNING: RBRP v1 has no authentication or TLS.
Trusted engineering network only.
```

Public Internet exposure is not supported.

---

## 8. Portable gateway DTOs

Conceptual portable types:

```text
ControlSourceId : uint64_t
RequestId       : uint32_t
GatewayTimeMs   : uint64_t
```

`ControlSourceId` is process-local and never serialized.

### 8.1 RobotCommand

Typed variant:

```text
EnableServos(mask)
DisableServos(mask)
SetServoAngle(servo_id, angle_cdeg)
NeutralServos(mask)
StartMotion(mode)
StopMotion
SetGaitBackend(backend)
```

No raw Protocol V2 command and no raw PWM command exist at this boundary.

### 8.2 Application submit DTO

```text
GatewayApplicationSubmitStatus:
  Submitted
  InvalidArgument
  PendingQualification
  NotActive
  PayloadTooLarge
  QueueFull
  TransportRejected

GatewayApplicationSubmitResult:
  status
  optional<uint16_t> sequence
```

Invariant:

```text
Submitted          => sequence present
all other statuses => sequence absent
```

### 8.3 Application lifecycle DTOs

```text
GatewayApplicationRunStatus:
  Progress
  Timeout
  Interrupted
  NeedsOpen
  SessionLost
  PollFatal

GatewayApplicationSessionState:
  ReopenRequired
  SafetyQuiet
  Resynchronizing
  Online

GatewayApplicationLinkState:
  Unconfirmed
  Active
  Degraded
  Lost
```

### 8.4 Final command outcome DTO

```text
GatewayCommandOutcomeEvent:
  outcome:
    Accepted
    Rejected
    OutcomeUnknown
    Cancelled
  sequence: uint16_t
  result: uint8_t
```

**No raw Protocol V2 request type crosses the portable boundary.**

LinkCore has already performed Protocol V2 ACK correlation. The gateway does not repeat that policy.

### 8.5 Telemetry DTOs

The Linux adapter maps typed Slice 6 telemetry into neutral DTOs carrying the STM32 telemetry frame sequence plus the existing typed fields.

Malformed telemetry becomes a **local diagnostic only**.

It is not forwarded as valid telemetry and never causes authority loss.

---

## 9. GatewayApplicationPort

Portable conceptual interface:

```text
open() -> errno-style int
run_once() -> GatewayApplicationRunResult
abort() -> GatewayApplicationAbortResult

submit(RobotCommand) -> GatewayApplicationSubmitResult

session_state() -> GatewayApplicationSessionState
link_state() -> GatewayApplicationLinkState
```

The Core never sees `/dev/serial0`.

`LinuxOnboardApplicationPort` stores the configured device path and maps:

```text
GatewayApplicationPort.open()
    -> OnboardApplication::open(device_path)
```

It also exhaustively maps typed commands to current Slice 6 methods.

No new angle calibration/range policy is added.

For `SetServoAngle`, the gateway validates only:

- Servo ID is 0..4;
- the wire representation is signed `int16_t`.

Final range/calibration behavior remains below the gateway.

`Backward` remains `PendingQualification` and must generate zero UART command bytes.

---

## 10. TCP source model

Slice 7 supports:

```text
MAX_CONNECTED_TCP_SOURCES = 1
```

A second accepted TCP connection is immediately closed and is not registered in the Core.

Each accepted current connection receives a unique monotonically increasing nonzero `ControlSourceId`.

A new connection always receives a new ID.

No connection inherits:

- authority;
- lease;
- recent request IDs;
- outstanding command correlations;
- telemetry slots;
- parser state;
- partial TX state;
- output queue entries.

`ControlSourceId` remains in the Core so a future ROS2 adapter can reuse the same authority abstraction.

---

## 11. Authority state machine

States:

```text
Unowned
Acquiring
Owned
```

### 11.1 AcquireControl

Acquire is valid only when:

```text
authority == Unowned
AND source completed Hello
AND application session == ReopenRequired
```

Flow:

```text
AcquireControl
    -> authority = Acquiring
    -> explicit GatewayApplicationPort.open()
```

On `open() != 0`:

```text
authority -> Unowned
AcquireReply(OpenFailed)
no retry
```

On `open() == 0`:

```text
authority -> Owned
application normally remains SafetyQuiet
```

If authority is `Unowned` but application session is already:

```text
SafetyQuiet
Resynchronizing
Online
```

then:

```text
AcquireReply(InvalidState)
```

The Core must not silently adopt an already-open session.

### 11.2 Commands before Active

Authority may be `Owned` while application is still `SafetyQuiet`.

Commands before `LinkState::Active` go through the typed application path and retain current `NotActive` semantics.

Acquire does not manufacture `Active`.

### 11.3 ReleaseControl

Release is a safety boundary, not a motion command.

```text
ReleaseControl
    -> revoke authority
    -> GatewayApplicationPort.abort()
    -> consume terminal command outcomes
    -> authority = Unowned
```

Do not synthesize:

- Stop;
- Neutral;
- Disable;
- PWM.

A still-connected client may later acquire again only after the application is back in `ReopenRequired`.

---

## 12. Remote authority lease

### 12.1 Defaults

```text
ControlHeartbeat period: 250 ms
Authority lease timeout: 1000 ms
```

The remote lease is separate from STM32/Pi liveness.

### 12.2 Trusted receive timestamp

The TCP worker captures:

```text
received_at_ms
```

when a complete valid RBRP frame has been decoded.

It uses the process-local monotonic/steady clock.

This timestamp:

- is not on the wire;
- is not client-supplied;
- cannot be forged by the client.

Each inbound request is carried as:

```text
RemoteEnvelope:
  ControlSourceId source
  GatewayTimeMs received_at_ms
  RemoteMessage message
```

The worker never modifies authority lease state.

### 12.3 Initial lease anchor

The initial lease does **not** start at AcquireControl receive time.

After successful explicit open:

```text
commit authority = Owned
granted_at_ms = owner-thread monotonic now
lease_deadline = granted_at_ms + 1000 ms
```

Reason: no authority exists until acquisition has actually succeeded.

### 12.4 Heartbeat refresh

Only a valid `ControlHeartbeat` from the current owner refreshes the lease.

A heartbeat is timely iff:

```text
received_at_ms < current_lease_deadline
```

When timely:

```text
new_deadline = received_at_ms + 1000 ms
```

Owner processing time is not used.

A heartbeat received before the deadline remains timely even if processed later.

A heartbeat received at or after the deadline is late and cannot revive old authority.

Commands, Hello messages, arbitrary TCP traffic, and a connected socket do not refresh the lease.

---

## 13. Owner loop and scheduling

Current `LinkRuntime::run_once()` owns the serial `poll()`, so Slice 7 does not expose the serial fd or build a unified TCP/serial poll loop.

The frozen owner loop is:

```text
1. process all pending SourceLost control signals
2. drain all RemoteEnvelopes currently queued
3. evaluate authority lease
4. call GatewayApplicationPort::run_once() exactly once when appropriate
5. process all pending SourceLost control signals again
6. drain all RemoteEnvelopes currently queued again
7. evaluate authority lease again
8. publish outputs / wake TCP worker
```

The inbound queue is already hard-bounded, so there is no separate “8 messages only” budget.

This prevents a timely heartbeat from being hidden behind an artificial message-count limit.

If inbound capacity is exhausted:

```text
SourceLost
-> close source
-> revoke authority
-> abort application
```

No inbound command is silently dropped.

When the application is `NeedsOpen`, the owner may wait on the inbound bridge condition variable with a bounded wake interval rather than busy-spin.

---

## 14. Cross-thread bridge

### 14.1 Worker -> Owner

The worker owns TCP I/O.

It sends ordinary messages through a bounded inbound queue:

```text
RemoteEnvelope queue
max complete messages: 32
max queued payload bytes: 16 KiB
```

It also has a separate high-priority lifecycle-control channel:

```text
SourceLostSignal:
  ControlSourceId source
  SourceLostReason reason
```

This control channel is never shared with the ordinary data queue.

If necessary, the worker stops accepting/registering new sources rather than losing a lifecycle-control signal.

The worker signals an owner-side condition variable when inbound work or source-loss work becomes available.

### 14.2 Owner -> Worker

All network output is internally source-addressed:

```text
GatewayOutbound:
  ControlSourceId source
  GatewayMessage message
```

This includes:

- HelloReply;
- AcquireReply;
- ControlState;
- CommandSubmitted;
- CommandOutcome;
- ServiceError;
- LeakTelemetry;
- ImuTelemetry;
- DepthTelemetry.

The worker may transmit an item only when:

```text
outbound.source == current_source_id
```

Otherwise the item is stale and is dropped.

Owner-side source closure uses a separate control channel:

```text
CloseSourceSignal:
  ControlSourceId source
  CloseSourceReason reason
```

A worker closes the current client because of this signal only when:

```text
signal.source == current_source_id
```

A stale CloseSource for an old source is ignored/logged.

### 14.3 Owner -> Worker wakeup

The TCP worker poll set contains:

```text
listener fd
current client fd (when present)
owner-to-worker eventfd
```

Owner writes to `eventfd` when:

- critical output becomes available;
- telemetry becomes sendable;
- a CloseSource request is posted;
- shutdown is requested.

`eventfd` is only a wakeup mechanism. It is not the lifecycle signal itself.

---

## 15. Source-generation isolation

Source identity is a safety boundary.

Every cross-thread lifecycle or output object carries the intended `ControlSourceId`.

This prevents ABA/stale-source races such as:

```text
A disconnects
B connects
late A SourceLost arrives
```

The late A signal must never revoke B.

Likewise:

```text
A pending command
A disconnects
B connects
owner later generates A OutcomeUnknown/Cancelled
```

Those outputs are tagged with A's `ControlSourceId` and must never be transmitted to B.

On connection close the TCP worker destroys:

- parser state;
- partial RX state;
- partial TX state;
- unsent source-local telemetry state;
- source-local socket state.

Old-source network bytes are never continued on a new socket.

---

## 16. RBRP v1 framing

RBRP v1 is a small versioned binary framing protocol over TCP.

### 16.1 Header

Exactly 16 bytes:

| Offset | Field | Width | Encoding |
|---:|---|---:|---|
| 0..3 | magic | 4 | ASCII `RBRP` |
| 4 | version | 1 | `1` |
| 5 | kind | 1 | message kind |
| 6..7 | flags | 2 | uint16 LE, must be 0 |
| 8..11 | payload_len | 4 | uint32 LE, 0..512 |
| 12..15 | request_id | 4 | uint32 LE |

No C/C++ struct-layout serialization is allowed.

All integer encoding/decoding is explicit little-endian.

There is:

- no COBS;
- no Protocol V2 CRC duplication;
- no additional RBRP checksum.

TCP already supplies an ordered reliable byte stream; RBRP supplies message boundaries and application semantics.

### 16.2 Streaming decoder

Decoder must handle:

- fragmented header;
- fragmented payload;
- several frames in one `recv`;
- a frame ending exactly on the input boundary.

It must never allocate an unbounded buffer from a peer-supplied length.

Maximum payload:

```text
512 bytes
```

---

## 17. RBRP v1 message kinds

### Client -> Pi

| Kind | Name |
|---:|---|
| `0x01` | Hello |
| `0x02` | AcquireControl |
| `0x03` | ControlHeartbeat |
| `0x04` | ReleaseControl |
| `0x05` | CommandRequest |

### Pi -> Client

| Kind | Name |
|---:|---|
| `0x81` | HelloReply |
| `0x82` | AcquireReply |
| `0x83` | ControlState |
| `0x84` | CommandSubmitted |
| `0x85` | CommandOutcome |
| `0x86` | LeakTelemetry |
| `0x87` | ImuTelemetry |
| `0x88` | DepthTelemetry |
| `0x89` | ServiceError |

There is no RBRP ACK message.

`CommandSubmitted` means admission only.

`CommandOutcome` is the final STM32-correlated result.

---

## 18. Request ID rules

- Every client request uses a nonzero `request_id`.
- `request_id=0` is reserved for unsolicited Pi->client state/telemetry.
- Direct replies echo the triggering request ID.
- A command outcome uses the original CommandRequest request ID.
- Request IDs are scoped to one `ControlSourceId`.
- A request ID cannot be reused while outstanding.
- The source keeps a bounded recent-completed set of 64 IDs and rejects recent reuse.
- A new TCP source starts a completely new request-ID namespace.

For ordinary request/reply messages, an ID becomes completed once its direct reply is produced.

For a successfully submitted command, the ID remains outstanding until final `CommandOutcome`.

A locally rejected command completes after `CommandSubmitted` reports the rejection status.

Duplicate IDs must never re-execute a command.

---

## 19. Exact client payloads

### 19.1 Hello (`0x01`)

Payload exactly 2 bytes:

```text
client_capabilities: uint16 LE
```

v1 requires:

```text
client_capabilities == 0
```

Hello must be the first accepted request on a source.

A second Hello is a nonfatal semantic error.

### 19.2 AcquireControl (`0x02`)

Payload length:

```text
0
```

Requires Hello first.

### 19.3 ControlHeartbeat (`0x03`)

Payload length:

```text
0
```

Only the current owner can refresh the lease.

### 19.4 ReleaseControl (`0x04`)

Payload length:

```text
0
```

Only current owner may release.

### 19.5 CommandRequest (`0x05`)

First byte:

```text
command_kind: uint8
```

| Command | Value | Remaining payload | Total |
|---|---:|---|---:|
| EnableServos | `0x01` | `mask:uint16 LE` | 3 |
| DisableServos | `0x02` | `mask:uint16 LE` | 3 |
| SetServoAngle | `0x03` | `servo_id:uint8, angle_cdeg:int16 LE` | 4 |
| NeutralServos | `0x04` | `mask:uint16 LE` | 3 |
| StartMotion | `0x05` | `mode:uint8` | 2 |
| StopMotion | `0x06` | none | 1 |
| SetGaitBackend | `0x07` | `backend:uint8` | 2 |

Servo masks:

```text
nonzero subset of 0x001F
```

Servo IDs:

```text
0..4
```

Motion:

```text
1 Forward
2 Backward
3 TurnLeft
4 TurnRight
5 Ascend
6 Descend
```

Backward remains `PendingQualification`.

Gait backend:

```text
0 SimpleGait
1 CPG
```

There is no remote raw PWM command.

---

## 20. Server payloads

### 20.1 HelloReply (`0x81`)

Exactly 8 bytes:

```text
server_capabilities             uint16 LE = 0
max_payload                     uint16 LE = 512
control_heartbeat_interval_ms   uint16 LE = 250
authority_lease_timeout_ms      uint16 LE = 1000
```

### 20.2 AcquireReply (`0x82`)

Exactly 12 bytes:

```text
result              uint8
authority_state     uint8
session_state       uint8
link_state          uint8
lease_timeout_ms    uint32 LE
detail              uint32 LE
```

AcquireResult:

```text
0 Granted
1 RequiresHello
2 AlreadyOwnedByOther
3 AlreadyOwnedBySource
4 OpenFailed
5 InvalidState
```

`Granted` requires successful explicit open.

`OpenFailed.detail` may contain errno.

### 20.3 ControlState (`0x83`)

Exactly 8 bytes:

```text
authority_state     uint8
session_state       uint8
link_state          uint8
reason              uint8
lease_remaining_ms  uint32 LE
```

AuthorityState:

```text
0 Unowned
1 Acquiring
2 Owned
```

SessionState:

```text
0 ReopenRequired
1 SafetyQuiet
2 Resynchronizing
3 Online
```

LinkState:

```text
0 Unconfirmed
1 Active
2 Degraded
3 Lost
```

StateReason:

```text
0 None
1 Acquired
2 Released
3 LeaseExpired
4 SourceDisconnected
5 RemoteProtocolViolation
6 CriticalTxFailure
7 LinkLost
8 OpenFailed
9 Shutdown
```

### 20.4 CommandSubmitted (`0x84`)

Exactly 4 bytes:

```text
status             uint8
sequence_present   uint8
stm32_sequence     uint16 LE
```

Status:

```text
0 Submitted
1 InvalidArgument
2 PendingQualification
3 NotActive
4 PayloadTooLarge
5 QueueFull
6 TransportRejected
```

Invariant:

```text
Submitted => sequence_present=1
otherwise => sequence_present=0 and stm32_sequence=0
```

### 20.5 CommandOutcome (`0x85`)

Exactly 5 bytes:

```text
outcome         uint8
command_kind    uint8
stm32_sequence  uint16 LE
result_code     uint8
```

Outcome:

```text
0 Accepted
1 Rejected
2 OutcomeUnknown
3 Cancelled
```

`result_code`:

- Accepted -> 0
- OutcomeUnknown -> 0
- Cancelled -> 0
- Rejected -> current STM32 AckResult byte

### 20.6 ServiceError (`0x89`)

Exactly 8 bytes:

```text
error_code     uint16 LE
related_kind   uint8
reserved       uint8 = 0
detail         uint32 LE
```

Active v1 semantic errors:

```text
0 None
1 NotHello
2 AlreadyHello
3 InvalidRequestId
4 DuplicateRequestId
5 NotAuthority
6 AuthorityBusy
7 InvalidMessagePayload
8 UnsupportedCommand
9..15 Reserved
```

Fatal framing errors never use ServiceError.

Critical queue exhaustion never uses ServiceError.

Malformed STM32 telemetry never uses ServiceError.

---

## 21. Fatal framing versus semantic errors

Immediately close the source, with no ServiceError, for:

- bad magic;
- unsupported version;
- nonzero flags;
- `payload_len > 512`;
- unknown top-level kind;
- known top-level kind with impossible exact payload length.

If that source owns authority:

```text
SourceLost
-> revoke
-> abort
-> existing STM32 safety path
```

Well-framed semantic errors may keep the connection alive, for example:

- request ID zero;
- duplicate request ID;
- request before Hello;
- non-authority command;
- unsupported nested command kind;
- unsupported Hello capability.

Invalid typed command parameters use `CommandSubmitted(InvalidArgument)` when the top-level CommandRequest itself is well-framed.

---

## 22. Two-stage command semantics

Example:

```text
Remote:
request_id = 42
SetServoAngle(...)

        |
        v

ControlGatewayCore
        |
        v

GatewayApplicationPort.submit(...)
        |
        v

Submitted
stm32_sequence = 813
```

Immediate remote reply:

```text
CommandSubmitted
request_id = 42
status = Submitted
stm32_sequence = 813
```

This is **not physical success**.

Correlation map:

```text
813 ->
  source = current ControlSourceId
  request_id = 42
  command_kind = SetServoAngle
```

Later neutral application outcome:

```text
sequence = 813
Accepted / Rejected / OutcomeUnknown / Cancelled
```

Core looks up by live STM32 sequence and emits:

```text
CommandOutcome
request_id = 42
stm32_sequence = 813
...
```

No raw request type is required in the portable Core.

An outcome with no live sequence correlation is a bounded local diagnostic and is never assigned to a new request or new source.

---

## 23. Telemetry

Telemetry messages are unsolicited:

```text
request_id = 0
```

They are sent only to the current TCP authority holder.

### 23.1 LeakTelemetry (`0x86`)

Exactly 3 bytes:

```text
stm32_sequence uint16 LE
state          uint8
```

State:

```text
0 Unknown
1 Dry
2 Wet
```

### 23.2 ImuTelemetry (`0x87`)

Exactly 58 bytes:

```text
stm32_sequence                uint16 LE
schema_version                uint8
validity_flags                uint8
acc_mg[3]                     3 * int16 LE
gyro_tenth_dps[3]             3 * int16 LE
angle_centidegrees[3]         3 * int16 LE
rx_byte_count                 uint32 LE
header_count                  uint32 LE
valid_frame_count             uint32 LE
checksum_error_count          uint32 LE
rx_buffer_overflow_count      uint32 LE
rx_rearm_failure_count        uint32 LE
uart_error_count              uint32 LE
mag_frame_count               uint32 LE
unsupported_frame_count       uint32 LE
```

### 23.3 DepthTelemetry (`0x88`)

Exactly 40 bytes:

```text
stm32_sequence                uint16 LE
schema_version                uint8
validity_flags                uint8
depth_mm                      int32 LE
temperature_centi_c           int16 LE
sample_age_ms                 uint16 LE
rx_byte_count                 uint32 LE
valid_line_count              uint32 LE
parse_error_count             uint32 LE
overlong_line_count           uint32 LE
rx_buffer_overflow_count      uint32 LE
hard_rearm_failure_count      uint32 LE
uart_error_count              uint32 LE
```

The gateway forwards the already-decoded typed value. It does not invent a freshness or physical-range policy.

Malformed telemetry:

```text
local log/counter only
no valid telemetry frame
no ServiceError
no authority change
no abort
no command completion
```

---

## 24. Network backpressure

### 24.1 Bounds

Per current source:

| Resource | Bound |
|---|---:|
| Complete inbound messages | 32 |
| Inbound payload bytes | 16 KiB |
| Critical outbound frames | 32 |
| Critical outbound bytes | 32 KiB |
| Latest telemetry slots | 3 |
| Outstanding command correlations | 32 |
| Recent request IDs | 64 |
| Connected TCP sources | 1 |

These are Slice 7 network-layer bounds and do not modify Slice 1-6 capacities.

### 24.2 Critical outbound

Critical output:

- HelloReply;
- AcquireReply;
- ControlState;
- CommandSubmitted;
- CommandOutcome;
- ServiceError.

Critical items are FIFO and never silently dropped.

If a new critical item cannot be enqueued:

```text
revoke authority
-> abort application
-> post source-addressed CloseSource
-> wake TCP worker
-> worker closes matching source
```

No success response is fabricated.

### 24.3 Telemetry coalescing

There are three latest-value slots:

```text
Leak
IMU
Depth
```

If a telemetry frame has not begun transmission, a newer value of the same type may replace it.

Once any byte of a serialized telemetry frame has been written to TCP:

```text
frame = in-flight
```

That frame must:

- finish completely; or
- be abandoned only because the socket is being closed/fails.

A partially transmitted frame is never replaced by a newer telemetry frame.

Critical output is prioritized over starting a new telemetry frame.

---

## 25. Failure and physical-safety semantics

Every authority-loss cause uses one path:

```text
authority lost
    -> ControlGatewayCore revokes authority
    -> GatewayApplicationPort.abort()
    -> LinuxOnboardApplicationPort
    -> OnboardApplication.abort()
    -> LinkRuntime / SerialSession teardown
    -> Pi Protocol V2 heartbeat stops
    -> STM32 host-liveness timeout
    -> STM32 SafetySupervisor
    -> physical safe behavior
```

The gateway does not synthesize Stop/Neutral/Disable/PWM as a guessed safety sequence.

Authority loss includes:

- ReleaseControl;
- TCP EOF/disconnect;
- remote lease timeout;
- fatal RBRP framing violation;
- inbound queue exhaustion;
- critical TX failure/exhaustion;
- STM32/UART session loss;
- normal process shutdown.

If the source is not the current owner, source loss must not abort another source.

---

## 26. UART/session loss while TCP remains connected

The owner monitors:

- application run status;
- neutral link/session events;
- `session_state`;
- `link_state`.

The following revoke authority:

```text
SessionLost
PollFatal
unexpected ReopenRequired while Owned
LinkState::Lost
```

If TCP is still usable, best-effort source-addressed `ControlState(LinkLost)` may be queued before closure policy completes, but safety teardown does not wait for network delivery.

No automatic UART reopen occurs.

A fresh remote Acquire is required later.

---

## 27. Reconnect and no replay

Mandatory reconnect:

```text
new TCP socket
    -> new ControlSourceId
    -> Hello
    -> explicit AcquireControl
    -> application must be ReopenRequired
    -> explicit open()
    -> SafetyQuiet
    -> raw resynchronization
    -> Protocol V2 Heartbeat
    -> Link Active
    -> fresh CommandRequest
```

Reconnect is never resume.

A new source does not inherit:

- old authority;
- lease;
- request IDs;
- correlations;
- output;
- telemetry;
- partial writes;
- old commands.

`OutcomeUnknown` is never silently converted into success.

Old commands are never automatically resent.

Same-process reopen continues to use the same `OnboardApplication`/`LinkRuntime` object, preserving lower-layer sequence continuity.

---

## 28. Normal gateway shutdown

For normal CLI exit / SIGINT / SIGTERM:

```text
1. stop accepting new connections and new remote commands
2. if authority is Owned or application session is open:
     GatewayApplicationPort.abort()
3. revoke/clear authority
4. post source-addressed CloseSource if a client exists
5. signal owner-to-worker eventfd
6. TCP worker closes current client and listener
7. join TCP worker
8. destroy Core / application objects
```

Do not synthesize actuator commands.

Normal shutdown uses the same `abort()` / STM32 safety ownership model.

`SIGKILL`, power loss, or hard process death cannot execute graceful shutdown. Physical safety in those cases relies on existing STM32 heartbeat/liveness behavior.

---

## 29. ROS2 future integration

ROS2 is explicitly outside Slice 7, but Slice 7 must not block it.

Future:

```text
TcpAdapter -------\
                   \
                    -> ControlGatewayCore -> one application owner
                   /
Ros2Adapter ------/
```

The ROS2 adapter must:

- obtain a unique `ControlSourceId`;
- obey the same authority manager;
- obey the same lease contract;
- never own `/dev/serial0`;
- never call LinkRuntime directly;
- never build a second safety state machine.

Suggested future ROS2 mapping:

### Topics

- Leak telemetry;
- IMU telemetry;
- Depth telemetry;
- link state;
- control state;
- optional asynchronous low-level command/result stream.

### Services

- AcquireControl;
- ReleaseControl;
- status/query/configuration.

### Actions

Long-running behavior such as:

- DiveToDepth;
- HoldDepth;
- ExecuteTrajectory;
- FollowTarget.

DDS/RMW liveliness may be an auxiliary source-loss observation, but the gateway authority lease remains the project safety contract.

Safety must not depend on one DDS/RMW vendor.

High-frequency autonomy/vision feedback loops should run onboard the Pi/ROS2 side rather than sending high-rate actuator commands from Windows Qt.

---

## 30. Software test strategy

Three test layers are mandatory.

### 30.1 Portable RBRP codec tests

Verify exact bytes for:

- header encode/decode;
- fragmented header;
- fragmented payload;
- several frames in one input buffer;
- bad magic;
- unsupported version;
- nonzero flags;
- payload >512 without oversized allocation;
- unknown top-level kind;
- exact payload lengths;
- fatal framing closes without ServiceError;
- request ID zero;
- every message schema;
- every typed command;
- signed `int16` angle;
- Backward gate;
- exact telemetry lengths;
- enum/status encoding.

### 30.2 Portable ControlGatewayCore tests

Use `FakeGatewayApplicationPort`.

No POSIX or Slice 6 Linux classes in these tests.

Cover:

- Hello lifecycle;
- successful Acquire/open;
- Acquire InvalidState;
- initial lease anchored at grant-commit time;
- heartbeat refresh anchored at trusted receive time;
- Release -> exactly-once abort;
- lease expiry;
- timely heartbeat processed late;
- heartbeat actually received late;
- >8 ordinary envelopes followed by a timely heartbeat does not false-expire;
- near-capacity queue plus timely heartbeat;
- queue exhaustion causes SourceLost;
- authority ownership;
- duplicate request IDs;
- local submit rejections;
- sequence-only final-outcome correlation;
- Submitted -> Accepted;
- Submitted -> Rejected;
- OutcomeUnknown;
- Cancelled;
- missing/stale outcome -> local diagnostic only;
- UART/session loss;
- Backward PendingQualification;
- no replay;
- critical-output failure -> revoke + abort + CloseSource;
- stale SourceLost for old source cannot affect a newer source;
- no authoritative actuator shadow state.

### 30.3 Linux TCP + real application-port + PTY integration

Use:

```text
loopback TCP
+
Linux TCP worker
+
Gateway owner
+
real LinuxOnboardApplicationPort
+
real OnboardApplication
+
PTY STM32 simulator
```

Verify:

- Hello/HelloReply;
- Acquire;
- SafetyQuiet -> Resynchronizing -> Active;
- exactly one UART owner;
- second TCP connection immediately closed;
- eventfd wakeup;
- source-addressed SourceLost/CloseSource;
- request_id -> STM32 sequence;
- admission separate from final ACK outcome;
- Accepted/Rejected;
- telemetry forwarding only to authority;
- telemetry coalescing;
- partially transmitted telemetry not replaced;
- fatal framing close with no ServiceError;
- malformed telemetry local-only;
- disconnect -> abort;
- lease timeout -> abort;
- pending -> OutcomeUnknown;
- queued -> Cancelled;
- same-process reconnect;
- STM32 sequence continuity;
- no replay;
- UART loss while TCP stays connected;
- stale CloseSource cannot close a new source;
- A disconnect / B reconnect / late A abort outcomes -> B receives zero A messages;
- old-source partial TX and telemetry destroyed on close;
- critical backpressure;
- bounded inbound flood behavior;
- Backward -> zero UART command bytes;
- normal shutdown ordering.

Long-running tests must exercise 16-bit STM32 sequence wrap/correlation behavior without changing LinkCore ownership.

---

## 31. Hardware acceptance gates

Hardware validation is performed by the user.

### Gate A — TCP + telemetry

Verify:

```text
Laptop/test client
-> TCP connect
-> Hello
-> AcquireControl
-> SafetyQuiet/resync
-> Link Active
-> typed Leak/IMU/Depth telemetry
```

Invalid Depth sensor data must remain distinct from a valid measurement claim.

### Gate B — one safe unloaded servo

Perform remotely:

```text
EnableServos
-> SetServoAngle
-> NeutralServos
-> DisableServos
```

Verify both protocol outcome and physical behavior.

### Gate C — physical network break safety

With actuator active:

```text
physically break Laptop <-> Pi network
```

Verify:

```text
remote lease expires
-> Pi revokes authority
-> Pi aborts application
-> STM32 heartbeat stops
-> STM32 Safety physically acts
```

Do not replace this with a clean TCP close test.

### Gate D — reconnect without replay

Same Pi gateway process:

```text
network restored
-> new TCP source
-> Hello
-> explicit Acquire
-> Active
```

Verify:

- STM32 sequence continuity;
- no old command replay;
- servo does not automatically resume old angle.

### Gate E — combined load

Run:

```text
ControlHeartbeat
+ command ACK traffic
+ Leak
+ IMU
+ Depth
+ CPG Forward
```

Record:

- no false authority lease timeout;
- no unexpected Degraded/Lost;
- no malformed remote telemetry;
- no critical queue overflow;
- queue high-water marks;
- telemetry coalescing/drops;
- no obvious gait cadence regression.

---

## 32. Empirical qualification items

These are not architecture choices and do not authorize Codex to redesign the system:

1. 32-frame / 32 KiB critical output sizing under realistic load.
2. 32-message / 16 KiB inbound sizing under realistic load.
3. 250 ms heartbeat / 1000 ms lease margin on the Pi under load.
4. Long-running behavior across STM32 16-bit sequence wrap.
5. Physical network-loss-to-safe-state timing.

If measurements show a problem, return to architecture review before changing semantics.

---

## 33. Frozen invariants checklist

Implementation is conforming only if all remain true:

```text
[ ] exactly one OnboardApplication/UART owner
[ ] ControlGatewayCore is portable
[ ] portable Core does not include Slice 6 Linux-only headers
[ ] TCP worker never calls application/runtime
[ ] TCP request_id != STM32 sequence
[ ] client never chooses STM32 sequence
[ ] Submitted != physical success
[ ] only final CommandOutcome completes a submitted command
[ ] no automatic reconnect
[ ] no command retry/replay
[ ] Backward remains PendingQualification
[ ] raw PWM not exposed remotely
[ ] raw Protocol V2 not tunneled remotely
[ ] one TCP source maximum
[ ] every cross-thread lifecycle/output item is source-addressed
[ ] stale source signals/outputs cannot affect a new source
[ ] authority lease is owned only by ControlGatewayCore
[ ] initial lease begins at successful grant commit
[ ] heartbeat refresh uses trusted receive_at timestamp
[ ] malformed telemetry cannot trigger control safety
[ ] queue exhaustion fails safe
[ ] authority loss always goes through abort -> STM32 safety
[ ] no inferred Stop/Neutral/Disable safety sequence
[ ] Qt is untouched in Slice 7
[ ] Firmware/Protocol/LinkCore/Transport/Session/Runtime are untouched
[ ] ROS2 future adapter reuses the same Core and application owner
```

---

## 34. Phase boundary

With this document approved:

```text
Slice 7 DESIGN = FROZEN
```

The next phase is **Implementation Plan**, authored by ChatGPT and executed by Codex.

Codex must not change the frozen architecture while producing the plan or implementation.

No production code, tests, CMake, Qt, Firmware, or frozen lower-layer changes are part of the design phase itself.
