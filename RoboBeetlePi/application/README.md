# Raspberry Pi Application / Actuator Integration (Slice 6)

`OnboardApplication -> LinkRuntime -> SerialSession -> PosixSerialTransport
-> /dev/serial0 -> STM32 USART2` is the application path. The portable C++17
`rbp2_robot_codec` library composes existing Protocol V2 payloads and decodes
LeakStatus, ImuSnapshot, and DepthSnapshot. Linux-only
`rbp2_onboard_application` delegates lifecycle and request ownership to the
existing runtime. No Firmware, Qt, or frozen Pi production layer is changed.

## Typed API

The facade offers `enable_servos(mask)`, `disable_servos(mask)`,
`neutral_servos(mask)`, `set_servo_angle(id, angle_cdeg)`,
`start_motion(mode)`, `stop_motion()`, and `set_gait_backend(backend)`.
`set_servo_pwm_maintenance(id, pulse_us)` is a bring-up / maintenance interface,
not the regular motion interface; the smoke CLI does not expose it.
Masks are nonzero subsets of `0x001f`; servo IDs 0..4 are FrontRight,
FrontLeft, FrontAxis, RearRight, RearLeft. Angles are signed centidegrees.
The STM32 owns calibration, angle-to-PWM conversion, gait ownership, and
STOPPED/BUSY decisions. Backward is `PendingQualification` and sends no request.

`CommandSubmitResult::Submitted` plus its sequence means admission and sequence
allocation only. Later raw LinkEvents carry `RequestAccepted`,
`RequestRejected`, `RequestOutcomeUnknown`, or `RequestCancelled`. Neither
submission nor an accepted ACK establishes physical robot state. The facade
does not maintain authoritative servo-enabled, motion, or backend state.

`run_once()` preserves RuntimeStatus and errno. Every raw LinkEvent remains in
the event stream; telemetry frames additionally produce a typed value or
`TelemetryMalformed`. Invalid telemetry does not complete requests or abort the
link. Validity flags, diagnostic counters, and sample age are exposed as wire
values without an application stale policy.

`open(device_path)` is explicit, including reopen after loss. Runtime statuses
`NeedsOpen`, `SessionLost`, and `PollFatal` are reported to the caller. There is
no automatic reconnect, request retry, or actuator replay. An uncertain command
requires a new operator/application decision. The runtime owns Heartbeat, ACK
timeout/correlation, sequence continuity, 575 ms Safety quiet, standalone raw
0x00 resynchronization, and Lost teardown. The facade never touches native fds.

## Build and manual smoke tool

On Linux / Raspberry Pi:

```sh
cmake -S RoboBeetlePi -B /tmp/robobeetle-pi-build -G Ninja \
  -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
cmake --build /tmp/robobeetle-pi-build
ctest --test-dir /tmp/robobeetle-pi-build --output-on-failure
/tmp/robobeetle-pi-build/robobeetle_pi_smoke /dev/serial0
```

The tool requires exactly one device path. It opens that path at startup; an
initial open failure exits. A later `reopen` command uses the same path and the
same application/runtime object after an explicit loss.
Enter one operator-selected command per line:

| Command | Meaning |
| --- | --- |
| `link status` | Read current session/link state |
| `reopen` | Explicitly open the original path when session is `ReopenRequired` |
| `telemetry display` | Confirm that incoming typed telemetry is displayed |
| `enable <mask>` | Submit ServoEnable |
| `disable <mask>` | Submit ServoDisable |
| `neutral <mask>` | Submit Neutral |
| `angle <servo> <cdeg>` | Submit one signed int16 angle |
| `gait simple` / `gait cpg` | Submit backend selection |
| `motion forward` / `motion turn_left` / `motion turn_right` | Submit selected motion |
| `motion ascend` / `motion descend` / `motion stop` | Submit selected motion or STOP |
| `motion backward` | Print Pending hardware qualification; send nothing |
| `help` | Print syntax |
| `quit` / `exit` / EOF | Abort the session and print its outcomes |

Masks accept decimal or `0x` hexadecimal, e.g. mask `1` selects servo 0 and
`0x001f` selects all five. Servo IDs and angles use decimal. Invalid tokens,
numeric overflow, extra arguments, and out-of-range IDs are rejected locally.
Every actuator action requires an explicit command; there is no batch sequence.

The foreground loop continuously calls `OnboardApplication::run_once()` while
the operator enters commands. A separate readiness check monitors only stdin;
reads are bounded and never wait for the remainder of a partially entered line.
The existing runtime owns all serial polling, Heartbeat, ACK deadlines, and
liveness. No background thread or duplicate scheduler is introduced. Each
complete input line triggers at most one operator command; EOF discards an
incomplete final line. Lines longer than 256 bytes are rejected.

Observe Active before submitting an actuator request, then inspect its eventual
ACK or uncertain outcome. Incoming LinkEvents and typed telemetry are printed
continuously. `link status` only reads state; normal runtime service continues
independently of that command, help, rejected arguments, or Backward. Waiting
for operator input does not pause link service. Input processing may wait for
the current runtime iteration to return. After loss the tool remains in
`ReopenRequired`; it never reconnects automatically. The operator may explicitly
enter `reopen`, which calls `open()` on the same `OnboardApplication`/
`LinkRuntime` instance and reports the resulting session/link state. Same-process
sequence continuity remains owned by `LinkRuntime`, and no uncertain actuator
request is replayed. If the session is not `ReopenRequired`, `reopen` rejects
without silently closing or reopening the active link. There is no reconnect
policy or daemonization.

Quit/EOF aborts transport ownership and outstanding requests; it does not send
Motion STOP or prove that an actuator stopped. If a STOP is required, the
operator must explicitly request it and inspect its outcome while the link is
active. Physical setup, safe mechanical conditions, and hardware decisions
remain the user's responsibility.

## Verification and evidence boundary

Portable codec tests and Protocol/LinkCore/FrameTxQueue tests run on Windows
and Linux. POSIX transport, SerialSession, LinkRuntime, OnboardApplication PTY
tests, and the smoke executable are Linux-only. Windows CMake deliberately
omits the Linux targets. PTYs verify software behavior and wire traffic, not
physical motion, UART waveform timing, underwater behavior, or hardware safety.

The Windows development environment does not run the Linux-only targets below;
portable build evidence and Raspberry Pi native evidence are recorded
separately.

Windows portable verification on 2026-09-17: fresh CMake/Ninja configure and
build with GCC 16.1.0 and `-Wall -Wextra -Werror` **PASS**; CTest **2/2 PASS**
(`rbp2_protocol_tests`, `rbp2_robot_codec_tests`); both direct test executables
**PASS**. Generated Windows build rules omit `robobeetle_pi_smoke` as required.

Final Slice 6 software gate on 2026-09-17: the Windows portable Protocol,
LinkCore, FrameTxQueue, and robot-codec build/tests **PASS**; the unchanged
Firmware host runner **PASS** with 36 executables, 13 app/backend/benchmark/
diagnostics compile-contract objects, and the USART2 source/config contract.

Raspberry Pi native verification for hardware-tested implementation SHA
`511a3715c5aa7da37775741e2756986d395a2dda`: ARM Linux build **PASS**, CTest
**6/6 PASS** (`rbp2_protocol_tests`, `rbp2_posix_serial_transport_tests`,
`rbp2_serial_session_tests`, `rbp2_link_runtime_tests`,
`rbp2_robot_codec_tests`, `rbp2_onboard_application_tests`), and smoke CLI
build/link **PASS**.

## Final Hardware Acceptance (user-reported)

Hardware-tested implementation SHA: `511a3715c5aa7da37775741e2756986d395a2dda`

### Gate A — Link and typed telemetry: PASS

- `/dev/serial0` open
- session `Online`
- link `Active`
- Leak / IMU / Depth typed telemetry path verified
- Depth typed telemetry path was verified, but this hardware environment did
  not produce a valid depth sensor sample (`flags=0`, `valid_lines=0`); this is
  not a depth sensor measurement PASS

### Gate B — Single safe servo: PASS

- Single unloaded/safe servo
- enable ACK OK
- angle ACK OK
- physical servo movement verified
- neutral ACK OK
- disable ACK OK

### Gate C — Loss, safety, and same-process reopen: PASS

- Active actuator link loss detected
- STM32 safety behavior physically verified
- Pi reached `Lost` / `NeedsOpen`
- explicit same-process reopen verified
- SafetyQuiet / resynchronization verified
- sequence continuity verified
- old actuator command was not replayed
- servo did not return automatically to the pre-loss angle

### Gate D — Gait behavior: PASS

- SimpleGait Forward → STOP physically verified
- CPG Forward → STOP physically verified
- smooth return to Neutral verified
- Backward was not tested and remains `PendingQualification`

### Gate E — Concurrent load: PASS

Concurrent Heartbeat + ACK + Leak + IMU + Depth + CPG Forward load:

- Leak frames: 334
- IMU frames: 167
- Depth frames: 167
- `TelemetryMalformed`: 0
- `DecodeError`: 0
- `OutcomeUnknown`: 0
- `RequestRejected`: 0
- `TransportWriteFailed`: 0
- `Degraded`: 0
- `Lost`: 0
- `SessionLost`: 0
- `PollFatal`: 0
- final link status: `session=Online link=Active`
- physical motion had no obvious stutter / abnormal stop

These user-reported gates record hardware acceptance for the stated setup.
They do not claim a valid depth sensor measurement where the sample was
invalid, and Backward remains pending qualification.
