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

The tool requires exactly one device path. It opens once; open failure exits.
Enter one operator-selected command per line:

| Command | Meaning |
| --- | --- |
| `link status` | Read session/link state without servicing the runtime |
| `telemetry display` | Run one runtime iteration and print received events |
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

This is a manually stepped diagnostic tool: waiting for terminal input pauses
runtime service. A valid command normally runs one runtime iteration afterward;
`link status`, help, invalid commands, and Backward do not. Use repeated
`telemetry display` commands to advance SafetyQuiet/resynchronization and observe
Active before submitting an actuator request, then to observe its eventual ACK
or uncertain outcome. One iteration may return before an ACK arrives. Operator
delays can cause heartbeat or ACK deadlines to expire; inspect the reported
outcome and do not assume an action succeeded. After loss, this CLI requires an
explicit process restart to open again. It is not a continuous service, daemon,
background reconnect worker, or terminal UI.

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

For this Windows development environment, native Linux / Pi CTest, PTY tests,
and Linux smoke compilation/execution are **NOT RUN**. Run the Linux commands
above before Pi hardware acceptance; portable build results are not substitutes.

Windows portable verification on 2026-09-17: fresh CMake/Ninja configure and
build with GCC 16.1.0 and `-Wall -Wextra -Werror` **PASS**; CTest **2/2 PASS**
(`rbp2_protocol_tests`, `rbp2_robot_codec_tests`); both direct test executables
**PASS**. Generated Windows build rules omit `robobeetle_pi_smoke` as required.

**Hardware Acceptance: PENDING USER VERIFICATION**
