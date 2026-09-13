# RoboBeetle Motion / Simple Gait Design

## Status

This design is for the first desktop-verifiable Motion / Gait foundation. It
is intentionally limited to a parameterized `SimpleGaitGenerator`; it does
not implement the paper's complete CPG oscillator, Raspberry Pi / ROS2, or
closed-loop attitude control.

## Baseline and constraints

- PR #13 was verified at reviewed HEAD
  `dbce63c516b83f5f109f3802652c954836a64b4c`, merged with a normal merge
  commit, and the resulting `origin/main` is
  `adca47db966d1a2052a6aec1a08001dfb5fbf14d`.
- The existing five-servo calibration contract is immutable. Motion emits
  logical centidegrees and must not change calibration endpoints or raw PWM
  validation.
- Installed rear clearance creates a common Motion operational envelope of
  `-3000..+4500 cdeg` for RearRight and RearLeft. Their Servo calibration
  envelope remains `-4500..+4500 cdeg`.
- All profiles are `Bench Provisional / Pending Water Verification`. The
  software can prove logical direction, mirror mapping, phase relation,
  continuity, clearance limits, and stop behavior; it cannot prove water
  propulsion, turning effectiveness, or ascend/descend sign.

The audit found a cooperative `app_main_process()` loop, centralized
ServoService enable/calibration gates, a single heartbeat SafetySupervisor,
and the unused Protocol V2 message ID `0x15`. No second scheduler or wire
protocol is introduced.

## Scope

The supported logical modes are:

```c
MOTION_STOP = 0,
MOTION_FORWARD,
MOTION_BACKWARD,
MOTION_TURN_LEFT,
MOTION_TURN_RIGHT,
MOTION_ASCEND,
MOTION_DESCEND,
MOTION_COUNT
```

The runtime state is distinct from the selected mode:

```c
MOTION_STATE_STOPPED,
MOTION_STATE_RUNNING,
MOTION_STATE_STOPPING,
MOTION_STATE_FAULTED
```

`MOTION_STATE_FAULTED` is used after an immediate safety takeover. A future
explicit Start Motion may re-arm it only after the normal Servo enable and
Safety gates pass; reconnect never resumes it implicitly.

## Module boundaries

### Logical target and generator contract

`Core/Motion/joint_targets.h` defines the HAL-independent `joint_targets_t`
with five signed `int32_t` centidegree fields in semantic Servo ID order:
FrontRight, FrontLeft, FrontAxis, RearRight, RearLeft.

`Core/Motion/gait_generator.h` defines the replaceable contract. It accepts a
logical mode and amplitude/bias scales, advances by an explicit tick, and
returns `joint_targets_t`; it has no Servo, HAL, PWM, or allocation dependency.
The contract is deliberately sufficient for a future `CPGGaitGenerator` to
replace `SimpleGaitGenerator` without changing MotionManager, ServoService,
calibration, or PWM.

`Core/Motion/simple_gait_generator.c` owns the provisional profile table and
the deterministic phase oscillator:

```text
phase += 2*pi*frequency_hz*dt_seconds
theta_i = bias_i*bias_scale
        + amplitude_i*amplitude_scale*sin(phase + phase_offset_i)
```

The default table contains only concentrated profile values: 0.5 Hz,
10-degree paddle amplitude, 50% turn-side scale, and +/-10-degree FrontAxis
bias for the ascend/descend candidates. Forward uses front phase 0 and rear
phase pi; both sides use the same logical sign. Backward uses the concentrated
logical stroke inversion candidate and is documented as mathematically
phase-equivalent for a pure sine, not as a hydrodynamic conclusion.

The generator returns logical targets only. MotionManager applies the installed
rear operational envelope after profile interpolation and before ServoService
calibration. A clamp is allowed but never silent: the common Motion layer
reports it through Manager diagnostics/counters. ServoService then still
enforces the immutable absolute calibration bounds. This keeps the envelope
independent of any particular generator implementation.

### MotionManager

`Core/Motion/motion_manager.c` owns the state machine, the non-blocking
foreground scheduler, profile cross-fade, graceful stop, and immediate abort.
It advances by the wrap-safe elapsed wall time between processed calls (with a
10 ms bench cadence floor), stores no raw PWM, and never runs in an ISR.

Start behavior:

- Paddle modes require all four paddle Servo enable bits.
- Ascend/descend additionally require FrontAxis enabled.
- Start rejects a missing enable with `ServoNotEnabled`, rejects a start while
  `STOPPING` with `BUSY`, and leaves the generator phase continuous.
- ServoService records a logical angle for each known channel. Enable starts a
  newly enabled channel at neutral; SetAngle, Neutral, and Motion angle writes
  update the tracker. Raw SetPWM marks its channel pose unknown because the
  inverse pulse mapping is not part of this contract.
- A start from stopped/faulted rejects an unknown required logical pose with
  the existing `HARDWARE_FAILURE` result mapping, then acquires Motion
  ownership and cross-fades the recorded pose to the selected full gait target
  over the shared 750 ms provisional transition duration.
- A running mode change cross-fades old and new logical profiles over the same
  duration without resetting phase.

Graceful STOP behavior is exact:

1. `request_stop()` returns `OK` immediately and leaves the Servo owner as
   Motion. The timestamped protocol path records the STOP acceptance time as
   the ramp origin.
2. State becomes `MOTION_STATE_STOPPING`; the oscillator phase is held at the
   request point, so the current target vector can converge monotonically to
   zero while both amplitude and bias scale down.
3. Each foreground process linearly ramps the retained target toward all-zero
   `joint_targets_t` using its actual unsigned elapsed wall time, for 750 ms.
4. At the deadline Manager writes the all-zero logical neutral target, calls
   `servo_service_motion_end()`, and enters `MOTION_STATE_STOPPED`.

The protocol ACK for STOP is sent at step 1, not step 4. Manual Set Angle,
Set PWM, and Neutral remain `BUSY` during the entire STOPPING interval.

Immediate stop behavior is separate:

- heartbeat watchdog / host liveness loss,
- an existing SafetySupervisor fail-safe event,
- Disable All (and any explicit Servo Disable that intersects the active gait),
- an internal target-application failure

call Manager's immediate abort path. It cancels the ramp, releases the Motion
owner, marks the state faulted where appropriate, and lets Safety/Servo
disable take effect in the same foreground pass. No 750 ms wait is permitted.
LeakStatus remains monitoring-only because the current repository does not
define a leak safety trip; if Safety later exposes one, it uses this same
immediate path.

### Servo ownership

ServoService keeps the existing manual APIs and calibration values. It adds a
small explicit owner state:

- `servo_service_motion_begin(mask)` succeeds only for enabled required
  channels and an idle owner.
- `servo_service_set_angle_from_motion()` is the only gait write path; it
  validates enabled state and absolute angle calibration before using the
  existing angle-to-pulse mapping.
- manual Enable/Set PWM/Set Angle/Neutral calls return `BUSY` while Motion
  owns an actuator.
- Motion ownership is held through STOPPING and released only at final
  neutral. `servo_service_motion_abort()` is used by the immediate safety
  path. Disable All remains unconditional and fail-closed.

The dispatcher receives a MotionManager pointer so an explicit Servo Disable
can abort Manager before stopping the selected channels. Safety `app_main`
does the same before `servo_service_disable_all()`.

### Application scheduling and safety

`app_main_process()` remains the only cooperative foreground loop. It calls
`motion_manager_process(&motion_manager, HAL_GetTick())`; Manager performs at
most one elapsed-time update per foreground call when the 10 ms bench cadence
is due. There is no blocking delay, busy wait, or timer ISR gait calculation. The existing
SafetySupervisor remains the source of host-liveness truth. Its fail-safe
branch immediately aborts Motion, disables all Servos, and invalidates the
existing dispatcher action cache. Motion never changes heartbeat timeout,
leak semantics, enable ownership, or watchdog behavior.

## Protocol V2

`RBP2_MSG_SET_MOTION_MODE = 0x15` is the first unused command ID. The exact
payload is three bytes:

```text
byte 0: schema = 1
byte 1: MotionMode (0..6)
byte 2: action: 0 = STOP, 1 = START
```

Canonical validation is strict: START requires a non-STOP mode; STOP uses
`MOTION_STOP`. Invalid schema/mode/action returns `InvalidPayload`. START is
also gated by host liveness and Servo readiness. Results reuse the existing
ACK frame and sequence/duplicate cache; successful duplicate requests replay
their cached result without repeating side effects. `BUSY = 7` is appended to
the result enum without renumbering existing result codes.

The host sends one command to start/stop a gait; it does not stream targets.

## Qt Bench UI

The Console adds one small group titled `Motion / Gait — Bench` with a mode
combo containing STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, ASCEND, and
DESCEND, plus Start, Stop, and a status label. It sends only the `0x15`
command and reports `Running <mode>`, `Stopping`, `Stopped`, or `Faulted`.

While local state is Running, Stopping, or has unresolved Motion work, manual
Servo controls are disabled and the controller also rejects them locally.
Firmware remains the final guard and returns `BUSY` for races or non-Qt clients. Global Disable All is
never disabled. A successful Stop ACK changes the Qt state to Stopping; a
single-shot 750 ms UI timer changes it to Stopped, while disconnect/liveness
loss immediately clears/faults it. This UI timing is provisional and does not
claim that the actuator has been water-verified.

## Verification contract

Pure-C host tests cover deterministic generator profiles, phase and elapsed
10/20/70/100 ms stepping with uint32 wrap, common-layer rear clamps and
diagnostics, logical-pose handoff/unknown raw-PWM rejection, Motion state
transitions, start gates, graceful STOP timing/monotonic convergence, phase
continuity, ownership, Disable All, watchdog interruption, no auto-resume,
idempotent STOP, and dispatcher ACK/duplicate/invalid-mode behavior. Existing
Servo calibration, descriptor, safety, protocol, sensor, and telemetry
regressions remain in the suite.

Qt tests cover message encoding, ACK-driven Running/Stopping/Stopped state,
BUSY/manual arbitration, liveness/disconnect reset, reconnect without auto
resume, and the minimal Bench UI group. `git diff --check` and available host
builds are required. ARM Build, Program Verify, and physical desktop motion
are reported as separate evidence categories; absence of the ARM toolchain or
hardware cannot be converted into PASS.

## Explicit non-claims

The following remain `Pending Water Verification`: forward propulsion,
backward propulsion, yaw/turn effectiveness, ascend/descend sign, optimal
amplitude, frequency, phase, and hydrodynamic efficiency. No CPG equations,
coupling, adaptive amplitude, sensor feedback, Raspberry Pi, ROS2, or closed
loop attitude control is included in this version.
