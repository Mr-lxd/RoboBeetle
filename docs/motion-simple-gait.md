# Motion / Gait — Bench-Provisional STOP Contract

This document records the Motion / Gait control foundation carried into
`feature/cpg-gait-core`. It is a host-test/software contract. The completed
SimpleGait mechanical baseline has Hardware Verified Front/Rear physical
anti-phase; the latest CPG desktop physical gait is also Hardware Verified for
the recorded desktop checks. ARM build/program performance evidence, true
water propulsion, and hydrodynamic effectiveness remain separate pending
categories.

The normal Firmware default is `MOTION_DEFAULT_GAIT_BACKEND_CPG=1`. An
explicit `MOTION_DEFAULT_GAIT_BACKEND_CPG=0` build retains SimpleGait as the
mechanical baseline. Each build registers exactly one generator with
MotionManager.

The host-only comparison tool is
`RoboBeetleFirmware/tests/tools/gait_trace_compare.c`; it produces deterministic
SimpleGait/CPG integer-target CSVs and a separate CPG internal-state CSV. The
trace is offline evidence only and adds no Protocol V2 telemetry or Qt runtime
backend selector.

### Host-only SimpleGait versus CPG trace

The trace runner samples both production generators at 10 ms on one fixed
timeline: `[0,5000)` `FORWARD`, `[5000,8000)` `TURN_LEFT`, `[8000,11000)`
`FORWARD`, `[11000,14000)` `TURN_RIGHT`, `[14000,17000)` `FORWARD`, followed
by one final `STOP` row at `17000 ms`. It advances each generator by exactly
10 ms before every row after the initialized `0 ms` sample and uses amplitude
and bias scales of `1.0F`.

`gait_trace_simple.csv` and `gait_trace_cpg.csv` each contain 1701 rows with
the canonical integer logical-target fields
`time_ms,backend,mode,front_right_cdeg,front_left_cdeg,rear_right_cdeg,`
`rear_left_cdeg,front_axis_cdeg`, followed by the host-only
`guarded_*_cdeg` projection. The raw fields are the generator requests; the
projection applies the existing Motion common guards, front `-4500..+2800`
and rear `-3000..+4500 cdeg`, while leaving `FrontAxis` unchanged. This is an
algorithm-request versus installed-safe-command comparison aid, not a
replacement for MotionManager's production sanitizer and not a PWM trace.

`gait_trace_cpg_internal.csv` contains the same timeline with CPG phase,
amplitude, target amplitude, `theta_dot0..theta_dot3`, and `raw_output0..3`
written at double precision for host-only research. These states are not
Protocol V2 telemetry. The deterministic regression compares all three files
byte-for-byte across two generated directories and validates the timeline,
mode boundaries, final STOP zeros, headers, row counts, and guard bounds.

## Wire contract

`SetMotionMode` is Protocol V2 message `0x15` with exactly three payload bytes:

```text
schema  uint8 = 1
mode    uint8
action  uint8
```

The stable mode order is:

```text
0 STOP       1 FORWARD     2 BACKWARD     3 TURN_LEFT
4 TURN_RIGHT 5 ASCEND      6 DESCEND     7 COUNT (sentinel)
```

`STOP` uses mode 0 and action `STOP`. The selected CPG generator, and the
explicit SimpleGait diagnostic generator, accept `FORWARD`, `TURN_LEFT`,
`TURN_RIGHT`, `ASCEND`, and `DESCEND` for
`START`. `BACKWARD` remains in the enum and wire schema for compatibility, but
is reserved pending bench/water verification; the Firmware generator rejects it
and the Qt Console does not emit it. A successful STOP ACK means that the stop
request was accepted and the Firmware has entered `MOTION_STOPPING`; it does
not mean that all targets are already neutral. The existing successful-request
duplicate cache applies to `0x15`, so a retry of the same sequence/type replays
the ACK without repeating the state transition.

ACK result `BUSY=7` is appended after the existing frozen result values 0–6;
the existing numeric meanings are unchanged.

## Firmware state and ownership

`MotionManager` owns the enabled Servo channels through these states:

```text
STOPPED ── START accepted ──> RUNNING
RUNNING ── STOP accepted ────> STOPPING ──750 ms──> STOPPED
RUNNING/STOPPING ─ Safety ──> FAULTED
```

The provisional transition duration is centralized in
`Core/Motion/motion_config.h` as `MOTION_TRANSITION_DURATION_MS=750U`. The
Motion scheduler is cooperative and only processes foreground calls at the
10 ms bench cadence or slower; each call advances by the wrap-safe unsigned
wall-time delta since the previous processed call. A delayed 70 or 100 ms
foreground pass therefore consumes 70 or 100 ms of the transition instead of
pretending that only one nominal tick elapsed. It does not run gait code from
an interrupt. The protocol STOP path supplies its acceptance timestamp as the
ramp clock origin, so time spent before STOP was accepted is never consumed.

During ordinary STOPPING:

- the generator phase is held;
- the retained logical joint-target vector is interpolated to the all-zero
  neutral vector over the 750 ms ramp;
- every logical target converges monotonically toward zero in integer cdeg;
- Motion ownership remains held, so manual Enable/SetPWM/SetAngle/Neutral
  returns `BUSY` and cannot compete for the actuator;
- the final zero target is written before ownership is released and the state
  becomes `STOPPED`.

Motion START also has an explicit pose handoff. ServoService records the last
known logical angle per enabled channel. Enable initializes a newly enabled
channel to logical neutral; SetAngle, Neutral, and Motion angle writes update
the tracker. Raw SetPWM deliberately marks that channel's logical pose
unknown because no inverse pulse-to-angle contract is assumed. START rejects
an unknown required pose with the existing `HARDWARE_FAILURE` result mapping;
it also rejects a known front logical pose outside `-4500…+2800 cdeg` or a
known rear logical pose outside `-3000…+4500 cdeg` before acquiring Motion
ownership. After a valid known pose is available, START cross-fades the recorded logical targets to the selected
gait target over the same provisional 750 ms window, including the mirrored
left/right channels. During either the initial START ramp or a mode cross-fade,
only a repeated START for the current transition target is idempotently
accepted; a different mode returns `BUSY` without overwriting the transition.

The generator emits logical targets only. The common MotionManager output
guard enforces the front operational envelope `-4500…+2800 cdeg` and rear
operational envelope `-3000…+4500 cdeg` immediately before Servo
calibration/PWM conversion and owns the clamp diagnostic. STOPPING applies the
same guard after interpolating its retained targets, so a future alternate gait
generator or retained transition vector cannot bypass it.

`Disable`/`Disable All`, heartbeat/liveness loss, and the existing
`SafetySupervisor` fail-safe path bypass the ramp. `app_main` evaluates the
SafetySupervisor before non-heartbeat command dispatch and before Motion
scheduling; after Servo Disable payload validation, the dispatcher aborts
Motion only when the requested mask intersects active Motion ownership. A
future leak safety trip must use this same immediate takeover path. The current
PA11 leak implementation is still monitoring-only and has no leak-to-Safety
trip, so this feature does not invent one.

After an interrupted stop, the manager is faulted/stopped as appropriate,
ownership is released, actuators are disabled by the safety caller, and a later
heartbeat/reconnect never resumes the old mode. An explicit re-enable and new
Motion START are required.

## PWM Disable safe-stop

`Neutral` and `Disable` have deliberately different meanings:

- `Neutral` keeps PWM drive enabled, writes the descriptor's legal calibrated
  neutral pulse, and records logical `0 degrees`.
- `Disable` and `Disable All` do not write Neutral. They clear logical actuator
  ownership immediately and stop PWM drive through the channel-local safe-stop
  path. A channel that is already stopped makes repeated Disable a no-op.

TIM3/TIM4 are configured as PWM mode 1, active-high, up-counting timers with
`PSC=15`, `ARR=3002`, and a 1 microsecond timer tick. The resulting period is
`ARR+1 = 3003` ticks, approximately 3 ms / 333 Hz. Because HAL enables
`OCxPE`, a readable `CCR` can be a preload value while the current shadow
compare still drives the pin. Therefore a logically active channel whose timer
is running never uses `CNT`/`CCR` ordering to infer LOW: it always clears the
stale `CCxIF`, publishes a per-channel `stop_pending` bit, enables that
channel's `CCxIE`, and leaves `CCxE` enabled until the next real compare edge.
The driver re-reads `CNT`, `CCR`, and `CCxIF` after arming for the race audit,
but only `CCxIF` can authorize same-edge finalization. The HAL compare callback
runs after HAL clears the flag; only then does the driver call the existing
`HAL_TIM_PWM_Stop()` finalizer. A timer that is not running, or a channel that
is already inactive, may finalize immediately.

The finalizer preserves HAL `ChannelState` and disables only the selected
channel. The STM32 HAL used here gates its `__HAL_TIM_DISABLE()` operation on
all `CCxE/CCxNE` outputs being clear, so a shared TIM3/TIM4 counter continues
for other enabled channels. `stop_pending` is per semantic channel: SetAngle,
ApplyPWM, Motion writes, and Enable are rejected/BUSY until its finalizer runs;
Motion cannot rewrite that channel and Enable cannot race a pending stop.
Disable All arms each channel independently and does not insert a Neutral write.

This physical safe-stop wait is bounded by at most one complete PWM frame plus
compare-ISR service time: approximately 3 ms plus ISR latency in the current
configuration. It is a frame-level electrical edge guarantee, not the separate
750 ms graceful Motion STOP transition. Safety ownership and the high-level
Motion/Safety state change immediately; a safety event interrupts any graceful
ramp and never waits 750 ms for logical target interpolation.

Host tests verify this decision/state policy and the one-frame bound, not the
physical waveform. The remaining evidence is explicitly:
`Physical no-jump Disable: Pending Hardware Re-verification`.
The planned bench checks are: (A) Enable → SetAngle → individual Release PWM;
(B) repeat A several times; (C) Motion → Stop → Disable All; and (D)
simultaneous multi-channel Enable → Disable All. A logic analyzer or oscilloscope is an
optional root-cause confirmation: before the fix, inspect whether the last HIGH
pulse was truncated; after the fix, the last pulse must be either complete or
already in its LOW window when output is disabled. No runt pulse is acceptable.

## Bench-provisional gait profile

`SimpleGaitGenerator` emits five semantic logical targets in this order:

```text
FrontRight, FrontLeft, FrontAxis, RearRight, RearLeft
```

The fallback `SimpleGaitGenerator` table uses 0.5 Hz, 1000 cdeg paddle
amplitude, a π front/rear phase relation, same-phase front and rear pairs,
50% amplitude on the reduced side for turning, and ±1000 cdeg FrontAxis bias
for ASCEND/DESCEND candidates. `FORWARD` keeps the approved front/rear
approximately 180° phase relation; the larger rear paddle area is a mechanical
fact only and does not establish a front/rear amplitude ratio. `BACKWARD` has no
SimpleGait profile and remains Pending. Paddle modes drive the four paddles;
ASCEND and DESCEND require all five enabled channels. Front operational output
is clamped to −4500…+2800 cdeg and rear operational output to
−3000…+4500 cdeg, each through the common diagnostic counter. These values are
bring-up parameters, not hydrodynamic or water-tested calibration.

The production Legacy Source-Compatible CPG profile uses `T=2.0 s` as a nominal
period parameter. Its measured steady-state period is not assumed to be exactly
`2.0 s` or `0.5 Hz`; the source-compatible `nu_i` dependence determines the
observed frequency. See [`cpg-gait-core.md`](cpg-gait-core.md) for the current
long-run measurement.

The normal target path is intentionally explicit:

```text
Motion command
  → MotionManager
  → CPGGaitGenerator
  → logical joint targets (cdeg)
  → ServoService Motion-owned angle API
  → existing Servo calibration
  → PWM driver
```

The explicit `MOTION_DEFAULT_GAIT_BACKEND_CPG=0` diagnostic override replaces
only the generator node with `SimpleGaitGenerator`; the downstream guard,
ServoService, calibration, and PWM path are identical.

## Qt behavior

The Qt Console sends the same exact `0x15` payload. A successful START ACK
changes the local state to `Running`; a successful STOP ACK changes it to
`Stopping`, and a single-shot UI timer based on the same centralized 750 ms
provisional duration changes the display to `Stopped`. The timer is a UI
transition estimate, not actuator confirmation. Transport error, disconnect,
heartbeat fail-closed, and Disable All clear/fault the local Motion state;
liveness fail-closed also clears the local logical enabled/pending masks.
Reconnect does not auto-resume.

While local Motion is `Running`, `Stopping`, or has unresolved in-flight,
queued, or deferred Motion work, manual Servo controls are disabled and the
Controller rejects manual actuator commands locally. A STOP supersedes stale
Motion START/mode requests: DirectUart marks the old pending request
cancelled before sending STOP, while APC220 keeps one exchange in flight and
places STOP above ordinary work but below the safety-priority Disable lane.
Late stale Motion ACKs are ignored. Firmware-side `BUSY` ACKs are decoded and
displayed. An explicit single-channel Disable
uses the same ownership intersection: a non-owned channel may be disabled
without faulting Motion, while an owned channel immediately fail-closes Motion
and cancels queued Motion work. Global `Disable All` remains available and is
safety-prioritized in the existing APC220 bounded scheduler.
During a Running mode change, the Controller retains the old/new required-mask
union for the same provisional crossfade window, including after the mode ACK,
so a channel released only by the new mode cannot race Firmware ownership.

## Deterministic verification

Firmware host coverage is reproducible from the repository root with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\RoboBeetleFirmware\tests\run_host_tests.ps1
```

The runner compiles and executes 29 Firmware test programs with C11,
`-Wall -Wextra -Werror`, and a separate `app_main_jy901s_api` compile
contract. Motion-specific assertions cover:

- `RUNNING → STOPPING → STOPPED`;
- acceptance-time STOP ACK and approximately 750 ms elapsed ramp;
- monotonic/convergent neutral targets and final zero write;
- STOP supersession of DirectUart/APC220 in-flight START, queued mode changes,
  and deferred Motion retry, including stale ACK suppression;
- actual 10/20/70/100 ms Motion wall-time gaps and uint32 timestamp wrap;
- non-neutral manual-pose START cross-fade, mirrored left/right handoff, and
  raw SetPWM unknown-pose rejection through existing `HARDWARE_FAILURE`;
- START rejection for known front and rear poses outside their operational
  envelopes;
- generator-independent front/rear envelope clamping, including the STOPPING
  path, and Motion-owned diagnostics;
- idempotent same-target START and `BUSY` rejection for reentrant different
  modes during START/mode transitions;
- manual Servo `BUSY` arbitration during STOPPING;
- old/new ownership union through an acknowledged mode transition;
- immediate Disable All and heartbeat/liveness takeover;
- default CPG backend selection plus explicit SimpleGait (`=0`) and CPG (`=1`)
  backend-selection tests, with exactly one registered generator per build;
- CPG output beyond the installed front/rear limits is clamped by the common
  MotionManager sanitizer before ServoService observes it;
- PWM1 safe-stop conservative active-running policy, including the
  preload/shadow mismatch regression, stale-flag clearing/recheck policy,
  per-channel pending ownership, one-frame latency bound, shared-timer
  independence, and no Neutral write on Disable All;
- no auto-resume after interrupted stop;
- idempotent STOP while already STOPPED;
- exact `0x15` payload, duplicate replay, invalid-payload rejection, and
  Protocol `BUSY` mapping.

Qt verification uses the existing Console CMake/CTest targets and adds exact
wire, controller, APC220 queue, and MainWindow lifecycle coverage. Hardware
exercise remains a separate pending evidence category.
