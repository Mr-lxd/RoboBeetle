# Motion / Gait — Bench-Provisional STOP Contract

This document records the first Motion / Gait control foundation added on
`feature/motion-simple-gait`. It is a host-test/software contract. The branch
does not claim an ARM build, programming/verification, or physical actuator
verification until those steps are independently run.

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

`START` accepts modes 1–6. `STOP` uses mode 0 and action `STOP`. A successful
STOP ACK means that the stop request was accepted and the Firmware has entered
`MOTION_STOPPING`; it does not mean that all targets are already neutral. The
existing successful-request duplicate cache applies to `0x15`, so a retry of
the same sequence/type replays the ACK without repeating the state transition.

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
Motion scheduler is cooperative and ticks every 10 ms in `app_main`; it does
not run gait code from an interrupt.

During ordinary STOPPING:

- the generator phase is held;
- the retained logical joint-target vector is interpolated to the all-zero
  neutral vector over the 750 ms ramp;
- every logical target converges monotonically toward zero in integer cdeg;
- Motion ownership remains held, so manual Enable/SetPWM/SetAngle/Neutral
  returns `BUSY` and cannot compete for the actuator;
- the final zero target is written before ownership is released and the state
  becomes `STOPPED`.

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

## Bench-provisional gait profile

`SimpleGaitGenerator` emits five semantic logical targets in this order:

```text
FrontRight, FrontLeft, FrontAxis, RearRight, RearLeft
```

The current table-driven provisional profile uses 0.5 Hz, 1000 cdeg paddle
amplitude, a π front/rear phase relation, 50% amplitude on the reduced side for
turning, backward stroke-sign inversion, and ±1000 cdeg FrontAxis bias for
ASCEND/DESCEND candidates. Paddle modes drive the four paddles; ASCEND and
DESCEND require all five enabled channels. Rear operational output is clamped
to −3000…+4500 cdeg with a diagnostic counter. These values are bring-up
parameters, not hydrodynamic or water-tested calibration.

The target path is intentionally explicit:

```text
Motion command
  → MotionManager
  → SimpleGaitGenerator
  → logical joint targets (cdeg)
  → ServoService Motion-owned angle API
  → existing Servo calibration
  → PWM driver
```

## Qt behavior

The Qt Console sends the same exact `0x15` payload. A successful START ACK
changes the local state to `Running`; a successful STOP ACK changes it to
`Stopping`, and a single-shot UI timer based on the same centralized 750 ms
provisional duration changes the display to `Stopped`. The timer is a UI
transition estimate, not actuator confirmation. Transport error, disconnect,
heartbeat fail-closed, and Disable All clear/fault the local Motion state;
liveness fail-closed also clears the local logical enabled/pending masks.
Reconnect does not auto-resume.

While local Motion is `Running` or `Stopping`, manual Servo controls are
disabled and the Controller rejects manual actuator commands locally. Firmware
side `BUSY` ACKs are decoded and displayed. An explicit single-channel Disable
uses the same ownership intersection: a non-owned channel may be disabled
without faulting Motion, while an owned channel immediately fail-closes Motion
and cancels queued Motion work. Global `Disable All` remains available and is
safety-prioritized in the existing APC220 bounded scheduler.

## Deterministic verification

Firmware host coverage is reproducible from the repository root with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\RoboBeetleFirmware\tests\run_host_tests.ps1
```

The runner compiles and executes 19 Firmware test programs with C11,
`-Wall -Wextra -Werror`, and a separate `app_main_jy901s_api` compile
contract. Motion-specific assertions cover:

- `RUNNING → STOPPING → STOPPED`;
- acceptance-time STOP ACK and approximately 750 ms elapsed ramp;
- monotonic/convergent neutral targets and final zero write;
- manual Servo `BUSY` arbitration during STOPPING;
- immediate Disable All and heartbeat/liveness takeover;
- no auto-resume after interrupted stop;
- idempotent STOP while already STOPPED;
- exact `0x15` payload, duplicate replay, invalid-payload rejection, and
  Protocol `BUSY` mapping.

Qt verification uses the existing Console CMake/CTest targets and adds exact
wire, controller, APC220 queue, and MainWindow lifecycle coverage. Hardware
exercise remains a separate pending evidence category.
