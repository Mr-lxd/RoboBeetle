# Runtime SimpleGait / CPG Selector Design — 2026-09-15

## Goal

Add a runtime selector for the two existing Firmware gait generators:

```text
SIMPLE_GAIT = 0
CPG         = 1
```

The Qt Console may request a backend, but the STM32 Firmware owns the
authoritative backend and performs the switch. Qt must never calculate or
stream per-frame Servo trajectories.

The existing production default remains CPG and the existing
`MOTION_DEFAULT_GAIT_BACKEND_CPG` compile-time default/fallback contract is
retained.

## Non-goals and frozen boundaries

This feature does not change:

- CPG beta, `dt`, `theta_dot` semantics, phase coupling, or numeric types;
- CPG mathematics or Servo calibration;
- Clock/RCC, PWM, UART, APC220 blocking behavior, queue budgets, or
  foreground scheduling;
- Motion cadence, Safety timeout values, JY901S, Depth, Leak, or Water
  behavior;
- Servo stutter, communication backlog, or cadence-jitter fixes;
- runtime gait trajectory generation in Qt;
- backend telemetry or a backend query/readback protocol.

The existing foreground order remains unchanged:

```text
UART host queue drain
→ JY901S queue drain
→ Depth queue drain
→ HAL_GetTick
→ Safety
→ MotionManager
```

## State and safety contract

`MotionManager` owns the current backend and active generator interface. The
backend selector is accepted only when the Motion state is exactly
`MOTION_STATE_STOPPED`.

The required decision order is:

```text
if state != STOPPED:
    return BUSY
validate requested backend
if requested == current:
    return OK / deterministic no-op
reset newly selected generator
update backend and active generator
remain STOPPED
```

This ordering is intentional: `RUNNING` and `STOPPING` return `BUSY` even
when the requested backend is already current. `FAULTED` is also not an
accepted switching state because the contract is STOPPED-only.

A successful switch must not:

- call Servo write or Servo ownership APIs;
- automatically request or complete Motion STOP;
- start Motion;
- produce trajectory output;
- advance the scheduler or generator;
- inherit phase, elapsed time, or other runtime state from the old backend.

After a successful switch, the Motion state, active Motion mode, last Servo
target/output, and ownership state remain unchanged. The newly selected
generator is reset before it becomes active.

## Generator abstraction

`gait_generator_ops_t` gains:

```c
void (*reset)(void *context);
```

The existing generator interface remains the boundary through which
`MotionManager` advances and samples a generator.

Reset behavior is deterministic and backend-local:

- SimpleGait resets `phase_rad` to its initialized value (`0.0F`).
- CPG uses the existing `cpg_core_reset()`, preserving the existing
  profile/model parameters while clearing oscillator runtime state,
  elapsed remainder, executed-step count, and discarded catch-up count.

No CPG equations or model parameters are changed by this feature.

The production application registers both initialized generators with
`MotionManager`. An additive multi-backend initialization API is preferred so
the existing single-generator initializer remains source-compatible for
legacy/custom host fixtures. The production call still selects its initial
backend from `MOTION_DEFAULT_GAIT_BACKEND_CPG`.

## MotionManager API and result mapping

The manager exposes an API with the following semantics:

```c
motion_manager_result_t motion_manager_set_gait_backend(
    motion_manager_t *manager,
    motion_gait_backend_t backend);
```

The internal result set adds `MOTION_MANAGER_RESULT_INVALID_BACKEND`.
Existing results, including `BUSY`, remain unchanged. The new result maps to
the existing wire-level `RBP2_RESULT_INVALID_PAYLOAD`; no new wire result
code is introduced.

The manager also exposes its current backend for host assertions and local
diagnostics. It is not emitted as a new telemetry frame.

## Protocol V2 contract

Add:

```text
RBP2_MSG_SET_GAIT_BACKEND = 0x16
```

The command payload is exactly one byte:

```text
backend uint8
  0 = SIMPLE_GAIT
  1 = CPG
```

Dispatcher behavior:

```text
decode exact one-byte payload
→ call motion_manager_set_gait_backend()
→ map result to existing ACK result
```

The mappings are:

| Condition | ACK result |
| --- | --- |
| valid STOPPED switch or same-backend no-op | `RBP2_RESULT_OK` |
| RUNNING, STOPPING, or other non-STOPPED state | `RBP2_RESULT_BUSY` |
| invalid payload length/value | `RBP2_RESULT_INVALID_PAYLOAD` |
| manager/backend infrastructure failure | existing `RBP2_RESULT_HARDWARE_FAILURE` |

The dispatcher does not hold backend state, select generators, reset
generators, write Servo output, or alter Motion state directly. The existing
successful-request duplicate ACK cache applies to `0x16` exactly as it does
to other acknowledged commands.

## Qt state and UI contract

The Qt Controller adds a `GaitBackend` value matching the wire values and a
`setGaitBackend()` command method. It tracks the selector lifecycle as:

```text
UNKNOWN
  → REQUESTED
  → CONFIRMED_SIMPLE_GAIT
  → CONFIRMED_CPG
```

The confirmed value is local ACK-confirmed command state, not independent
Firmware telemetry. No backend query is added in this feature.

Behavior:

- a user selection sends only `SetGaitBackend`;
- the requested value is retained separately while its ACK is pending;
- matching ACK `OK` updates the confirmed backend;
- `BUSY`, invalid/error ACK, timeout, transport error, or write failure clears
  the request but preserves the previous confirmed value;
- disconnect and reconnect reset the controller's backend state to `UNKNOWN`;
- the controller never starts/stops Motion as part of backend selection;
- the controller never sends Servo targets for a backend selection.

The MainWindow adds:

```text
Gait Backend: [ SimpleGait | CPG ]
```

The UI must not display a requested value as confirmed before ACK `OK`.
Pending/rejected requests remain visible through a pending/status indication
or revert to the last confirmed value. `BUSY` is surfaced through the
existing protocol monitor/log path.

## Verification plan

### Firmware

Add failing tests before implementation for:

- default backend is CPG under the normal compile-time default;
- STOPPED CPG → SimpleGait and SimpleGait → CPG return `OK`;
- STOPPED same-backend selection returns deterministic `OK/no-op`;
- RUNNING same-backend selection returns `BUSY`;
- STOPPING selection returns `BUSY`;
- invalid backend returns `MOTION_MANAGER_RESULT_INVALID_BACKEND`;
- moving rejection leaves backend, Motion state, generator state, and Servo
  output unchanged;
- successful switching does not start Motion or write Servo output;
- SimpleGait reset is deterministic;
- CPG reset preserves profile/model parameters and clears runtime oscillator,
  elapsed, executed-step, and discarded-catch-up state;
- Protocol `0x16` exact payload encode/decode, invalid length/value, ACK `OK`,
  and ACK `BUSY` mappings.

The existing Firmware host gate remains mandatory.

### Qt

Add tests for:

- selector payload encoding and known message type `0x16`;
- SimpleGait and CPG selection;
- ACK `OK` confirmation;
- `BUSY` handling without changing confirmed state;
- timeout/error without falsely confirming a request;
- disconnect/reconnect returning selector state to `UNKNOWN`;
- MainWindow combo presence and command dispatch;
- no Qt trajectory or Servo-frame path is used by selector handling.

Qt CMake/CTest is required when the Qt6 development package is available. If
the environment still lacks Qt6, the result must be reported as not run, not
as a pass.

## Review and integration boundary

The feature is implemented on the independent branch
`codex/runtime-gait-selector`, based on
`13ea8aba453cb4bc49aa04c71748fb630563b15f`. The final change must be limited
to the selector implementation, its tests, and directly relevant protocol/UI
documentation. The branch will be pushed and an independent PR opened for
external review; it will not be merged in this task.
