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
if requested backend is not registered with a usable generator/reset:
    return HARDWARE_FAILURE
if requested == current:
    return OK / deterministic no-op
reset newly selected generator
update backend and active generator
remain STOPPED
```

This ordering is intentional: `RUNNING` and `STOPPING` return `BUSY` even
when the requested backend is already current. `FAULTED` is also not an
accepted switching state because the contract is STOPPED-only.

The state check is performed before backend-value validation. Therefore a
moving/stopping request with byte `0xff` still returns `BUSY`; the invalid
backend result is reachable only after the manager is already `STOPPED`.
After a valid backend value is established, a missing target interface,
missing context, or missing reset callback returns `HARDWARE_FAILURE` without
changing the current backend, active generator, Motion state, targets, or
Servo state. This infrastructure check also prevents a same-backend no-op
from masking an unusable registration.

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
- CPG exposes a backend-level
  `cpg_gait_generator_reset(cpg_gait_generator_t *generator)` API. It copies
  the configured `cpg_gait_profile_t`, then reconstructs the core through the
  same initialization path as `cpg_gait_generator_init_with_profile()`. A
  bare `cpg_core_reset()` is insufficient because
  `cpg_gait_generator_sample()` mutates `core.params.target_amplitude` for the
  selected Motion mode and `cpg_core_reset()` preserves that mutable field.
  The backend reset therefore restores the canonical profile-derived initial
  parameters instead of retaining a previous TURN target-amplitude vector.

The CPG reset preserves both custom and production profiles while clearing
all oscillator/runtime state, including phase, derivative/memory state,
output memory, elapsed remainder, executed-step count, and discarded
catch-up count. Its resulting core is the same deterministic initial state
that a fresh `cpg_gait_generator_init_with_profile()` would produce for the
same profile. It does not change beta, `dt`, `theta_dot` semantics, phase
coupling, or the production `double` representation. `cpg_core_reset()` may
remain available for core-level callers, but it is not the backend selector's
sole CPG reset operation.

The required regression constructs a fresh CPG with a selected profile,
advances/samples a TURN mode, stops the Motion lifecycle, switches to
SimpleGait, switches back to CPG, and resets before the next Motion START.
The reset CPG state must equal the fresh profile-matched CPG state, including
the profile-derived mutable target amplitudes. The regression runs for the
production profile and at least one custom profile.

No CPG equations or model parameters are changed by this feature.

The production application registers both initialized generators with
`MotionManager` using an explicit multi-backend initializer:

```c
void motion_manager_init_with_backends(
    motion_manager_t *manager,
    servo_service_t *servo_service,
    safety_supervisor_t *safety_supervisor,
    gait_generator_t simple,
    gait_generator_t cpg,
    motion_gait_backend_t initial_backend);
```

`initial_backend` is selected explicitly from the existing
`MOTION_DEFAULT_GAIT_BACKEND_CPG` compile-time default/fallback. No function
pointer or context identity is inspected to infer a backend.

The existing single-generator initializer remains source-compatible for
legacy/custom host fixtures, but its backend identity is
`MOTION_GAIT_BACKEND_UNSPECIFIED` and its runtime selector is unavailable.
It retains the supplied generator as the active legacy generator for those
fixtures; it never guesses whether that generator is SimpleGait or CPG. A
valid selector request made against such an unregistered backend returns
`MOTION_MANAGER_RESULT_HARDWARE_FAILURE` with no mutation.

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

The valid backend enum values are exactly `SIMPLE_GAIT=0` and `CPG=1` for
wire use. `MOTION_GAIT_BACKEND_UNSPECIFIED` is an internal-only legacy state;
it is never encoded and is not a valid selector payload. A current backend
getter may report `UNSPECIFIED` for a legacy single-generator manager, while
the multi-backend production manager always starts with the explicit initial
backend.

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

Dispatcher precedence and behavior:

```text
if payload length != 1:
    return RBP2_RESULT_INVALID_PAYLOAD
decode the exact one-byte payload as a backend value
→ call motion_manager_set_gait_backend()
→ map result to existing ACK result
```

The dispatcher does not pre-validate the byte value or inspect Motion state.
For an exact one-byte payload, MotionManager receives the request and applies
the state-first precedence: non-`STOPPED` returns `BUSY` even for an invalid
byte such as `0xff`; `STOPPED` plus an invalid byte returns
`MOTION_MANAGER_RESULT_INVALID_BACKEND`, mapped to
`RBP2_RESULT_INVALID_PAYLOAD`.

The mappings are:

| Condition | ACK result |
| --- | --- |
| valid STOPPED switch or same-backend no-op | `RBP2_RESULT_OK` |
| RUNNING, STOPPING, or other non-STOPPED state | `RBP2_RESULT_BUSY` |
| invalid payload length/value | `RBP2_RESULT_INVALID_PAYLOAD` |
| manager/backend infrastructure failure | existing `RBP2_RESULT_HARDWARE_FAILURE` |

The explicit regression `RUNNING + backend=0xff` must produce
`RBP2_RESULT_BUSY`, while `STOPPED + backend=0xff` produces
`RBP2_RESULT_INVALID_PAYLOAD`.

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
- at most one selector request may be outstanding at a time, including an
  APC220 queued/deferred selector or an in-flight/retrying selector;
- while one selector request is outstanding, the Controller rejects or the UI
  disables further selector changes; it never queues a second selector;
- the requested value is retained separately while its ACK is pending;
- matching ACK `OK` updates the confirmed backend;
- only an ACK matching the original request sequence and
  `SetGaitBackend` type can confirm the request;
- `BUSY`, invalid/error ACK (including a sequence/type mismatch), timeout,
  transport error, or write failure clears the pending request but preserves
  the previous confirmed value;
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
- RUNNING selection with backend byte `0xff` returns `BUSY` before invalid
  backend validation;
- invalid backend returns `MOTION_MANAGER_RESULT_INVALID_BACKEND`;
- legacy single-generator initialization does not infer backend identity and
  a missing registered generator/reset infrastructure returns
  `MOTION_MANAGER_RESULT_HARDWARE_FAILURE` without mutation;
- moving rejection leaves backend, Motion state, generator state, and Servo
  output unchanged;
- successful switching does not start Motion or write Servo output;
- SimpleGait reset is deterministic;
- CPG reset preserves production and custom profile/model parameters, restores
  profile-derived target amplitudes, and clears runtime oscillator,
  elapsed/remainder, executed, and discarded-catch-up state;
- fresh CPG initialization equals TURN → STOP → switch away → switch back/reset
  before the next Motion START;
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
- selector serialization rejects or disables a second request while the first
  is queued/in flight/retrying;
- ACK sequence/type mismatch cannot confirm a selector and clears its pending
  state;
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
