# Legacy Source-Compatible CPG v1 / RoboBeetle Semantic Adapter

## Status

This document is the approved design for the RoboBeetle CPG gait core
reproduction and integration task. It covers the numerical core, the current
RoboBeetle semantic adapter, backend selection, lifecycle semantics, tests, and
documentation. It does not implement the feature.

The implementation branch is `feature/cpg-gait-core`. Its baseline is
`origin/main` at `bb48f8d057c5a83f1ea91bc88b40b3dbc32419de`.

## Decision summary

The production STM32 C implementation is named **Legacy Source-Compatible CPG
v1**. Its equations, parameters, state update order, numeric representation,
and nominal integration step follow the historical generated STM32 C source,
not a reconstruction from the paper's printed equations.

The core has no knowledge of RoboBeetle Servo IDs, PWM, calibration, shell
clearance, Protocol V2, Qt, HAL, or interrupts. A separate
`CPGGaitGenerator` semantic adapter converts four numerical oscillator outputs
to the current `joint_targets_t` logical-angle contract. `SimpleGaitGenerator`
remains available as the comparison and rollback baseline.

The approved source/model differences remain documented as unresolved
paper/source discrepancies. This design does not call the paper wrong and
does not implement a second paper-faithful production model.

## Evidence and provenance

The formal model evidence is the local thesis PDF:

```text
D:\RoboBeetle\resource\高新义毕业论文.pdf
SHA-256: 7DBD3D58AA5BADFD913DD11B34460903C8E68F2DE23E0004D526AFEF8839D265
```

The numerical and execution-order evidence is the generated source file:

```text
D:\RoboBeetle\resource\CPG（高新义毕业论文 源代码）\CPG（高新义毕业论文 源代码）\stm32_demo\CPG_RoboBeetle_stm_ert_rtw\CPG_RoboBeetle_stm.c
SHA-256: 879BD16C8853AA5191953E8598173EB37CE5104870640E9DF04A3ABE4E314BD3
```

The expected reference directory
`D:\RoboBeetle\resource\深科毕业论文 源代码` is unavailable. It is recorded
as **Expected reference source unavailable**, not silently substituted and not
treated as a blocker for this implementation.

The source uses `real_T`, which is `double` in its `rtwtypes.h`. The new core
therefore uses `double` for state and model arithmetic so the host golden
comparison does not introduce an avoidable float-width change.

## Scope and non-goals

In scope:

- a deterministic four-node, HAL-independent CPG core;
- exact legacy source-compatible numerical golden vectors;
- a RoboBeetle semantic adapter returning logical centidegree targets;
- Forward, conservative left/right turn modulation, ASCEND/DESCEND FrontAxis
  bias, and STOP behavior through the existing generator interface;
- preservation of current MotionManager ownership, transitions, safety, and
  operational envelope behavior;
- host tests, firmware build registration, and source-grounded documentation.

Out of scope:

- a paper-faithful CPG implementation;
- changing Protocol V2 or adding a runtime backend selector;
- Qt CPG/Simple selectors, beta sliders, frequency sliders, or phase-matrix
  editors;
- true Backward gait, phase-inverted reverse motion, or a sign-inverted fake
  reverse mode;
- closed-loop sensory feedback, IMU/depth/PID/ROS2 control, or water
  propulsion claims;
- ServoCalibration, raw PWM, shell-clearance constants, or SafetySupervisor
  changes.

## Architecture and data flow

The production path is:

```text
app_main backend selection
        |
        v
gait_generator_t
        |
        +--> CPGGaitGenerator --> Legacy Source-Compatible CPG v1
        |                              |
        +--> SimpleGaitGenerator       v
                              LogicalJointTargets (joint_targets_t)
                                      |
                                      v
                         MotionManager common guard
                                      |
                                      v
                                  ServoService
                                      |
                                      v
                              ServoCalibration / PWM
```

`MotionManager` continues to own start, mode transition, graceful STOP, the
750 ms transition ownership, immediate abort, and the common rear operational
envelope. CPG output is never applied directly to a timer compare register.

`app_main` will initialize both generator objects and pass the selected
interface to the existing `MotionManager`. The compile-time default is CPG for
this feature branch. A single documented compile-time configuration value can
select SimpleGait for A/B comparison or rollback; no protocol or Qt surface is
added for this choice. Existing direct SimpleGait tests remain unchanged.

The existing `gait_generator_ops_t` is sufficient. CPG uses its `advance`,
`sample`, `is_mode_valid`, and diagnostic-count callbacks without adding Servo
or application dependencies to the core.

## Legacy Source-Compatible CPG v1 core

### State and parameters

The core stores four numerical oscillator slots in historical source order,
but calls them only `legacy_node[0..3]`; these names carry no historical Servo
ID meaning. Each node stores:

- phase `phi_i` in radians;
- amplitude state `r_i`, derivative `r_dot_i`, and source acceleration-memory
  value `r_ddot_mem_i` in logical degree model units;
- offset state `x_i`, derivative `x_dot_i`, and source acceleration-memory value
  `x_ddot_mem_i` in logical degree model units;
- phase-rate memory `phi_dot_mem_i` in radians per second;
- the current target amplitude `R_i` and target offset `X_i`;
- the source output-memory value `theta_i` in logical degrees;
- the source discrete derivative input `theta_dot_i` and its previous scaled
  output sample held by the source UnitDelay state.

Each directed source coupling slot stores the target phase state and its
derivative plus its source acceleration-memory value. The centralized model
profile stores `beta_i`, `T_i`, `k_v_i`, `a_i`, `b_i`, `c_ij`, `w_ij`, desired
phase `DeltaPhi_ij`, target amplitudes, target offsets, and the fixed
integration step.

Units are explicit:

| Quantity | Unit |
| --- | --- |
| `phi`, phase coupling, desired phase | radian |
| `r`, `x`, target amplitude, target offset, raw output | logical degree |
| `r_dot`, `x_dot`, `r_ddot_mem`, `x_ddot_mem` | source discrete model units |
| `phi_dot_mem` | radian per second |
| `T` | second |
| `nu` | Hz |
| `a`, `b`, `c`, `k_v` | source model gain units |
| `joint_targets_t` | signed centidegree |
| nominal integration step | second, `0.01` |

The frozen per-node derivative observation is part of the state contract even
though the generated C keeps it as a local expression rather than a named
state:

| Node | production field | exact source expression | initial value | unit | consumed by |
| --- | --- | --- | ---: | --- | --- |
| 0 | `theta_dot[0]` | `rtb_TSamp - UD_DSTATE`, where `rtb_TSamp = 100 * Memory19_PreviousInput` | `0` | logical degree/s | `exp(-k_v_0 * theta_dot[0])` in `nu_0` |
| 1 | `theta_dot[1]` | `rtb_TSamp_d - UD_DSTATE_o`, where `rtb_TSamp_d = 100 * Memory7_PreviousInput` | `0` | logical degree/s | `exp(-k_v_1 * theta_dot[1])` in `nu_1` |
| 2 | `theta_dot[2]` | `rtb_TSamp_p - UD_DSTATE_b`, where `rtb_TSamp_p = 100 * Memory11_PreviousInput` | `0` | logical degree/s | `exp(-k_v_2 * theta_dot[2])` in `nu_2` |
| 3 | `theta_dot[3]` | `rtb_TSamp_b - UD_DSTATE_a`, where `rtb_TSamp_b = 100 * Memory15_PreviousInput` | `0` | logical degree/s | `exp(-k_v_3 * theta_dot[3])` in `nu_3` |

`theta_dot[i]` is computed from the step-entry output-memory and UnitDelay
values before any Euler state write; it is a scaled discrete difference, not
an analytic derivative or a post-Euler difference. The UnitDelay value is the
previous scaled output sample and is updated to `100 * output_memory[i]` only
after the current `theta_dot[i]` has been consumed.

### Frozen `theta_dot_i` source semantics

`theta_dot_i` is not an analytic derivative and is not a separately named
state in the generated C. It is the local expression formed from a scaled
output-memory sample and a UnitDelay state immediately before each node's
phase-rate expression:

```text
CPG_i[k]       = Memory_i_PreviousInput[k]
TSamp_i[k]     = CPG_i[k] * 100.0
theta_dot_i[k] = TSamp_i[k] - UD_i[k]
               = 100.0 * (Memory_i[k] - Memory_i[k-1])
```

The four exact generated-source symbol pairs are:

| Legacy node | output-memory symbol read at step start | scaled sample | previous-sample state | `theta_dot` expression |
| --- | --- | --- | --- | --- |
| node 0 | `CPG_1 = Memory19_PreviousInput` | `rtb_TSamp = CPG_1 * 100.0` | `UD_DSTATE` | `rtb_TSamp - UD_DSTATE` |
| node 1 | `CPG_2 = Memory7_PreviousInput` | `rtb_TSamp_d = CPG_2 * 100.0` | `UD_DSTATE_o` | `rtb_TSamp_d - UD_DSTATE_o` |
| node 2 | `CPG_3 = Memory11_PreviousInput` | `rtb_TSamp_p = CPG_3 * 100.0` | `UD_DSTATE_b` | `rtb_TSamp_p - UD_DSTATE_b` |
| node 3 | `CPG_4 = Memory15_PreviousInput` | `rtb_TSamp_b = CPG_4 * 100.0` | `UD_DSTATE_a` | `rtb_TSamp_b - UD_DSTATE_a` |

The source output and `ampli` values use the same logical-angle numeric unit;
the adapter interprets that unit as logical degree. Because `TSamp_i` is the
source output multiplied by `1/(w*Ts)=100`, `theta_dot_i` has logical degrees
per second when the source output is expressed in logical degrees. The
UnitDelay state has the same scaled-output/per-second unit. The source has no
dedicated `theta_dot` integrator or derivative state beyond these UnitDelay
values.

Static-storage initialization gives all four `UD_DSTATE*` values zero. The
historical `CPG_RoboBeetle_stm_initialize()` initializes `gait`, `prd`,
`ampli`, and `Beta` but does not overwrite the UnitDelay or output-memory
states; the generated C static block state is therefore initially zero.

For every source step, the order is frozen as follows:

1. `Def_pa` selects the signed target amplitudes and source coupling arrays.
2. Each `Memory*_PreviousInput` is read into `CPG_i`, then `TSamp_i` and
   `theta_dot_i` are formed from the current UnitDelay value.
3. The amplitude, offset, phase-rate, and phase-target accelerations are
   evaluated from pre-Euler states. Each `theta_dot_i` used by `nu_i` is thus
   a pre-Euler discrete difference of the output-memory sequence.
4. The source computes the next output-memory values as
   `r_i[k]*sin(phi_i[k]) + x_i[k]`, using pre-Euler oscillator states. The
   generated code writes these values to `Memory7`, `Memory11`, `Memory15`,
   and finally `Memory19` before any oscillator Euler state write.
5. The generated C then interleaves UnitDelay and per-node Euler writes in
   this order: node 0 `UD_DSTATE` followed by node 0 amplitude/offset/phase
   integrators; node 1 `UD_DSTATE_o` followed by node 1 integrators; node 2
   `UD_DSTATE_b` followed by node 2 integrators; and node 3 `UD_DSTATE_a`
   followed by node 3 integrators. Each UnitDelay receives the current
   pre-step scaled sample (`UD_i[k+1] = TSamp_i[k]`).
6. The per-node Euler writes use the previous `Memory*_PreviousInput` values:
   phase uses the previous phase-rate memory, amplitude derivative uses the
   previous amplitude-acceleration memory, and offset derivative uses the
   previous offset-acceleration memory. The current phase-rate, amplitude
   acceleration, and offset acceleration expressions are written to their
   corresponding source memories only after these state updates.
7. The 12 phase-target integrators and their derivative-memory arrays receive
   their `0.01` Euler updates after the per-node writes: each target state uses
   the pre-step target derivative, each target derivative uses the previous
   target-acceleration memory, and the current target acceleration is then
   stored for the next step. No derivative expression is recalculated after a
   state write, and the source does not recompute output-memory values from
   post-Euler oscillator states until a later step.

The new core must expose `theta_dot_i` and the corresponding previous-scaled
sample in its golden snapshot. This prevents an implementation from replacing
the source difference with a mathematically plausible analytic derivative.

The source anchors for this contract are `CPG_RoboBeetle_stm.c:138-145`,
`:186-199`, `:211-218`, `:254-267`, `:296-303`, `:343-356`, `:385-392`, and
`:432-446` for the four scaled samples and phase-rate expressions;
`:275-278`, `:364-367`, `:454-457`, and `:480-483` for output-memory writes;
and `:491`, `:543`, `:573`, and `:601` for UnitDelay writes. The initialization
entry point is at `:853-861`; the UnitDelay and output-memory state remains
zero through C static-storage initialization rather than an explicit reset
assignment in that function.

The production default profile uses the source-compatible formula and
parameter layout. The golden fixture explicitly uses the historical source
initialization: `beta_i=0.75`, `T_i=1.0 s`, and `ampli_i=30 logical degrees`.
The bench profile keeps front and rear nominal amplitudes as separate
centralized fields and starts both at the same conservative logical amplitude
of `10 degrees`; this is a neutral bench placeholder, not a verified
front/rear hydrodynamic ratio. It uses `T=2.0 s` as a nominal period
parameter; the actual steady-state period is not assumed to be exactly 2.0 s
because source `nu_i` depends on `beta_i`, `theta_dot_i`, and `k_v_i`. The
paper's `45 degrees` / `1.3 Hz` example is not used as a default.

### Equations and source choices

The core implements the source-compatible form of the following model:

```text
nu_i = [ (2*beta_i - 1)
          / (2*beta_i*(1 - beta_i)*(exp(-k_v_i*theta_dot_i) + 1))
          + 1/(2*beta_i) ] / T_i

r_ddot_i = a_i * [ (a_i/4)*(R_i - r_i) - r_dot_i ]
x_ddot_i = b_i * [ (b_i/4)*(X_i - x_i) - x_dot_i ]

DeltaPhi_t_ddot_ij = c_ij * [ (c_ij/4)*(DeltaPhi_ij - DeltaPhi_t_ij)
                              - DeltaPhi_t_dot_ij ]

phi_dot_i = 2*pi*nu_i
            + sum_j w_ij*sin(phi_j - phi_i - DeltaPhi_t_ij)

theta_i = x_i + r_i*sin(phi_i)
```

The implementation follows the historical generated source's actual
three-neighbor coupling slots and source constants. It does not infer a new
coupling graph from the paper image.

For reproducibility, the source-compatible baseline parameters are recorded
here in the generated source's flattened slot order:

```text
k_v       = [1, 1, 1, 1]
beta      = [0.75, 0.75, 0.75, 0.75]
T         = [1, 1, 1, 1] seconds
ampli     = [30, 30, 30, 30] logical degrees
X         = [0, 0, 0, 0] logical degrees
DeltaPhi  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] radians
w         = [2, 2, 0, 2, 0, 2, 2, 0, 2, 0, 2, 2]
c         = [20, 20, 0, 20, 0, 20, 20, 0, 20, 0, 20, 20]
a         = 20 for every oscillator
b         = 20 for every oscillator
dt        = 0.01 seconds
```

The zero entries in the flattened `w` and `c` arrays are retained as source
slot values; their slot ordering is not reinterpreted as a current logical
joint order. For the historical Forward initialization (`gait=1`), the
source target assignment is:

```text
R_source = [-ampli[0], +ampli[1], +ampli[2], -ampli[3]]
```

The production bench profile changes only the centralized target-amplitude
and period values for conservative operation; it does not change the model
equations, source slot topology, beta term, or integrator.

The source's `Def_pa` behavior for the Forward baseline produces the signed
target pattern `[-R_0, +R_1, +R_2, -R_3]`, with zero desired phase. The core
keeps that signed-amplitude behavior. A negative target amplitude therefore
acts as an equivalent `pi` output phase inversion for the periodic output; it
is not a Servo direction or PWM convention.

Each source-compatible step uses Forward Euler at `0.01 s`. The generated
source updates the output-memory values before writing the Euler-updated
states. The new core preserves that order and exposes both the source-emitted
raw output and the post-step internal state so golden vectors can distinguish
them rather than silently shifting the comparison by one sample.

### Elapsed-time catch-up

`MotionManager` continues to pass the wrap-safe actual elapsed wall time. The
CPG adapter never assumes that one foreground call equals 10 ms.

The adapter accumulates elapsed milliseconds and advances the core in 10 ms
Euler substeps. A remainder below 10 ms is retained for the next call. This
gives exact source behavior at nominal cadence while preserving elapsed-time
semantics for 20/70/100 ms foreground gaps.

### Safety-before-catch-up contract

Liveness has final authority over any elapsed-time catch-up. Both the existing
`app_main_process()` safety prepass and the defensive check at the beginning
of `motion_manager_process()` must run before `motion_manager_tick()` can call
the selected generator. The active-path order is therefore:

```text
process(now_ms)
  -> if active and host is not alive:
       immediate Motion abort; release Servo ownership; return
       (no CPG advance, no CPG sample, no Servo write)
  -> compute wrap-safe elapsed time
  -> if cadence is due, CPG catch up in bounded substeps
  -> sample logical targets, apply common guard, write through ServoService
```

If a foreground gap is already beyond the heartbeat/liveness deadline, the
first branch wins. A stale active Motion must not catch up 70 steps and write
an actuator command before the heartbeat abort. The integration regression
therefore counts generator advances and Servo writes across a gap greater than
`SAFETY_SUPERVISOR_HEARTBEAT_TIMEOUT_MS`, requiring safety abort, zero
post-gap CPG actuator commands, and no implicit resume after heartbeat
recovery.

To make extreme gaps bounded and deterministic, one `advance` call processes
at most `100` source substeps (`1.0 s`). Excess elapsed time is discarded after
the cap and increments the CPG catch-up diagnostic counter; it is not allowed
to create an unbounded loop. Under the existing 500 ms heartbeat timeout,
normal active Motion processing should fail-safe before this defensive cap is
needed for a live actuator path. The behavior is still directly unit tested.

## RoboBeetle semantic adapter

The adapter is the only layer that assigns numerical outputs to current
logical joints. It does not claim that historical `CPG_1` through `CPG_4`
were physically wired as current Servo IDs.

The approved semantic topology is:

- FrontRight and FrontLeft share phase;
- RearRight and RearLeft share phase;
- the Front pair and Rear pair are approximately `pi` apart;
- the front/rear amplitude ratio is unknown;
- both front and rear pair strokes are retained as the Forward bench
  candidate.

The adapter uses an explicit semantic table over anonymous legacy slots:

```text
FrontRight <- legacy_node[0]
FrontLeft  <- legacy_node[3]
RearRight  <- legacy_node[1]
RearLeft   <- legacy_node[2]
```

This groups the source signed-amplitude pattern's two negative nodes as the
Front pair and its two positive nodes as the Rear pair. The table is an
adapter policy derived from the confirmed current topology, not a historical
Servo mapping. The mapping is covered by tests using effective phase
(`phi + pi` when the signed amplitude is negative) and logical output values.

The adapter's profiles are:

| Mode | Paddle targets | Turn behavior | FrontAxis |
| --- | --- | --- | --- |
| STOP | zero logical output | none | zero |
| FORWARD | conservative front/rear nominal amplitudes | none | zero |
| TURN_LEFT | same topology | reduce left pair to 50% | zero |
| TURN_RIGHT | same topology | reduce right pair to 50% | zero |
| ASCEND | Forward paddle topology | none | `+1000 cdeg * bias_scale` |
| DESCEND | Forward paddle topology | none | `-1000 cdeg * bias_scale` |
| BACKWARD | invalid | not implemented | not applicable |

Turn is implemented by selecting a mode-specific signed target-amplitude
vector in the semantic adapter before the next core advance. If `F` is the
front amplitude, `R` is the rear amplitude, and `s` is the profile's reduced
side scale, the vectors are:

```text
FORWARD/ASCEND/DESCEND: [-F, +R, +R, -F]
TURN_LEFT:              [-F, +R, s*R, -s*F]
TURN_RIGHT:             [-s*F, s*R, +R, -F]
```

The left pair is legacy nodes 3/2 and the right pair is legacy nodes 0/1.
The adapter returns the current raw CPG state with only the existing global
`amplitude_scale`; it does not apply an additional left/right output scale.
Consequently a turn sample cannot jump solely because of mode selection. The
next 10 ms core advances consume the new target through the legacy
`r/r_dot/r_ddot` state and preserve the existing `theta_dot -> nu_i` feedback.
The numerical core contains no `TURN_LEFT` or `TURN_RIGHT` branch.
Front/rear phase topology and the signed source convention remain unchanged.

`FrontAxis` is a profile bias only; it is not a fifth oscillator. ASCEND and
DESCEND remain `Bench Provisional / Pending Water Verification`.

The adapter rounds raw logical degrees to signed centidegrees when constructing
`joint_targets_t`. It does not clamp rear targets. Rear targets then pass
through the existing MotionManager common sanitizer (`-3000..+4500 cdeg`) and
the existing ServoService logical-angle and calibration checks.

Adapter regressions verify both mode semantics and the dynamic production
trajectory: each turn mode installs its expected signed target vector, the
current sample remains the current raw output, and later 10 ms advances move
the amplitude state toward the installed target. A 5000-step production run
samples actual advanced CPG output, confirms exact front/rear pair symmetry,
keeps the signed effective front/rear phase error within the documented
approximate-phase bound, and enforces a 1500 cdeg conservative output envelope.

## Mode, reset, and ownership semantics

The protocol and `motion_mode_t` values remain unchanged. `BACKWARD` remains
wire-compatible but is rejected by the CPG generator, so no phase shift or
sign inversion is presented as reverse locomotion.

Reset and lifecycle behavior is:

1. Generator initialization and explicit test reset zero all phase, amplitude,
   offset, derivative, output-memory, and elapsed-remainder state. It installs
   the centralized profile parameters and starts with zero current output.
2. `MOTION_START` does not reset the oscillator. The existing MotionManager
   cross-fades from the recorded logical pose over 750 ms while CPG state stays
   continuous.
3. Graceful STOP does not advance the CPG while MotionManager owns the
   750 ms output ramp. The core phase and amplitude state remain available for
   a later explicit restart; MotionManager still writes the final all-zero
   logical target and releases ownership only at the existing deadline.
4. A new START after STOP or a safety fault preserves oscillator continuity;
   reconnect never auto-resumes because that remains a MotionManager and
   SafetySupervisor rule. A full generator reinitialization is the only normal
   reset to phase zero.
5. A running mode transition preserves phase. The semantic profile changes
   target amplitude, the core's amplitude dynamics respond smoothly, and
   MotionManager blends the old and new logical target vectors over 750 ms.

The existing generator ABI supplies the mode to `sample`, so the adapter
installs the corresponding target vector when logical targets are emitted.
During a MotionManager mode transition, the destination-mode sample is the
last sample and therefore leaves the destination vector installed for the
next core advance. MotionManager still owns the actuator-level 750 ms blend;
the adapter does not change source-core state ordering or add a mode branch to
the numerical model.

## Paper/source discrepancy record

The following are unresolved paper/source discrepancies. The engineering
choice for this PR is Legacy Source-Compatible CPG v1 in every case.

| Topic | Paper evidence | Legacy source evidence | PR choice | Status |
| --- | --- | --- | --- | --- |
| Nonlinear beta term | Eq. 4-1 prints `2*beta_i*(2*beta_i-1)` | generated C evaluates `2*Beta[i]*(1-Beta[i])` | source term | Unresolved paper/source discrepancy |
| Beta initialization | Table 4-2 shows `beta=[0.5,0.5,0.5,0.5]` | `CPG_RoboBeetle_stm_initialize()` sets every `Beta[i]=0.75` | `0.75` | Unresolved paper/source discrepancy |
| Integrator | thesis text describes fourth-order Runge-Kutta in Simulink | generated embedded C uses fixed Forward Euler updates with `0.01` | Forward Euler `0.01 s` | Unresolved paper/source discrepancy |

Future paper-faithful equations, Euler/RK4 comparison, beta comparison, and
closed-loop sensory feedback are documented as research interfaces only. They
are not compiled into this PR's production path.

## Verification design

### STM32F407 double-precision performance evidence

The production core remains `double` throughout this PR. The STM32F407's
single-precision hardware FPU is a performance consideration, not permission
to change the source-compatible numeric type. A target benchmark is added
alongside the firmware verification work and uses the same compiler,
optimization flags, clock configuration, and linker settings as the firmware
image.

The evidence record contains:

- baseline and feature image `text`, `data`, and `bss` sizes from the ARM
  size/map output;
- FLASH delta as `(text + data)_feature - (text + data)_baseline`;
- RAM delta as `(data + bss)_feature - (data + bss)_baseline`;
- DWT cycle-counter timing for one nominal 10 ms CPG core substep;
- DWT timing for representative 20 ms, 70 ms, and 100 ms elapsed catch-up
  calls, corresponding to 2, 7, and 10 source substeps;
- worst-case bounded catch-up timing for the configured 100-substep cap;
- target clock, compiler flags, optimization level, measurement repetitions,
  and observed min/max values.

Nominal single-substep execution must be compared with the 10 ms scheduler
period. Catch-up measurements are reported separately from the safety
contract; a stale active Motion is aborted before catch-up and cannot use a
long-gap timing result to justify an actuator write. If the measured `double`
implementation cannot meet the target real-time budget, the result is
reported as a blocker for this production profile and the follow-up design is
explicitly a double-reference / float-production parity study. This PR does
not silently switch to float and does not claim target timing evidence when
the ARM toolchain or target measurement surface is unavailable.

### Independent numerical golden

The golden fixture is created from the historical STM32 C algorithm behavior
before or independently of the new production implementation. Tests must not
generate expected data by calling the new core and then compare it to itself.

The source-compatible golden test executable is named
`cpg_legacy_source_compatible_tests`, and its test names include
`legacy_source_compatible`. It covers the exact observation times:

```text
0, 0.01, 0.1, 0.5, 1.0, 2.0, 5.0 seconds
```

For all four oscillator slots, each observation records and compares:

- post-step internal phase/state;
- amplitude state and derivative;
- target amplitude;
- `beta`;
- exact `theta_dot_i` discrete difference;
- the corresponding previous-scaled-output UnitDelay state (`UD_DSTATE*`);
- source-emitted raw oscillator output;
- output-memory ordering at the first sample.

The frozen fixture records the historical source path and SHA-256 above. Its
tolerance is explicit and appropriate for the source's `double` host
arithmetic; no tolerance is used to hide a state-order or sign-convention
error.

Additional source-compatible scenarios cover:

- cold start from all-zero state;
- target amplitude change from 10 to 20 logical degrees;
- frequency/period change;
- signed target amplitude and equivalent phase inversion;
- reset and replay determinism.

### Core properties

Host tests cover beta `.75` behavior, finite/no-NaN/no-Infinity state,
amplitude convergence, phase/frequency behavior, long-run boundedness,
deterministic fixed-step catch-up, remainder retention, and the 100-substep
maximum-gap policy.

The long-run period test runs the production nominal-period profile for a
fixed multi-cycle window after its amplitude transient. It detects repeated
same-direction zero crossings (or equivalent stable output phase markers),
requires a positive finite measured period, and reports:

```text
nominal_period_s, measured_period_s, measured_frequency_hz,
measured_period / nominal_period
```

It does not label `T=2.0 s` as the actual oscillator period unless the measured
source-compatible trajectory supports that conclusion. The same measurement
is taken from the production adapter's deterministic trajectory; the separate
source differential check compares the core states directly without treating
`1/T` as a fixed output frequency.

### Adapter and integration

Adapter tests cover:

- FrontRight/FrontLeft effective phase equality;
- RearRight/RearLeft effective phase equality;
- Front/Rear effective phase difference of approximately `pi`;
- signed logical centidegree targets and no PWM dependency;
- Forward and conservative turn asymmetry;
- FrontAxis-only ASCEND/DESCEND bias;
- STOP output and BACKWARD rejection.

Motion integration tests cover CPG Forward start, 750 ms graceful STOP,
mode transition continuity, turn target asymmetry, downstream rear envelope
clamping, Disable All immediate takeover, and reconnect without auto-resume.
The existing firmware host regression suite remains mandatory.

The firmware CMake target and host PowerShell test runner will register the new
core and adapter sources/tests. Qt configure/build and ARM cross-build results
will be reported separately from host evidence; unavailable toolchains are not
represented as passing.

## Documentation and review closeout

The implementation documentation will use the full name **Legacy
Source-Compatible CPG v1** wherever the model is discussed. It will preserve
the core/adapter/target/safety diagram, exact units and equations, source
provenance, discrepancy table, reset semantics, golden-vector provenance,
SimpleGait comparison boundary, and verified versus pending water behavior.

The PR will be pushed as a new PR targeting `main` and will not be merged.
Final closeout must report the exact branch, baseline, commit SHA, changed
files, `git diff --check`, host/Qt/ARM verification boundaries, PR state, and
the explicit handoff line:

```text
READY FOR EXTERNAL GITHUB REVIEW
```
