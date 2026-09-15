# SimpleGait Bench Default for Front-Assembly Remap — Design Spec

**Status:** Approved by the user's 2026-09-14 diagnostic bench-checkpoint request.

## Goal

Produce a diagnostic Firmware image whose normal Debug/bench build uses the
already verified `SimpleGaitGenerator` as the sole Motion generator, so the
front-assembly mechanical remap can be tested independently of CPG semantics.

## Design

Keep the existing compile-time selector in `Core/App/app_main.c` and change
only its fallback default:

```c
#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 0
#endif
```

The existing `#if` branch then passes exactly one interface to
`motion_manager_init()`:

```text
Qt Motion command
  -> MotionManager
  -> SimpleGaitGenerator
  -> LogicalJointTargets
  -> Motion operational guard
  -> ServoService
  -> ServoCalibration
  -> PWM
```

Both generator objects remain initialized and both source implementations
remain linked. The CPG object is not selected for actuator output in the
normal bench image. The CPG target benchmark remains disabled by its existing
CMake default and is not part of this image's actuator path.

Add a host integration regression that wraps `motion_manager_init()`, captures
the interface supplied by `app_main_init()`, and compares its `ops` pointer to
`simple_gait_generator_interface()`. The test also advances the captured
interface to the SimpleGait Forward quarter-cycle and checks the known
`+1000/+1000/-1000/-1000` logical target pattern. Existing compile-contract
checks continue to compile both backend selector values and both CPG benchmark
values.

Add a build note with the literal evidence string
`Motion backend: SimpleGait (bench-remap verification)`, the source default,
the explicit CPG override, and the fact that this is temporary diagnostic
configuration rather than CPG retirement.

## Preserved behavior and explicit non-goals

- `simple_gait_generator.c` is unchanged: frequency, amplitude, phase offsets,
  turn scaling, and ASCEND/DESCEND bias are the reference stimulus.
- The current FrontRight/FrontLeft/FrontAxis descriptor remap and calibration
  remain unchanged.
- CPG source, CPG golden tests, `theta_dot` semantics, double precision,
  target-performance evidence, Motion safety ordering, Protocol V2, and Qt UI
  remain unchanged.
- No backend selection is added to Protocol V2 or the Qt UI.
- Physical side/sign results and hydrodynamic meaning remain pending until the
  user runs the required hardware checklist on this image.

## Verification boundary

The implementation gate is the complete Firmware host runner, including CPG,
SimpleGait, MotionManager, Servo, Safety, Protocol, and the new app-main
backend regression, plus the Qt clean build/CTest gate and `git diff --check`.
ARM build/program/timing and physical remap evidence remain pending when the
ARM toolchain or hardware is unavailable.
