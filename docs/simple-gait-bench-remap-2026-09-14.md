# SimpleGait Bench Image — Front-Assembly Remap Checkpoint

## Purpose

This records the completed SimpleGait mechanical-remap diagnostic for the
2026-09-14 front assembly. It isolated the SimpleGait stimulus from the
source-compatible CPG dynamics and established the installed mechanical
baseline before the next CPG target/hardware verification phase.

## Build evidence

```text
Motion backend: SimpleGait (bench-remap verification, completed)
Hardware result: Front/Rear physical anti-phase **Hardware Verified**
```

The diagnostic override in
`RoboBeetleFirmware/Core/App/app_main.c` is:

```c
#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
/* Normal image defaults to the source-compatible CPG backend. */
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 1
#endif

/* Build the completed diagnostic image with:
 * -DMOTION_DEFAULT_GAIT_BACKEND_CPG=0
 */
```

With the explicit `=0` override, `app_main_init()` passes exactly
`simple_gait_generator_interface(&simple_gait_generator)` to
`motion_manager_init()`. The diagnostic actuator path was:

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

The CPG sources remain linked and the CPG unit, golden, safety/catch-up, and
period tests remain part of the Firmware host gate. The normal image now
defaults to `-DMOTION_DEFAULT_GAIT_BACKEND_CPG=1`; the explicit `=0` override
remains available for reproducing this SimpleGait baseline. Each build
registers exactly one generator with MotionManager. The separate
`ROBOBEETLE_CPG_TARGET_BENCHMARK` option is OFF by default; enabling that
diagnostic benchmark does not mix CPG output into a SimpleGait build.

## Preserved boundaries

- `simple_gait_generator.c` is unchanged. Forward remains front pair same
  phase, rear pair same phase, and front versus rear approximately pi apart;
  the installed SimpleGait Forward result is **Hardware Verified** with the
  front and rear groups physically anti-phase. Frequency, amplitude, turn
  scaling, and ASCEND/DESCEND bias are unchanged.
- The current FrontRight/FrontLeft/FrontAxis descriptor remap and calibration
  are unchanged.
- The installed shell command envelope is now `FrontRight 1140..1860 us` and
  `FrontLeft 1160..1900 us`; MotionManager applies the common front angle guard
  `-4500..+2800 cdeg`, while calibration retains its full endpoint tuples.
- CPG source mathematics, `theta_dot` semantics, double precision, target
  timing conclusions, Protocol V2, Motion safety ordering, and Qt UI are
  unchanged. The SimpleGait result freezes the front remap/sign boundary for
  future CPG debugging.
- This result does not prove physical CPG gait, Turn effectiveness, true water
  propulsion, FrontAxis hydrodynamic effect, or target timing.

## Required hardware checkpoint

After flashing this image, record actual directions rather than only “normal”:

1. Operate logical FrontRight only at `+10` and `-10` degrees, then logical
   FrontLeft only at `+10` and `-10` degrees; record actual physical side
   identity instead of inferring it from historical wiring.
2. Neutral all paddles, then set `FrontRight=+10`, `FrontLeft=+10`,
   `RearRight=-10`, `RearLeft=-10`; repeat with every sign reversed and record
   the physical front/rear directions.
3. Run SimpleGait Forward only after the static pose: the two front legs should
   be mutually same-phase, the two rear legs mutually same-phase, and the front
   group anti-phase with the rear group. The recorded result is **Hardware
   Verified** for this diagnostic baseline; the previous same-direction
   anomaly is **Closed for SimpleGait** and the shell cap was not its root cause
   at the `+/-10 degree` amplitude.
4. Run Turn Left/Right and confirm the reduced action is on the actual left or
   right side respectively.
5. Command FrontAxis `0`, `+10`, and `-10` degrees and record the mechanical
   directions. Ascend/Descend hydrodynamic meaning remains **Pending Water
   Verification**.

Do not change FrontRight/FrontLeft angle signs from this baseline. If a future
CPG Forward run is wrong while this SimpleGait baseline remains correct,
investigate CPG state, semantic adapter, backend selection, transition, or
actuator command path. Calibration and physical remap are frozen unless new
contradictory hardware evidence is recorded.
