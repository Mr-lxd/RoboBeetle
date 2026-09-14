# SimpleGait Bench Image — Front-Assembly Remap Checkpoint

## Purpose

This is a temporary diagnostic bench configuration for the 2026-09-14 front
assembly remap. It isolates the already verified SimpleGait stimulus from the
source-compatible CPG dynamics before CPG target/hardware verification.

## Build evidence

```text
Motion backend: SimpleGait (bench-remap verification)
```

The normal source fallback in
`RoboBeetleFirmware/Core/App/app_main.c` is:

```c
#ifndef MOTION_DEFAULT_GAIT_BACKEND_CPG
/* Diagnostic bench image: isolate mechanical remap with SimpleGait. */
#define MOTION_DEFAULT_GAIT_BACKEND_CPG 0
#endif
```

With that default, `app_main_init()` passes exactly
`simple_gait_generator_interface(&simple_gait_generator)` to
`motion_manager_init()`. The actuator path is:

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
period tests remain part of the Firmware host gate. A later CPG verification
image can explicitly compile `-DMOTION_DEFAULT_GAIT_BACKEND_CPG=1`. The
separate `ROBOBEETLE_CPG_TARGET_BENCHMARK` option is OFF by default; enabling
that diagnostic benchmark does not make CPG an actuator generator.

## Preserved boundaries

- `simple_gait_generator.c` is unchanged. Forward remains front pair same
  phase, rear pair same phase, and front versus rear approximately pi apart;
  frequency, amplitude, turn scaling, and ASCEND/DESCEND bias are unchanged.
- The current FrontRight/FrontLeft/FrontAxis descriptor remap and calibration
  are unchanged.
- CPG source mathematics, `theta_dot` semantics, double precision, target
  timing conclusions, Protocol V2, Motion safety ordering, and Qt UI are
  unchanged.
- This image does not prove physical side binding, angle sign, FrontAxis
  direction, target timing, or hydrodynamic meaning.

## Required hardware checkpoint

After flashing this image, record actual directions rather than only “normal”:

1. Enable logical FrontRight only; command `+10` and `-10` degrees and confirm
   the robot's actual right-front leg and record both directions.
2. Enable logical FrontLeft only; command `+10` and `-10` degrees and confirm
   the robot's actual left-front leg and record both directions.
3. Run SimpleGait Forward: the two front legs should be mutually same-phase,
   the two rear legs mutually same-phase, and the front group anti-phase with
   the rear group.
4. Run Turn Left/Right and confirm the reduced action is on the actual left or
   right side respectively.
5. Command FrontAxis `0`, `+10`, and `-10` degrees and record the mechanical
   directions. Ascend/Descend hydrodynamic meaning remains **Pending Water
   Verification**.

Do not change FrontRight/FrontLeft angle signs from this image based on prior
CPG-only observations. Decide any second-stage calibration change only after
these SimpleGait measurements.
