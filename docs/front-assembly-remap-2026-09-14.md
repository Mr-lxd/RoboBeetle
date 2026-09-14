# Front Assembly Installation Remap — 2026-09-14

## Scope

The front mechanism was installed with a 180-degree mechanical coordinate
change before CPG target-hardware verification:

- the former physical FrontLeft actuator is now on the robot's actual right
  front leg;
- the former physical FrontRight actuator is now on the robot's actual left
  front leg;
- the FrontAxis actuator was installed with its physical direction reversed.

This is a descriptor-layer coordinate remap, not a gait redesign. Logical
names, Protocol V2 IDs, masks, body-frame angle signs, CPG node semantics,
Turn Left/Right behavior, MotionMode values, and Qt labels/layout remain
unchanged. The binding is intentionally kept below the semantic adapter:

```text
Legacy Source-Compatible CPG v1
  -> semantic adapter
  -> LogicalJointTargets
  -> MotionManager common guard
  -> ServoService
  -> ServoCalibration
  -> logical-to-physical descriptor binding
```

## Current logical-to-physical contract

| Logical actuator | ID / mask | Physical binding | Logical calibration: negative / neutral / positive | Raw numeric command bounds |
| --- | --- | --- | --- | --- |
| FrontRight | `0` / `0x0001` | former FrontLeft, TIM3_CH2 / PA7 | `-45/0/+45 cdeg -> 2020/1580/1140 us` | `1140..2020 us` |
| FrontLeft | `1` / `0x0002` | former FrontRight, TIM3_CH1 / PA6 | `-45/0/+45 cdeg -> 1000/1450/1900 us` | `1000..1900 us` |
| FrontAxis (`Depth`) | `2` / `0x0004` | same physical channel, TIM3_CH3 / PB0 | `-90/0/+90 cdeg -> 2430/1745/1060 us` | `1060..2430 us` |
| RearRight | `3` / `0x0008` | unchanged, TIM4_CH1 / PD12 | `-45/0/+45 cdeg -> 1110/1570/2030 us` | `1110..2030 us` |
| RearLeft | `4` / `0x0010` | unchanged, TIM4_CH2 / PD13 | `-45/0/+45 cdeg -> 1940/1450/960 us` | `960..1940 us` |

The raw bounds are ascending numeric validation limits. They must not be
reversed merely because a logical calibration endpoint decreases with angle.
The existing signed-delta interpolation in
`RoboBeetleFirmware/Core/Servo/servo_calibration.c` remains authoritative;
there is no separate reverse flag. For FrontAxis, its raw envelope and
neutral remain `1060..2430 us` and `1745 us`, while only the logical endpoint
direction is reversed. The signed-delta midpoint results are `-45 degrees ->
2087 us` and `+45 degrees -> 1403 us` under the existing integer truncation.

The former pre-remap logical descriptor values are retained as historical
baseline, not deleted:

```text
FrontRight:  TIM3_CH1 / PA6,  -45/0/+45 -> 1000/1450/1900 us, raw 1000..1900
FrontLeft:   TIM3_CH2 / PA7,  -45/0/+45 -> 2020/1580/1140 us, raw 1140..2020
FrontAxis:   TIM3_CH3 / PB0,  -90/0/+90 -> 1060/1745/2430 us, raw 1060..2430
```

## Semantic invariants

- `SERVO_ID_FRONT_RIGHT` always means the robot's actual right front leg;
  `SERVO_ID_FRONT_LEFT` always means the actual left front leg.
- Forward keeps the same logical front pair and rear pair semantics.
- Turn modulation continues to use logical right/left amplitudes; no CPG node
  swap or second swap is allowed.
- ASCEND/DESCEND keep their existing logical FrontAxis bias signs. Physical
  water direction remains **Pending Water Verification**.
- `FrontAxis` is a bias actuator, not a fifth CPG oscillator.
- The rear Motion operational guard remains `-3000..+4500 cdeg` (`-30..+45
  degrees`) and does not move into calibration.
- `SimpleGaitGenerator` remains available. Backward remains pending/disabled.
- Closed-loop CPG, IMU/depth feedback, ROS2, and Protocol V2 changes are out
  of scope.

## Verification status

Firmware descriptor, calibration, driver-binding, ServoService, Protocol
Dispatcher, CPG/Motion/Safety host regressions, and the matching Qt descriptor
tests pass. The front assembly remap is therefore **Software Verified**.

The following evidence is deliberately not inferred from host tests:

| Evidence | Status |
| --- | --- |
| Logical descriptor/table parity | **Software Verified** |
| Logical FrontRight -> physical TIM3_CH2 / actual right front leg | **Pending Hardware Verification** |
| Logical FrontLeft -> physical TIM3_CH1 / actual left front leg | **Pending Hardware Verification** |
| FrontAxis logical sign after physical reversal | **Pending Hardware Verification** |
| STM32F407 ARM build/program verify | **Pending** under the existing target gate |
| CPG FLASH/RAM and DWT timing evidence | **Pending**; production core remains `double` |
| Physical Forward/Turn/Ascend/Descend behavior | **Pending** |
| Water/hydrodynamic behavior | **Pending Water Verification** |

The CPG long-run host result remains a nominal-period measurement, not a
claim that `2.0 s` equals an actual `0.5 Hz` oscillator period. The current
recorded production-profile measurement is `nominal_period_s=2.0`,
`measured_period_s=1.504827586`, `measured_frequency_hz=0.664527956`, and
ratio `0.752413793`; this is governed by the source-compatible `nu_i`
dependence and is unrelated to the mechanical remap.

## Required post-push hardware checklist

| Test | Procedure | Acceptance boundary |
| --- | --- | --- |
| A — FrontRight | Enable only logical FrontRight; Set Angle `+10` and `-10` | The robot's actual right front leg moves; no left-front motion |
| B — FrontLeft | Enable only logical FrontLeft; Set Angle `+10` and `-10` | The robot's actual left front leg moves; no right-front motion |
| C — FrontAxis | Command `0`, `+10`, and `-10` degrees | Physical direction agrees with the logical sign after the reversal |
| D — Forward CPG | Run Forward and inspect front sides, then Turn Left/Right | Front pair is on the correct physical sides; no side swap in turns |
| E — Ascend/Descend | Desk-only mechanical test of both logical signs | Mechanism direction only; hydrodynamic effect remains **Pending Water Verification** |

Do not label any row above **Hardware Verified** until the current image is
built, programmed, and the stated physical behavior is observed.
