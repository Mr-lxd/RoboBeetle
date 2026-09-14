# Front Assembly Installation Remap — 2026-09-14

## Scope

The front mechanism was installed with a 180-degree mechanical coordinate
change before CPG target-hardware verification:

- the former physical FrontLeft actuator is now on the robot's actual right
  front leg;
- the former physical FrontRight actuator is now on the robot's actual left
  front leg;
  - the FrontAxis actuator retains the current TIM3_CH3 binding and logical
    calibration direction; a desk observation found `ASCEND (+10 degrees)`
    tilts the front portion downward and `DESCEND (-10 degrees)` tilts it
    upward. This is **Bench Mechanical Verified** only.

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
| FrontRight | `0` / `0x0001` | intended physical right front, former FrontLeft, TIM3_CH2 / PA7 | `-45/0/+45 cdeg -> 1140/1580/2020 us` | `1140..1860 us` |
| FrontLeft | `1` / `0x0002` | intended physical left front, former FrontRight, TIM3_CH1 / PA6 | `-45/0/+45 cdeg -> 1900/1450/1000 us` | `1160..1900 us` |
| FrontAxis (`Depth`) | `2` / `0x0004` | same physical channel, TIM3_CH3 / PB0 | `-90/0/+90 cdeg -> 2430/1745/1060 us` | `1060..2430 us` |
| RearRight | `3` / `0x0008` | unchanged, TIM4_CH1 / PD12 | `-45/0/+45 cdeg -> 1110/1570/2030 us` | `1110..2030 us` |
| RearLeft | `4` / `0x0010` | unchanged, TIM4_CH2 / PD13 | `-45/0/+45 cdeg -> 1940/1450/960 us` | `960..1940 us` |

The raw bounds are ascending numeric validation limits. They must not be
reversed merely because a logical calibration endpoint decreases with angle.
The shell measurements that define the new installed command envelope are
physical FrontRight increasing-pulse backward motion capped at `1860 us` and
physical FrontLeft decreasing-pulse backward motion capped at `1160 us`.
The existing signed-delta interpolation in
`RoboBeetleFirmware/Core/Servo/servo_calibration.c` remains authoritative;
there is no separate reverse flag. For FrontAxis, its raw envelope and
neutral remain `1060..2430 us` and `1745 us`; FrontAxis is unchanged by this
calibration fix. Its signed-delta midpoint results remain `-45 degrees ->
2087 us` and `+45 degrees -> 1403 us` under the existing integer truncation.

The former pre-remap logical descriptor values are retained as historical
baseline, not deleted:

```text
FrontRight:  TIM3_CH1 / PA6,  -45/0/+45 -> 1000/1450/1900 us, raw 1000..1900
FrontLeft:   TIM3_CH2 / PA7,  -45/0/+45 -> 2020/1580/1140 us, raw 1140..2020
FrontAxis:   TIM3_CH3 / PB0,  -90/0/+90 -> 1060/1745/2430 us, raw 1060..2430
```

## Calibration evidence chain

### Stage 1 — physical side swap only

The first remap changed only the logical side-to-channel binding: logical
FrontRight moved to TIM3_CH2 / the former physical FrontLeft actuator, and
logical FrontLeft moved to TIM3_CH1 / the former physical FrontRight actuator.
SimpleGait was deliberately kept as the known logical anti-phase baseline.
The real bench result was that the physical front and rear paddles still moved
in the same direction. That result showed that channel swapping alone did not
correct the rotated front assembly: both front paddle angle coordinates were
also inverted.

### Stage 2 — final front angle-sign correction

This calibration-fix commit keeps the Stage 1 physical channel swap and
reverses only the two front logical endpoint pairs:

```text
FrontRight: TIM3_CH2, -45/0/+45 -> 1140/1580/2020 us, raw 1140..1860
FrontLeft:  TIM3_CH1, -45/0/+45 -> 1900/1450/1000 us, raw 1160..1900
```

The change is intentionally below MotionManager. `SimpleGaitGenerator`
continues to emit the same logical front/rear phase relationship, and CPG
production math is unchanged. The existing signed-delta calibration function
continues to map logical angles to pulses; no extra gait sign is applied.

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
- The front Motion operational guard is `-4500..+2800 cdeg` (`-45..+28
  degrees`) for both front paddles. This is an installed shell envelope, not a
  change to either `-4500..+4500 cdeg` calibration range.
- Raw manual/Qt command bounds are `FrontRight 1140..1860 us` and
  `FrontLeft 1160..1900 us`; the complete calibration endpoints remain
  `1140/1580/2020 us` and `1900/1450/1000 us` respectively.
- `SimpleGaitGenerator` remains available. Backward remains pending/disabled.
- Closed-loop CPG, IMU/depth feedback, ROS2, and Protocol V2 changes are out
  of scope.

## Verification status

Firmware descriptor, calibration, driver-binding, ServoService, Protocol
Dispatcher, CPG/Motion/Safety host regressions, and the matching Qt descriptor
tests pass. The final logical calibration contract is therefore **Software
Verified**. This does not replace the required post-fix hardware retest.

The following evidence is deliberately not inferred from host tests:

| Evidence | Status |
| --- | --- |
| Logical descriptor/table parity | **Software Verified** |
| Logical FrontRight -> physical TIM3_CH2 / actual right front leg | **Pending explicit hardware side confirmation** |
| Logical FrontLeft -> physical TIM3_CH1 / actual left front leg | **Pending explicit hardware side confirmation** |
| FrontAxis desk direction: `+10` downward / `-10` upward | **Bench Mechanical Verified** |
| FrontAxis post-fix end-to-end retest | **Pending Hardware Verification** |
| STM32F407 ARM build/program verify | **Pending** under the existing target gate |
| CPG FLASH/RAM and DWT timing evidence | **Pending**; production core remains `double` |
| Physical FrontRight/FrontLeft sign and side mapping | **Pending Hardware Verification** |
| Front shell operational limits: FR `1860 us`, FL `1160 us`; common Motion cap `+2800 cdeg` | **Software Verified / Bench Measured input** |
| Physical Forward anti-phase and Turn Left/Right side identity | **Pending Hardware Verification** |
| Water/hydrodynamic behavior | **Pending Water Verification** |

The CPG long-run host result remains a nominal-period measurement, not a
claim that `T=2.0 s` equals an actual `0.5 Hz` oscillator period. The current
recorded production-profile measurement is `nominal_period_s=2.0`,
`measured_period_s=1.504827586`, `measured_frequency_hz=0.664527956`, and
ratio `0.752413793`; this is governed by the source-compatible `nu_i`
dependence and is unrelated to the mechanical remap.

The current SimpleGait amplitude is only `+/-10 degrees`, well below the
`+28 degree` front cap. Therefore the automatic Forward same-direction
anomaly is still open and is not explained or claimed solved by this shell
limit. The static anti-phase pose below is required before routing the next
root-cause investigation.

## Required post-push hardware checklist

| Test | Procedure | Acceptance boundary |
| --- | --- | --- |
| A — physical identity | Operate only Qt logical FrontRight at `+10` and `-10`, then only Qt logical FrontLeft at `+10` and `-10`; record the actual physical side for each | Explicit table `logical FrontRight -> physical ?` and `logical FrontLeft -> physical ?`; no inference from historical wiring |
| B — static anti-phase | Neutral all paddles, then set `FrontRight=+10`, `FrontLeft=+10`, `RearRight=-10`, `RearLeft=-10`; repeat with every sign reversed | First pose: front pair backward/rear pair forward; second pose: front pair forward/rear pair backward |
| C — automatic Forward | Run SimpleGait Forward only after A/B | Front pair same-phase, rear pair same-phase, front/rear physical anti-phase; current same-direction symptom remains **Pending** until observed |
| D — Turn Left/Right | Run both turn modes after the static Forward check | Actual left/right side identity is preserved; no channel swap |
| E — FrontAxis / Ascend / Descend | Command FrontAxis `0`, `+10`, `-10`; run desk-only Ascend/Descend | `+FrontAxis` tilts the front portion downward and `-FrontAxis` upward; hydrodynamic effect remains **Pending Water Verification** |

Do not label any row above **Hardware Verified** until the current image is
built, programmed, and the stated physical behavior is observed.

If static pose B is physically anti-phase but automatic Forward remains
same-direction, keep calibration unchanged and investigate Motion runtime,
backend selection, transition, or actuator command path. If static pose B is
also physically same-direction, investigate logical ID/physical channel
identity or rear logical sign from the recorded measurements; do not choose a
new mapping from assumption alone.
