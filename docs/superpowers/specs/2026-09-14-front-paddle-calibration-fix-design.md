# Front Paddle Calibration Sign Fix Design — 2026-09-14

## Approved scope

The SimpleGait bench result establishes that the 180-degree front assembly
installation inverted both front paddle logical angle coordinates in addition
to swapping the physical left/right channels. This change fixes that coordinate
sign only in the logical ServoCalibration descriptor layer. It does not change
gait math, phase semantics, MotionManager, Protocol V2, UI labels/layout, or
the active SimpleGait bench backend.

## Frozen contract

| Logical actuator | ID | Physical output | `-45 / 0 / +45 deg` calibration | Raw numeric bounds |
| --- | ---: | --- | --- | --- |
| FrontRight | 0 | former FrontLeft, TIM3_CH2 / PA7 | `1140 / 1580 / 2020 us` | `1140..2020 us` |
| FrontLeft | 1 | former FrontRight, TIM3_CH1 / PA6 | `1900 / 1450 / 1000 us` | `1000..1900 us` |

`FrontAxis` remains TIM3_CH3 / PB0 with `-90 / 0 / +90 deg -> 2430 /
1745 / 1060 us` and raw bounds `1060..2430 us`. Rear calibration and the
rear Motion envelope remain unchanged. Protocol IDs remain FrontRight `0`,
FrontLeft `1`, FrontAxis `2`, RearRight `3`, and RearLeft `4`.

The existing signed-delta interpolation remains the sole angle-to-pulse
algorithm. No reverse flag and no extra `-1` in SimpleGait or CPG are allowed.

## Data flow and boundaries

```text
SimpleGait / CPG logical targets
  -> MotionManager common guard
  -> ServoService logical angle
  -> ServoCalibration signed-delta interpolation
  -> descriptor-selected timer/channel
```

The Firmware descriptor owns the physical timer/channel binding and calibration
tuple. The Qt descriptor mirrors logical calibration, neutral, and numeric
command ranges; it does not introduce physical channel metadata. The normal
bench build continues to select `SimpleGait` via
`MOTION_DEFAULT_GAIT_BACKEND_CPG = 0`.

## Verification contract

Firmware tests must catch the old front tuples before production changes, then
must pass with the new tuples at `-4500`, `-2250`, `0`, `+2250`, and `+4500`
cdeg. Binding tests must continue to prove FrontRight -> TIM3_CH2,
FrontLeft -> TIM3_CH1, and FrontAxis -> TIM3_CH3. Qt tests must prove matching
logical tuples and unchanged semantic IDs/display labels. Full Firmware and Qt
regressions, header self-sufficiency, `git diff --check`, and ARM-toolchain
availability are reported. Missing ARM tools mean no target FLASH/RAM/timing
claim.

Physical front-side/sign, Forward anti-phase, Turn, and desk-only FrontAxis
behavior remain pending re-verification on the newly flashed SimpleGait image.
Ascend/Descend hydrodynamic effect remains **Pending Water Verification**.
