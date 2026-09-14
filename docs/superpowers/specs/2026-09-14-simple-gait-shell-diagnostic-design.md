# SimpleGait Shell-Limit Diagnostic Design — 2026-09-14

## Approved scope

This checkpoint records the latest bench diagnosis for the existing PR #15
branch `feature/cpg-gait-core`. `SimpleGait` remains the active bench backend
and `CPG` remains compiled but paused. The current front calibration direction
is accepted and must not be inverted again:

| Logical actuator | Calibration `-45 / 0 / +45 deg` | Physical meaning of logical `+angle` |
| --- | --- | --- |
| FrontRight | `1140 / 1580 / 2020 us` | increasing pulse, physical backward stroke |
| FrontLeft | `1900 / 1450 / 1000 us` | decreasing pulse, physical backward stroke |

The calibration tuples are the complete bench relationship. They are distinct
from the installed operational envelope and from raw PWM command validation.
No `-1` is added to `SimpleGait` or `CPGGaitGenerator`, and no logical ID or
timer/channel remap is inferred from the unresolved hardware side check.

## Installed operational contract

The measured shell limits are:

- physical FrontRight: backward is increasing pulse, safe maximum `1860 us`;
- physical FrontLeft: backward is decreasing pulse, safe minimum `1160 us`;
- neutrals remain FrontRight `1580 us` and FrontLeft `1450 us`.

The production raw PWM command bounds are ascending numeric bounds:

| Logical actuator | Raw command range | Calibration endpoints retained |
| --- | --- | --- |
| FrontRight | `1140..1860 us` | `1140 / 1580 / 2020 us` |
| FrontLeft | `1160..1900 us` | `1900 / 1450 / 1000 us` |

The right shell limit maps to approximately `+28.6 deg` and the left shell
limit to approximately `+29.0 deg` under the retained calibration. Motion uses
the conservative common body-frame operational angle envelope
`-4500..+2800 cdeg` for both front paddles. The rear common Motion envelope
remains `-3000..+4500 cdeg`. Calibration angle ranges remain `-4500..+4500
cdeg`; the `+2800 cdeg` cap is not a calibration edit.

The common guard is applied by `MotionManager` after any generator samples and
before `ServoService` calibration/PWM conversion. Neither gait generator owns
the installed shell limit. The guard also runs during the graceful stop path.

## Deterministic production-pipeline regression

The regression uses the real `SimpleGaitGenerator`, `MotionManager`,
`ServoService`, `ServoCalibration`, and descriptor mapping. It does not inject a
precomputed logical target. At `phi = pi/2` with the current `A=1000 cdeg`
profile, the production path must produce logical targets

```text
FrontRight = +1000 cdeg
FrontLeft  = +1000 cdeg
RearRight  = -1000 cdeg
RearLeft   = -1000 cdeg
```

and final pulses with these direction relationships:

```text
FrontRight: pulse > 1580 us  -> physical backward
FrontLeft:  pulse < 1450 us  -> physical backward
RearRight:  pulse < 1570 us  -> physical forward/recovery
RearLeft:   pulse > 1450 us  -> physical forward/recovery
```

At the half-cycle, every sign and corresponding pulse direction must reverse.
This proves the logical-to-PWM semantic contract without claiming that the
current automatic Forward hardware symptom is solved.

## Hardware evidence boundary

The observed SimpleGait Forward physical same-direction anomaly remains open;
the current `+/-10 deg` amplitude is below the new `+28 deg` cap and therefore
does not identify shell interference as its root cause. The next hardware
checkpoint is the static anti-phase pose after this push:

```text
Forward pose:  FrontRight=+10, FrontLeft=+10, RearRight=-10, RearLeft=-10
Reverse pose:  FrontRight=-10, FrontLeft=-10, RearRight=+10, RearLeft=+10
```

Qt FrontRight and FrontLeft single-servo checks must first record actual
physical side identity. The intended logical FrontRight/FrontLeft driver
bindings remain documented, but explicit physical side confirmation is
**Pending**. Case A (static anti-phase correct, automatic Forward wrong) routes
the next investigation to Motion runtime/backend/transition/actuator path;
Case B routes it to logical ID/channel or rear sign evidence. Calibration must
not be changed based on inference.

FrontAxis remains unchanged: `+FrontAxis` tilts the front portion downward and
`-FrontAxis` tilts it upward (**Bench Mechanical Verified**); hydrodynamic
effect remains **Pending Water Verification**. FrontAxis is not a fifth CPG
oscillator. Backward remains pending/disabled. Closed-loop CPG, IMU/depth
feedback, ROS2, and Protocol V2 changes are out of scope.

## Verification and target-type policy

Firmware descriptor/calibration, raw PWM, Motion guard, full-pipeline
SimpleGait, rear guard, ServoService, Safety, Protocol, all host regressions,
Qt build/CTest, header self-sufficiency, and `git diff --check` are required.
The existing safety-before-catch-up contract must remain intact: if foreground
resumes after the heartbeat/liveness deadline, the safety decision occurs
before any catch-up actuator command and Motion does not auto-resume.

Production CPG numeric types remain `double` for source compatibility. This
checkpoint does not change them to `float`. ARM FLASH/RAM deltas and timing
evidence are reported only when the ARM toolchain and target measurement path
are available; host results do not substitute for target performance evidence.
