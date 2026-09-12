# Four-Paddle Servo Calibration Closeout Design

**Status:** Approved for implementation in existing PR #13

**Scope:** Finalize the logical-angle calibration contract for `FrontRight`, `FrontLeft`, `RearRight`, and `RearLeft`. Keep `FrontAxis` unchanged. This change does not implement gait, CPG, motion commands, APC, USART, Protocol V2 changes, UI layout changes, or resource-source analysis.

## Contract

All four paddle servos use one robot-logical convention:

- `0 deg` is mechanical neutral.
- `+45 deg` is the backward paddle stroke / forward-propulsion direction.
- `-45 deg` is the opposite direction.

The calibration layer owns neutral-pulse differences, left/right PWM inversion, and conversion from logical angle to PWM. Future gait/CPG code is expected to provide logical joint angles rather than raw PWM; no gait or CPG code is part of this PR.

The Firmware and Console descriptors must express these same values:

| Servo | -45 deg | 0 deg | +45 deg | Numeric PWM command limits |
|---|---:|---:|---:|---:|
| `FrontRight` | 1000 us | 1450 us | 1900 us | 1000–1900 us |
| `FrontLeft` | 2020 us | 1580 us | 1140 us | 1140–2020 us |
| `FrontAxis` | — | 1745 us | — | 1060–2430 us; -90/+90 deg |
| `RearRight` | 1110 us | 1570 us | 2030 us | 1110–2030 us |
| `RearLeft` | 1940 us | 1450 us | 960 us | 960–1940 us |

For `FrontAxis`, the existing `-90 deg = 1060 us`, `0 deg = 1745 us`, and `+90 deg = 2430 us` contract remains unchanged.

## Abstraction and validation

`servo_calibration_angle_to_pulse()` remains the single interpolation path. Its signed pulse deltas already represent both increasing and decreasing pulse mappings; no reverse flag and no Servo-ID-specific branch are needed. Angle calibration endpoints are directional (`min_angle` pulse can exceed `max_angle` pulse), while raw PWM command validation uses separate ascending numeric bounds. Validators must not infer raw bounds by comparing calibration endpoint fields.

Enable and Neutral use each descriptor's neutral pulse. Angle validation is inclusive at -4500/+4500 for the four paddle servos and -9000/+9000 for `FrontAxis`; raw PWM validation is inclusive at each descriptor's numeric command bounds.

## Evidence and documentation

- `FrontRight`: 1450/1900 are Bench Measured; 1000 is Symmetry-Derived / User Accepted.
- `FrontLeft`: 1580/1140 are Bench Measured; 2020 is Symmetry-Derived / User Accepted.
- `RearRight`: 1110/1570/2030 are Bench Hardware Verified.
- `RearLeft`: 1940/1450/960 are Bench Hardware Verified.
- `FrontAxis`: retain PR #13's existing calibration evidence and status.

Documentation may describe the future logical-angle data flow as `Motion Command -> Gait/CPG Generator -> Logical Joint Target -> ServoService set_angle -> Servo Calibration -> PWM`, but must state that this PR implements none of those future gait/CPG components or controls.

## Verification

Firmware and Console tests cover exact endpoint mappings, representative `-2250 cdeg` and `+2250 cdeg` interpolation, mirrored direction, inclusive/exclusive raw PWM and angle limits, all five neutral values, descriptor parity, and unchanged `FrontAxis` behavior. The full existing Firmware regression set and Console CTest must remain green. Header self-sufficiency, warnings, `git diff --check`, and ARM Build availability are reported separately; unavailable target tools remain Pending.
