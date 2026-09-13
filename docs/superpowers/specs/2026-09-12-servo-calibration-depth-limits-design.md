# Servo Calibration and FrontAxis Limits Design

**Status:** Approved for test-first implementation

**Scope:** This change is limited to the five-servo descriptor/calibration contract, servo software limits, related Firmware/Console tests, and the canonical documentation that describes those values. It is independent of the APC220 timing work.

## Baseline

The verified `origin/main` baseline is `a8491304f0d7c3d5c369671c108c6dae2b0a4a68` on branch `feature/servo-calibration-depth-limits`.

The current descriptors are:

- `FrontRight`: PWM `1050–1950 us`, angle `-45.00…+45.00 deg`.
- `FrontLeft`: PWM `1050–1950 us`, angle `-45.00…+45.00 deg`.
- `FrontAxis`/user-facing `Depth`: PWM `500–2500 us`, neutral `1500 us`, angle disabled, calibration pending.
- `RearRight` and `RearLeft`: PWM `1020–2020 us`, electrical calibration `520/1520/2520 us`, angle `-45.00…+45.00 deg`.

The repository contains no frozen FrontAxis angle sign convention. The user-provided convention therefore applies without reversal: face down is `-90 deg`, vertical is `0 deg`, and face up is `+90 deg`.

## Frozen contract

### FrontAxis / Depth actuator

The Firmware and Console descriptor tables will both contain:

| Field | Value |
|---|---:|
| electrical/calibration minimum pulse | `1060 us` |
| neutral pulse | `1745 us` |
| electrical/calibration maximum pulse | `2430 us` |
| calibration minimum angle | `-9000 cdeg` |
| calibration maximum angle | `+9000 cdeg` |
| command minimum pulse | `1060 us` |
| command maximum pulse | `2430 us` |
| command minimum angle | `-9000 cdeg` |
| command maximum angle | `+9000 cdeg` |
| angle supported | `true` |
| calibration pending | `false` in Console |

The physical bench interpretation is `1060 us = -90 deg` (front paddle face down), `1745 us = 0 deg` (vertical paddling), and `2430 us = +90 deg` (front paddle face up), for a measured 180-degree mechanical sweep. These are bench actuator calibration facts, not ROVMAKER depth-sensor calibration, hydrodynamic optimization, installed trim, or autonomous depth-control calibration.

The existing `servo_calibration_angle_to_pulse()` interpolation remains the only mapping implementation. Its current C integer-division semantics are preserved; with equal 685-us spans the expected intermediate values are `-45 deg -> 1403 us` and `+45 deg -> 2087 us`. Endpoint and neutral mappings must be exact.

Enable and Neutral already consume the descriptor calibration neutral pulse, so the updated FrontAxis behavior must initialize and neutralize at `1745 us` without a separate special case.

### RearRight / RearLeft

Only the PWM command exploration window changes:

```text
old: 1020–2020 us
new: 820–2220 us
```

The new range is temporary calibration exploration only. It is not a `-45/+45` mapping and must not change either servo's existing `520/1520/2520 us` calibration fields or `-4500…+4500 cdeg` angle command range. Final rear mechanical endpoints remain pending.

`FrontRight` and `FrontLeft` descriptors remain unchanged.

## Implementation boundaries

Firmware and Console maintain independent descriptor tables and will be updated together. Existing descriptor-driven Console range/angle behavior will be used; no new widgets, labels, dialogs, or layout code will be introduced. The Console's existing user-facing `Depth` label and internal `FrontAxis` identifier remain unchanged.

No changes are allowed to APC/RF configuration or timing, `LinkProfile`, USART, Protocol V2 framing or payloads, Safety heartbeat, Leak, JY901S, Raspberry Pi, ROS2, or ROVMAKER depth-sensor communication.

## Test design

Tests are written before production table changes and must first fail against the baseline. Firmware coverage will assert:

- FrontAxis descriptor parity, `angle_supported`, and all frozen values.
- PWM boundaries `1059/reject`, `1060/accept`, `1745/accept`, `2430/accept`, `2431/reject`.
- Angle boundaries `-9001/reject`, `-9000/accept`, `0/accept`, `+9000/accept`, `+9001/reject`.
- Exact endpoint/neutral mapping and the existing-rounding `-4500/+4500` values.
- FrontAxis Enable and Neutral write `1745 us`.
- Rear boundaries `819/reject`, `820/accept`, `2220/accept`, `2221/reject`.
- Existing FrontRight/FrontLeft boundaries remain unchanged.

Console coverage will assert the same descriptor contract and parity, that descriptor-driven PWM/angle command generation uses the new FrontAxis bounds, that FrontAxis is no longer calibration-pending, and that rear angle semantics and front-servo values remain unchanged. Existing UI structure tests remain the guard against layout changes.

All current Firmware host regressions remain in the gate, including Leak, IMU, Depth, transport, scheduler, and app integration tests. Console CTest remains a separate gate. Missing Qt6 in the current environment is reported as `Console configure/build: Pending`; it is never converted into a pass claim.

## Evidence boundaries

User-provided `1060/1745/2430` measurements may be recorded as `Bench Hardware Calibrated`. Until the resulting Firmware image is built, programmed, and exercised on hardware, the new FrontAxis angle behavior, neutral behavior, and software limit enforcement remain `Pending Hardware Verification`. Rear final `+/-45` PWM calibration also remains `Pending Hardware Calibration`.

The final change will be reviewed for direct standard-header use in touched C/C++ translation units, `git diff --check`, warnings, and absence of APC-related paths/files. The branch will be pushed and one PR opened against `main`; it will not be merged. External GitHub Review remains Pending.
