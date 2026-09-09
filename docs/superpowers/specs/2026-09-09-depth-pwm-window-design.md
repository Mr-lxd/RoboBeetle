# Depth PWM Calibration Window Follow-up Design

## Scope

This PR #8 follow-up widens only the `FrontAxis`/Depth PWM-only command envelope from `1450–1550 μs` to `1200–1800 μs` in the existing independent Firmware and Qt descriptor tables. It does not change Protocol V2, timer/pin mapping, servo behavior, safety policy, angle mapping, or the other four servo descriptors.

## Range layers

The S3150D seller/electrical metadata remains `500/1500/2500 μs` (with the documented voltage, travel, and dead-band metadata). Those values describe electrical/absolute capability and are not a user command safety range. The current descriptor command envelope is `1200–1800 μs` in both host and Firmware. `1500 μs` remains a provisional bring-up center candidate, not a calibrated mechanical center or Hardware Verified Neutral.

## Runtime behavior

Qt already derives the FrontAxis PWM SpinBox/Slider range from `ServoDescriptor::commandMinPwmUs` and `commandMaxPwmUs`; no independent UI clamp is added. Firmware `servo_service_set_pwm()` already validates against the Firmware descriptor command fields. Both sides therefore accept inclusive 1200/1800 and reject 1199/1801. `angleSupported`/`angle_supported` remains false, so Set Angle stays rejected.

## Verification boundary

Existing Qt descriptor/controller tests and Firmware descriptor/service tests cover the new boundaries and assert the four other command envelopes are unchanged. Host tests establish software consistency only. The supplied physical observation at 1480/1500/1520 μs verifies direction, while full 1200–1800 travel, safe endpoints, practical center, and angle calibration remain pending unloaded hardware verification.

## Documentation boundary

The Console and Firmware READMEs, Protocol V2 descriptor section, canonical hardware handoff, and engineering lessons state the two range layers, the supplied Hardware Verified facts, the original RearRight actuator fault, the validation taxonomy, the DAP flash/power-cycle reminder, and the next unloaded step sequence. No historical audit chapter is deleted or rewritten as current behavior.
