# Depth PWM Calibration Window Follow-up Design

## Scope

This PR #8 follow-up widens only the `FrontAxis`/Depth PWM-only command envelope from the current `1000–2000 μs` window to `500–2500 μs` in the existing independent Firmware and Qt descriptor tables. It also replaces the five user-facing display-name literals with exact ASCII semantic labels (`FrontRight`, `FrontLeft`, `Depth`, `RearRight`, `RearLeft`) while retaining internal `FrontAxis` identifiers. It does not change Protocol V2, timer/pin mapping, servo behavior, safety policy, angle mapping, or the other four servo descriptors.

## Range layers

The S3150D seller/electrical metadata remains `500/1500/2500 μs` (with the documented voltage, travel, and dead-band metadata). Those values describe electrical/absolute capability metadata, not a final user command safety range. The current descriptor command envelope is `500–2500 μs` in both host and Firmware, as a provisional endpoint-exploration window pending hardware verification. `1500 μs` remains a provisional bring-up center candidate, not a calibrated mechanical center or Hardware Verified Neutral.

## Runtime behavior

Qt already derives the FrontAxis PWM SpinBox/Slider range from `ServoDescriptor::commandMinPwmUs` and `commandMaxPwmUs`; no independent UI clamp is added. Firmware `servo_service_set_pwm()` already validates against the Firmware descriptor command fields. Both sides therefore accept inclusive 500/2500 and reject 499/2501. `angleSupported`/`angle_supported` remains false, so Set Angle stays rejected. The panel display names are exact ASCII labels; the internal semantic identifier remains `FrontAxis`.

## Verification boundary

Existing Qt descriptor/controller tests and Firmware descriptor/service tests cover the new boundaries and assert the four other command envelopes are unchanged; Qt descriptor tests also assert exact ASCII display names. Host tests establish software consistency only. The supplied physical observation at 1480/1500/1520 μs verifies direction, while full 500–2500 travel, safe endpoints, practical center, and angle calibration remain pending unloaded hardware verification.

## Documentation boundary

The Console and Firmware READMEs, Protocol V2 descriptor section, canonical hardware handoff, and engineering lessons state the two range layers, the supplied Hardware Verified facts, the original RearRight actuator fault, the validation taxonomy, the DAP flash/power-cycle reminder, the ASCII UI labels, and the next unloaded step sequence for the `500–2500 μs` exploration window. No historical audit chapter is deleted or rewritten as current behavior.
