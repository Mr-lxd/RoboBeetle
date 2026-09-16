#pragma once

#include "robobeetle/application/robot_types.hpp"
#include "robobeetle/protocol/frame.hpp"

#include <optional>

namespace robobeetle::application {

enum class CodecStatus {
    Ok, InvalidMask, InvalidServoId, InvalidMotionMode,
    InvalidMotionAction, InvalidGaitBackend,
};

struct CodecResult {
    CodecStatus status;
    protocol::Bytes payload;
};

// Payloads only: the runtime owns framing, sequence allocation, and delivery.
// The mask payload is shared by ServoEnable, ServoDisable, and Neutral.
CodecResult encode_servo_mask(std::uint16_t mask);
CodecResult encode_servo_angle(ServoId id, std::int16_t angle_cdeg);
// Bring-up/maintenance interface; no calibration or angle conversion occurs here.
CodecResult encode_servo_pwm(ServoId id, std::uint16_t pulse_us);
// Wire-level codec accepts Backward; the production facade enforces qualification.
CodecResult encode_motion(MotionMode mode, MotionAction action);
CodecResult encode_gait_backend(GaitBackend backend);

// Exact Firmware wire validation, without freshness or physical-range policy.
// If supplied, reason is set to None on success and a specific cause on failure.
std::optional<LeakTelemetry> decode_leak(
    const protocol::Bytes &payload, TelemetryMalformedReason *reason = nullptr) noexcept;
std::optional<ImuTelemetry> decode_imu(
    const protocol::Bytes &payload, TelemetryMalformedReason *reason = nullptr) noexcept;
std::optional<DepthTelemetry> decode_depth(
    const protocol::Bytes &payload, TelemetryMalformedReason *reason = nullptr) noexcept;

} // namespace robobeetle::application
