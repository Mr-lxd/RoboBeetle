#pragma once

#include <array>
#include <cstdint>

namespace robobeetle::application {

enum class ServoId : std::uint8_t {
    FrontRight = 0, FrontLeft = 1, FrontAxis = 2, RearRight = 3, RearLeft = 4,
};

enum class MotionMode : std::uint8_t {
    Stop = 0, Forward = 1, Backward = 2, TurnLeft = 3,
    TurnRight = 4, Ascend = 5, Descend = 6,
};

enum class MotionAction : std::uint8_t { Stop = 0, Start = 1 };
enum class GaitBackend : std::uint8_t {
    SimpleGait = 0, CPG = 1, ExperimentalFlex = 2,
};
enum class FrontRearCoordination : std::uint8_t {
    SameDirection = 0, OppositeDirection = 1,
};
inline constexpr std::uint16_t SupportedServoMask = 0x001f;

enum class LeakState : std::uint8_t { Unknown = 0, Dry = 1, Wet = 2 };

struct LeakTelemetry {
    LeakState state{LeakState::Unknown};
};

struct ImuDiagnostics {
    std::uint32_t rx_byte_count{};
    std::uint32_t header_count{};
    std::uint32_t valid_frame_count{};
    std::uint32_t checksum_error_count{};
    std::uint32_t rx_buffer_overflow_count{};
    std::uint32_t rx_rearm_failure_count{};
    std::uint32_t uart_error_count{};
    std::uint32_t mag_frame_count{};
    std::uint32_t unsupported_frame_count{};
};

struct ImuTelemetry {
    std::uint8_t schema_version{};
    // Bits 0, 1, 2 mark acc, gyro, angle as valid respectively.
    std::uint8_t validity_flags{};
    std::array<std::int16_t, 3> acc_mg{};
    std::array<std::int16_t, 3> gyro_tenth_dps{};
    std::array<std::int16_t, 3> angle_centidegrees{};
    ImuDiagnostics diagnostics{};
};

struct DepthDiagnostics {
    std::uint32_t rx_byte_count{};
    std::uint32_t valid_line_count{};
    std::uint32_t parse_error_count{};
    std::uint32_t overlong_line_count{};
    std::uint32_t rx_buffer_overflow_count{};
    std::uint32_t hard_rearm_failure_count{};
    std::uint32_t uart_error_count{};
};

struct DepthTelemetry {
    std::uint8_t schema_version{};
    // Bits 0 and 1 mark depth and temperature as valid respectively.
    std::uint8_t validity_flags{};
    std::int32_t depth_mm{};
    std::int16_t temperature_centi_c{};
    // 0xffff is unknown/saturated; this value does not imply a stale policy.
    std::uint16_t sample_age_ms{0xffff};
    DepthDiagnostics diagnostics{};
};

enum class TelemetryMalformedReason {
    None, WrongLength, WrongSchema, ReservedFlags, InvalidDomain, InvalidValue,
};

} // namespace robobeetle::application
