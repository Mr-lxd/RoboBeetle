#include "robobeetle/application/robot_codec.hpp"

#include <cstddef>

namespace robobeetle::application {
namespace {

std::uint16_t read_u16(const protocol::Bytes &payload, std::size_t offset) noexcept
{
    return static_cast<std::uint16_t>(payload[offset]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(payload[offset + 1U]) << 8U);
}

std::uint32_t read_u32(const protocol::Bytes &payload, std::size_t offset) noexcept
{
    return static_cast<std::uint32_t>(payload[offset]) |
           (static_cast<std::uint32_t>(payload[offset + 1U]) << 8U) |
           (static_cast<std::uint32_t>(payload[offset + 2U]) << 16U) |
           (static_cast<std::uint32_t>(payload[offset + 3U]) << 24U);
}

std::int16_t read_i16(const protocol::Bytes &payload, std::size_t offset) noexcept
{
    const auto raw = read_u16(payload, offset);
    // Widen before subtracting to avoid implementation-defined unsigned-to-signed casts.
    const auto value = raw < 0x8000U ? static_cast<std::int32_t>(raw)
                                   : static_cast<std::int32_t>(raw) - 0x10000;
    return static_cast<std::int16_t>(value);
}

std::int32_t read_i32(const protocol::Bytes &payload, std::size_t offset) noexcept
{
    const auto raw = read_u32(payload, offset);
    const auto value = raw < 0x80000000U ? static_cast<std::int64_t>(raw)
                                       : static_cast<std::int64_t>(raw) - 0x100000000LL;
    return static_cast<std::int32_t>(value);
}

void set_reason(TelemetryMalformedReason *out, TelemetryMalformedReason value) noexcept
{
    if (out != nullptr) {
        *out = value;
    }
}

template <typename T>
std::optional<T> malformed(
    TelemetryMalformedReason *out, TelemetryMalformedReason value) noexcept
{
    set_reason(out, value);
    return std::nullopt;
}

CodecResult encode_servo_value(ServoId id, std::uint16_t value)
{
    const auto wire_id = static_cast<std::uint8_t>(id);
    if (wire_id > static_cast<std::uint8_t>(ServoId::RearLeft)) {
        return {CodecStatus::InvalidServoId, {}};
    }
    return {CodecStatus::Ok, {1U, wire_id, static_cast<std::uint8_t>(value & 0xffU),
                             static_cast<std::uint8_t>(value >> 8U)}};
}

} // namespace

CodecResult encode_servo_mask(std::uint16_t mask)
{
    if (mask == 0U || (mask & ~SupportedServoMask) != 0U) {
        return {CodecStatus::InvalidMask, {}};
    }
    return {CodecStatus::Ok, {static_cast<std::uint8_t>(mask & 0xffU),
                             static_cast<std::uint8_t>(mask >> 8U)}};
}

CodecResult encode_servo_angle(ServoId id, std::int16_t angle_cdeg)
{
    return encode_servo_value(id, static_cast<std::uint16_t>(angle_cdeg));
}

CodecResult encode_servo_pwm(ServoId id, std::uint16_t pulse_us)
{
    return encode_servo_value(id, pulse_us);
}

CodecResult encode_motion(MotionMode mode, MotionAction action)
{
    const auto wire_mode = static_cast<std::uint8_t>(mode);
    const auto wire_action = static_cast<std::uint8_t>(action);
    if (wire_action > static_cast<std::uint8_t>(MotionAction::Start)) {
        return {CodecStatus::InvalidMotionAction, {}};
    }
    if (wire_mode > static_cast<std::uint8_t>(MotionMode::Descend) ||
        ((mode == MotionMode::Stop) != (action == MotionAction::Stop))) {
        return {CodecStatus::InvalidMotionMode, {}};
    }
    return {CodecStatus::Ok, {1U, wire_mode, wire_action}};
}

CodecResult encode_gait_backend(GaitBackend backend)
{
    const auto value = static_cast<std::uint8_t>(backend);
    if (value > static_cast<std::uint8_t>(GaitBackend::CPG)) {
        return {CodecStatus::InvalidGaitBackend, {}};
    }
    return {CodecStatus::Ok, {value}};
}

std::optional<LeakTelemetry> decode_leak(
    const protocol::Bytes &payload, TelemetryMalformedReason *reason) noexcept
{
    if (payload.size() != 1U) {
        return malformed<LeakTelemetry>(reason, TelemetryMalformedReason::WrongLength);
    }
    if (payload[0] > static_cast<std::uint8_t>(LeakState::Wet)) {
        return malformed<LeakTelemetry>(reason, TelemetryMalformedReason::InvalidValue);
    }
    set_reason(reason, TelemetryMalformedReason::None);
    return LeakTelemetry{static_cast<LeakState>(payload[0])};
}

std::optional<ImuTelemetry> decode_imu(
    const protocol::Bytes &payload, TelemetryMalformedReason *reason) noexcept
{
    // Firmware Core/Sensors/jy901s_telemetry.c defines these offsets and units.
    if (payload.size() != 56U) {
        return malformed<ImuTelemetry>(reason, TelemetryMalformedReason::WrongLength);
    }
    if (payload[0] != 1U) {
        return malformed<ImuTelemetry>(reason, TelemetryMalformedReason::WrongSchema);
    }
    if ((payload[1] & 0xf8U) != 0U) {
        return malformed<ImuTelemetry>(reason, TelemetryMalformedReason::ReservedFlags);
    }
    for (std::size_t domain = 0U; domain < 3U; ++domain) {
        if ((payload[1] & (1U << domain)) == 0U) {
            for (std::size_t byte = 0U; byte < 6U; ++byte) {
                if (payload[2U + domain * 6U + byte] != 0U) {
                    return malformed<ImuTelemetry>(reason, TelemetryMalformedReason::InvalidDomain);
                }
            }
        }
    }
    ImuTelemetry result{};
    result.schema_version = payload[0];
    result.validity_flags = payload[1];
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        result.acc_mg[axis] = read_i16(payload, 2U + axis * 2U);
        result.gyro_tenth_dps[axis] = read_i16(payload, 8U + axis * 2U);
        result.angle_centidegrees[axis] = read_i16(payload, 14U + axis * 2U);
    }
    result.diagnostics.rx_byte_count = read_u32(payload, 20U);
    result.diagnostics.header_count = read_u32(payload, 24U);
    result.diagnostics.valid_frame_count = read_u32(payload, 28U);
    result.diagnostics.checksum_error_count = read_u32(payload, 32U);
    result.diagnostics.rx_buffer_overflow_count = read_u32(payload, 36U);
    result.diagnostics.rx_rearm_failure_count = read_u32(payload, 40U);
    result.diagnostics.uart_error_count = read_u32(payload, 44U);
    result.diagnostics.mag_frame_count = read_u32(payload, 48U);
    result.diagnostics.unsupported_frame_count = read_u32(payload, 52U);
    set_reason(reason, TelemetryMalformedReason::None);
    return result;
}

std::optional<DepthTelemetry> decode_depth(
    const protocol::Bytes &payload, TelemetryMalformedReason *reason) noexcept
{
    // Firmware Core/Communication/depth_telemetry.c defines the validation rules.
    if (payload.size() != 38U) {
        return malformed<DepthTelemetry>(reason, TelemetryMalformedReason::WrongLength);
    }
    if (payload[0] != 1U) {
        return malformed<DepthTelemetry>(reason, TelemetryMalformedReason::WrongSchema);
    }
    if ((payload[1] & 0xfcU) != 0U) {
        return malformed<DepthTelemetry>(reason, TelemetryMalformedReason::ReservedFlags);
    }
    if (((payload[1] & 0x01U) == 0U &&
         (read_u32(payload, 2U) != 0U || read_u16(payload, 8U) != 0xffffU)) ||
        ((payload[1] & 0x02U) == 0U && read_u16(payload, 6U) != 0U)) {
        return malformed<DepthTelemetry>(reason, TelemetryMalformedReason::InvalidDomain);
    }
    DepthTelemetry result{};
    result.schema_version = payload[0];
    result.validity_flags = payload[1];
    result.depth_mm = read_i32(payload, 2U);
    result.temperature_centi_c = read_i16(payload, 6U);
    result.sample_age_ms = read_u16(payload, 8U);
    result.diagnostics.rx_byte_count = read_u32(payload, 10U);
    result.diagnostics.valid_line_count = read_u32(payload, 14U);
    result.diagnostics.parse_error_count = read_u32(payload, 18U);
    result.diagnostics.overlong_line_count = read_u32(payload, 22U);
    result.diagnostics.rx_buffer_overflow_count = read_u32(payload, 26U);
    result.diagnostics.hard_rearm_failure_count = read_u32(payload, 30U);
    result.diagnostics.uart_error_count = read_u32(payload, 34U);
    set_reason(reason, TelemetryMalformedReason::None);
    return result;
}

} // namespace robobeetle::application
