#include "test_support.hpp"
#include "robobeetle/application/robot_codec.hpp"

#include <array>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace rbp2_test { int failures = 0; }

namespace {

using namespace robobeetle::application;
using rbp2_test::bytes;
using rbp2_test::expect;
using Bytes = std::vector<std::uint8_t>;

bool encoded_as(const CodecResult &result, const Bytes &payload)
{
    return result.status == CodecStatus::Ok && result.payload == payload;
}

void write_le16(Bytes &payload, std::size_t offset, std::uint16_t value)
{
    payload[offset] = static_cast<std::uint8_t>(value & 0xffU);
    payload[offset + 1U] = static_cast<std::uint8_t>(value >> 8U);
}

void write_le32(Bytes &payload, std::size_t offset, std::uint32_t value)
{
    payload[offset] = static_cast<std::uint8_t>(value & 0xffU);
    payload[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
    payload[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
    payload[offset + 3U] = static_cast<std::uint8_t>(value >> 24U);
}

void command_payloads_and_validation()
{
    expect(encoded_as(encode_servo_mask(0x001f), bytes({0x1f, 0x00})),
           "servo mask uses little endian bytes");
    expect(encode_servo_mask(0).status == CodecStatus::InvalidMask,
           "zero servo mask is rejected");
    expect(encode_servo_mask(0x8000).status == CodecStatus::InvalidMask,
           "unsupported servo mask bits are rejected");

    expect(encoded_as(encode_servo_angle(ServoId::FrontLeft, 0x1234),
               bytes({1, 1, 0x34, 0x12})),
           "positive servo angle is encoded little endian");
    expect(encoded_as(encode_servo_angle(ServoId::FrontRight, 0),
               bytes({1, 0, 0, 0})),
           "zero servo angle is encoded");
    expect(encoded_as(encode_servo_angle(ServoId::RearLeft, -2),
               bytes({1, 4, 0xfe, 0xff})),
           "negative servo angle is two complement little endian");
    expect(encoded_as(encode_servo_angle(ServoId::FrontAxis, INT16_MIN),
               bytes({1, 2, 0x00, 0x80})),
           "minimum int16 servo angle is encoded");
    expect(encoded_as(encode_servo_angle(ServoId::RearRight, INT16_MAX),
               bytes({1, 3, 0xff, 0x7f})),
           "maximum int16 servo angle is encoded");
    expect(encode_servo_angle(static_cast<ServoId>(5), 0).status ==
               CodecStatus::InvalidServoId,
           "invalid servo id is rejected");

    expect(encoded_as(encode_servo_pwm(ServoId::FrontRight, 2000),
               bytes({1, 0, 0xd0, 0x07})),
           "maintenance PWM is encoded little endian");
    expect(encode_servo_pwm(static_cast<ServoId>(5), 1500).status ==
               CodecStatus::InvalidServoId,
           "invalid maintenance PWM servo id is rejected");

    expect(encoded_as(encode_motion(MotionMode::Forward, MotionAction::Start),
               bytes({1, 1, 1})),
           "motion START payload is encoded");
    expect(encoded_as(encode_motion(MotionMode::Stop, MotionAction::Stop),
               bytes({1, 0, 0})),
           "motion STOP payload is encoded");
    expect(encode_motion(MotionMode::Stop, MotionAction::Start).status ==
               CodecStatus::InvalidMotionMode,
           "motion START with STOP mode is rejected");
    expect(encode_motion(MotionMode::Forward, MotionAction::Stop).status ==
               CodecStatus::InvalidMotionMode,
           "motion STOP with non-STOP mode is rejected");
    expect(encoded_as(encode_motion(MotionMode::Backward, MotionAction::Start),
               bytes({1, 2, 1})),
           "codec accepts wire-level BACKWARD START value");
    expect(encode_motion(static_cast<MotionMode>(7), MotionAction::Start).status ==
               CodecStatus::InvalidMotionMode,
           "invalid motion mode is rejected");
    expect(encode_motion(MotionMode::Forward, static_cast<MotionAction>(2)).status ==
               CodecStatus::InvalidMotionAction,
           "invalid motion action is rejected");

    expect(encoded_as(encode_gait_backend(GaitBackend::SimpleGait), bytes({0})),
           "SimpleGait uses wire value zero");
    expect(encoded_as(encode_gait_backend(GaitBackend::CPG), bytes({1})),
           "CPG uses wire value one");
    expect(encoded_as(encode_gait_backend(GaitBackend::ExperimentalFlex),
                      bytes({2})),
           "ExperimentalFlex uses wire value two");
    expect(encode_gait_backend(static_cast<GaitBackend>(3)).status ==
               CodecStatus::InvalidGaitBackend,
           "invalid gait backend is rejected");

    expect(encoded_as(encode_front_rear_coordination(
                          FrontRearCoordination::SameDirection),
                      bytes({0})),
           "SameDirection coordination uses one-byte value zero");
    expect(encoded_as(encode_front_rear_coordination(
                          FrontRearCoordination::OppositeDirection),
                      bytes({1})),
           "OppositeDirection coordination uses one-byte value one");
    expect(encode_front_rear_coordination(
               static_cast<FrontRearCoordination>(2)).status ==
               CodecStatus::InvalidFrontRearCoordination,
           "invalid FrontRear coordination is rejected");
}

void telemetry_decoders_match_firmware_layout()
{
    TelemetryMalformedReason reason = TelemetryMalformedReason::None;

    const auto leak_unknown = decode_leak(bytes({0}), &reason);
    expect(leak_unknown && leak_unknown->state == LeakState::Unknown,
           "LeakStatus unknown decodes");
    const auto leak_dry = decode_leak(bytes({1}), &reason);
    expect(leak_dry && leak_dry->state == LeakState::Dry,
           "LeakStatus dry decodes");
    const auto leak_wet = decode_leak(bytes({2}), &reason);
    expect(leak_wet && leak_wet->state == LeakState::Wet,
           "LeakStatus wet decodes");
    const auto leak_invalid = decode_leak(bytes({3}), &reason);
    expect(!leak_invalid && reason == TelemetryMalformedReason::InvalidValue,
           "invalid LeakStatus value is malformed");
    expect(!decode_leak(bytes({}), &reason) &&
               reason == TelemetryMalformedReason::WrongLength,
           "empty LeakStatus is malformed");
    expect(!decode_leak(bytes({0, 1}), &reason) &&
               reason == TelemetryMalformedReason::WrongLength,
           "long LeakStatus is malformed");
    expect(decode_leak(bytes({1}), &reason) &&
               reason == TelemetryMalformedReason::None,
           "successful decode clears a previous malformed reason");
    expect(decode_leak(bytes({2})).has_value() && !decode_leak(bytes({3})),
           "LeakStatus reason output is optional");

    Bytes imu(56U, 0U);
    imu[0] = 1U;
    imu[1] = 0x07U;
    write_le16(imu, 2U, static_cast<std::uint16_t>(1000));
    write_le16(imu, 4U, static_cast<std::uint16_t>(-2000));
    write_le16(imu, 6U, static_cast<std::uint16_t>(32767));
    write_le16(imu, 8U, static_cast<std::uint16_t>(-123));
    write_le16(imu, 10U, static_cast<std::uint16_t>(456));
    write_le16(imu, 12U, static_cast<std::uint16_t>(-32768));
    write_le16(imu, 14U, static_cast<std::uint16_t>(-1));
    write_le16(imu, 16U, static_cast<std::uint16_t>(0));
    write_le16(imu, 18U, static_cast<std::uint16_t>(2345));
    const std::uint32_t imu_diagnostics[] =
        {0x01020304U, 0x11223344U, 0x55667788U, 0x99aabbccU,
         0xddeeff01U, 0x12345678U, 0x89abcdefU, 0xfedcba98U,
         0xdeadbeefU};
    for (std::size_t i = 0; i < 9U; ++i) {
        write_le32(imu, 20U + 4U * i, imu_diagnostics[i]);
    }
    const auto imu_decoded = decode_imu(imu, &reason);
    expect(imu_decoded && imu_decoded->schema_version == 1U &&
               imu_decoded->validity_flags == 0x07U &&
               imu_decoded->acc_mg == std::array<std::int16_t, 3>{1000, -2000, 32767} &&
               imu_decoded->gyro_tenth_dps == std::array<std::int16_t, 3>{-123, 456, -32768} &&
               imu_decoded->angle_centidegrees == std::array<std::int16_t, 3>{-1, 0, 2345} &&
               imu_decoded->diagnostics.rx_byte_count == 0x01020304U &&
               imu_decoded->diagnostics.header_count == 0x11223344U &&
               imu_decoded->diagnostics.valid_frame_count == 0x55667788U &&
               imu_decoded->diagnostics.checksum_error_count == 0x99aabbccU &&
               imu_decoded->diagnostics.rx_buffer_overflow_count == 0xddeeff01U &&
               imu_decoded->diagnostics.rx_rearm_failure_count == 0x12345678U &&
               imu_decoded->diagnostics.uart_error_count == 0x89abcdefU &&
               imu_decoded->diagnostics.mag_frame_count == 0xfedcba98U &&
               imu_decoded->diagnostics.unsupported_frame_count == 0xdeadbeefU,
           "IMU valid fixture matches every Firmware diagnostic offset");
    expect(!decode_imu(Bytes(55U, 0U), &reason) &&
               reason == TelemetryMalformedReason::WrongLength,
           "IMU wrong length is malformed");
    Bytes imu_wrong_schema = imu;
    imu_wrong_schema[0] = 2U;
    expect(!decode_imu(imu_wrong_schema, &reason) &&
               reason == TelemetryMalformedReason::WrongSchema,
           "IMU wrong schema is malformed");
    Bytes imu_reserved_flags = imu;
    imu_reserved_flags[1] = 0x08U;
    expect(!decode_imu(imu_reserved_flags, &reason) &&
               reason == TelemetryMalformedReason::ReservedFlags,
           "IMU reserved flags are malformed");
    Bytes imu_invalid_acc = imu;
    imu_invalid_acc[1] = 0x06U;
    imu_invalid_acc[2] = 1U;
    expect(!decode_imu(imu_invalid_acc, &reason) &&
               reason == TelemetryMalformedReason::InvalidDomain,
           "nonzero invalid IMU domain is malformed");
    Bytes imu_invalid_gyro = imu;
    imu_invalid_gyro[1] = 0x05U;
    imu_invalid_gyro[8] = 1U;
    expect(!decode_imu(imu_invalid_gyro, &reason) &&
               reason == TelemetryMalformedReason::InvalidDomain,
           "nonzero invalid IMU gyro domain is malformed");
    Bytes imu_invalid_angle = imu;
    imu_invalid_angle[1] = 0x03U;
    imu_invalid_angle[14] = 1U;
    expect(!decode_imu(imu_invalid_angle, &reason) &&
               reason == TelemetryMalformedReason::InvalidDomain,
           "nonzero invalid IMU angle domain is malformed");
    Bytes imu_zero_invalid_domains = imu;
    imu_zero_invalid_domains[1] = 0x00U;
    for (std::size_t offset = 2U; offset < 20U; ++offset) {
        imu_zero_invalid_domains[offset] = 0U;
    }
    const auto imu_zero_decoded = decode_imu(imu_zero_invalid_domains, &reason);
    expect(imu_zero_decoded &&
               imu_zero_decoded->acc_mg == std::array<std::int16_t, 3>{0, 0, 0} &&
               imu_zero_decoded->gyro_tenth_dps == std::array<std::int16_t, 3>{0, 0, 0} &&
               imu_zero_decoded->angle_centidegrees == std::array<std::int16_t, 3>{0, 0, 0},
           "zero-valued invalid IMU domains are legal");

    Bytes depth(38U, 0U);
    depth[0] = 1U;
    depth[1] = 0x03U;
    write_le32(depth, 2U, static_cast<std::uint32_t>(-4320));
    write_le16(depth, 6U, static_cast<std::uint16_t>(-257));
    write_le16(depth, 8U, 0xffffU);
    const std::uint32_t depth_diagnostics[] = {
        0x01020304U, 0x11223344U, 0x55667788U, 0x99aabbccU,
        0xddeeff01U, 0x12345678U, 0x89abcdefU};
    for (std::size_t i = 0; i < 7U; ++i) {
        write_le32(depth, 10U + 4U * i, depth_diagnostics[i]);
    }
    const auto depth_decoded = decode_depth(depth, &reason);
    expect(depth_decoded && depth_decoded->schema_version == 1U &&
               depth_decoded->validity_flags == 0x03U &&
               depth_decoded->depth_mm == -4320 &&
               depth_decoded->temperature_centi_c == -257 &&
               depth_decoded->sample_age_ms == 0xffffU &&
               depth_decoded->diagnostics.rx_byte_count == 0x01020304U &&
               depth_decoded->diagnostics.valid_line_count == 0x11223344U &&
               depth_decoded->diagnostics.parse_error_count == 0x55667788U &&
               depth_decoded->diagnostics.overlong_line_count == 0x99aabbccU &&
               depth_decoded->diagnostics.rx_buffer_overflow_count == 0xddeeff01U &&
               depth_decoded->diagnostics.hard_rearm_failure_count == 0x12345678U &&
               depth_decoded->diagnostics.uart_error_count == 0x89abcdefU,
           "Depth valid fixture matches every Firmware diagnostic offset");
    expect(!decode_depth(Bytes(37U, 0U), &reason) &&
               reason == TelemetryMalformedReason::WrongLength,
           "Depth wrong length is malformed");
    Bytes depth_wrong_schema = depth;
    depth_wrong_schema[0] = 2U;
    expect(!decode_depth(depth_wrong_schema, &reason) &&
               reason == TelemetryMalformedReason::WrongSchema,
           "Depth wrong schema is malformed");
    Bytes depth_reserved_flags = depth;
    depth_reserved_flags[1] = 0x80U;
    expect(!decode_depth(depth_reserved_flags, &reason) &&
               reason == TelemetryMalformedReason::ReservedFlags,
           "Depth reserved flags are malformed");
    Bytes depth_invalid_value = depth;
    depth_invalid_value[1] = 0x02U;
    depth_invalid_value[2] = 1U;
    expect(!decode_depth(depth_invalid_value, &reason) &&
               reason == TelemetryMalformedReason::InvalidDomain,
           "nonzero invalid depth domain is malformed");
    Bytes depth_invalid_age = depth;
    depth_invalid_age[1] = 0x00U;
    for (std::size_t offset = 2U; offset < 8U; ++offset) {
        depth_invalid_age[offset] = 0U;
    }
    write_le16(depth_invalid_age, 8U, 0U);
    expect(!decode_depth(depth_invalid_age, &reason) &&
               reason == TelemetryMalformedReason::InvalidDomain,
           "invalid depth requires unknown sample age");
    Bytes depth_invalid_temperature = depth;
    depth_invalid_temperature[1] = 0x01U;
    depth_invalid_temperature[6] = 1U;
    expect(!decode_depth(depth_invalid_temperature, &reason) &&
               reason == TelemetryMalformedReason::InvalidDomain,
           "nonzero invalid temperature domain is malformed");

    Bytes depth_zero_invalid(38U, 0U);
    depth_zero_invalid[0] = 1U;
    write_le16(depth_zero_invalid, 8U, 0xffffU);
    const auto depth_zero_decoded = decode_depth(depth_zero_invalid, &reason);
    expect(depth_zero_decoded && depth_zero_decoded->validity_flags == 0U &&
               depth_zero_decoded->depth_mm == 0 &&
               depth_zero_decoded->temperature_centi_c == 0 &&
               depth_zero_decoded->sample_age_ms == 0xffffU &&
               reason == TelemetryMalformedReason::None,
           "zero invalid depth and temperature with unknown age is legal");

    Bytes depth_boundary = depth;
    write_le32(depth_boundary, 2U, 0x80000000U);
    write_le16(depth_boundary, 6U, 0x8000U);
    write_le16(depth_boundary, 8U, 0U);
    const auto minimum = decode_depth(depth_boundary);
    expect(minimum && minimum->depth_mm == INT32_MIN &&
               minimum->temperature_centi_c == INT16_MIN && minimum->sample_age_ms == 0U,
           "signed depth and temperature minima decode without range policy");
    write_le32(depth_boundary, 2U, 0x7fffffffU);
    write_le16(depth_boundary, 6U, 0x7fffU);
    const auto maximum = decode_depth(depth_boundary);
    expect(maximum && maximum->depth_mm == INT32_MAX &&
               maximum->temperature_centi_c == INT16_MAX,
           "signed depth and temperature maxima decode without range policy");
    expect(decode_imu(imu).has_value() && !decode_imu(Bytes{}) &&
               !decode_depth(Bytes{}),
           "IMU and Depth reason outputs are optional");
}

} // namespace

int main()
{
    command_payloads_and_validation();
    telemetry_decoders_match_firmware_layout();
    if (rbp2_test::failures == 0) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
