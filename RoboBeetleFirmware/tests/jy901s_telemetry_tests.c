#include "jy901s_telemetry.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(bool condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static int16_t read_le16(
    const uint8_t *payload,
    uint8_t offset)
{
    uint16_t raw = (uint16_t)payload[offset]
        | (uint16_t)((uint16_t)payload[(uint8_t)(offset + 1U)] << 8U);

    return (int16_t)raw;
}

static uint32_t read_le32(
    const uint8_t *payload,
    uint8_t offset)
{
    return (uint32_t)payload[offset]
        | ((uint32_t)payload[(uint8_t)(offset + 1U)] << 8U)
        | ((uint32_t)payload[(uint8_t)(offset + 2U)] << 16U)
        | ((uint32_t)payload[(uint8_t)(offset + 3U)] << 24U);
}

static void test_fixed_layout_and_diagnostics(void)
{
    jy901s_imu_state_t state = {0};
    jy901s_parser_stats_t parser_stats = {0};
    jy901s_imu_telemetry_diagnostics_t transport = {0};
    uint8_t payload[JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH] = {0};

    state.acc_valid = true;
    state.acc_x_g = 1.234f;
    state.acc_y_g = -2.5f;
    state.acc_z_g = 0.001f;
    state.gyro_valid = true;
    state.gyro_x_dps = 12.3f;
    state.gyro_y_dps = -2.4f;
    state.gyro_z_dps = 0.0f;
    state.angle_valid = true;
    state.angle_roll_deg = 12.34f;
    state.angle_pitch_deg = -2.35f;
    state.angle_yaw_deg = 0.0f;

    parser_stats.header_count = 0x01020304U;
    parser_stats.valid_frame_count = 0x11121314U;
    parser_stats.checksum_error_count = 0x21222324U;
    parser_stats.mag_frame_count = 0x31323334U;
    parser_stats.unsupported_frame_count = 0x41424344U;
    transport.rx_byte_count = 0x51525354U;
    transport.rx_buffer_overflow_count = 0x61626364U;
    transport.rx_rearm_failure_count = 0x71727374U;
    transport.uart_error_count = 0x81828384U;

    expect(
        jy901s_imu_telemetry_encode(
            &state,
            &parser_stats,
            &transport,
            payload,
            sizeof payload) == JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH,
        "all-domain snapshot should have the fixed payload length");
    expect(payload[0] == JY901S_IMU_TELEMETRY_SCHEMA_VERSION,
           "snapshot schema version differs");
    expect(payload[1] == (JY901S_IMU_VALID_ACC
                          | JY901S_IMU_VALID_GYRO
                          | JY901S_IMU_VALID_ANGLE),
           "all valid domains should set exactly three flags");

    expect(read_le16(payload, 2U) == 1234,
           "Acc X must use integer millig encoding");
    expect(read_le16(payload, 4U) == -2500,
           "Acc Y must use signed integer millig encoding");
    expect(read_le16(payload, 6U) == 1,
           "Acc Z must use integer millig encoding");
    expect(read_le16(payload, 8U) == 123,
           "Gyro X must use 0.1 dps encoding");
    expect(read_le16(payload, 10U) == -24,
           "Gyro Y must use signed 0.1 dps encoding");
    expect(read_le16(payload, 12U) == 0,
           "Gyro Z encoding differs");
    expect(read_le16(payload, 14U) == 1234,
           "Angle roll must use 0.01 degree encoding");
    expect(read_le16(payload, 16U) == -235,
           "Angle pitch must use signed 0.01 degree encoding");
    expect(read_le16(payload, 18U) == 0,
           "Angle yaw encoding differs");

    expect(read_le32(payload, 20U) == transport.rx_byte_count,
           "RX byte diagnostic offset differs");
    expect(read_le32(payload, 24U) == parser_stats.header_count,
           "header diagnostic offset differs");
    expect(read_le32(payload, 28U) == parser_stats.valid_frame_count,
           "valid-frame diagnostic offset differs");
    expect(read_le32(payload, 32U) == parser_stats.checksum_error_count,
           "checksum diagnostic offset differs");
    expect(read_le32(payload, 36U) == transport.rx_buffer_overflow_count,
           "overflow diagnostic offset differs");
    expect(read_le32(payload, 40U) == transport.rx_rearm_failure_count,
           "re-arm diagnostic offset differs");
    expect(read_le32(payload, 44U) == transport.uart_error_count,
           "UART diagnostic offset differs");
    expect(read_le32(payload, 48U) == parser_stats.mag_frame_count,
           "Mag diagnostic offset differs");
    expect(read_le32(payload, 52U) == parser_stats.unsupported_frame_count,
           "unsupported diagnostic offset differs");
}

static void test_invalid_domains_are_zeroed(void)
{
    jy901s_imu_state_t state = {
        .acc_valid = false,
        .acc_x_g = 3.0f,
        .acc_y_g = -3.0f,
        .acc_z_g = 1.0f,
        .gyro_valid = true,
        .gyro_x_dps = 1.0f,
        .gyro_y_dps = 2.0f,
        .gyro_z_dps = 3.0f,
        .angle_valid = false,
        .angle_roll_deg = 4.0f,
        .angle_pitch_deg = 5.0f,
        .angle_yaw_deg = 6.0f,
    };
    jy901s_parser_stats_t parser_stats = {0};
    jy901s_imu_telemetry_diagnostics_t transport = {0};
    uint8_t payload[JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH] = {0};

    (void)jy901s_imu_telemetry_encode(
        &state,
        &parser_stats,
        &transport,
        payload,
        sizeof payload);

    expect(payload[1] == JY901S_IMU_VALID_GYRO,
           "invalid domains must clear their validity flags");
    expect(read_le16(payload, 2U) == 0
               && read_le16(payload, 4U) == 0
               && read_le16(payload, 6U) == 0,
           "invalid Acc values must be zero");
    expect(read_le16(payload, 14U) == 0
               && read_le16(payload, 16U) == 0
               && read_le16(payload, 18U) == 0,
           "invalid Angle values must be zero");
    expect(read_le16(payload, 8U) == 10
               && read_le16(payload, 10U) == 20
               && read_le16(payload, 12U) == 30,
           "valid Gyro values must remain encoded");
}

static void test_fixed_point_bounds_and_arguments(void)
{
    jy901s_imu_state_t state = {0};
    jy901s_parser_stats_t parser_stats = {0};
    jy901s_imu_telemetry_diagnostics_t transport = {0};
    uint8_t payload[JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH] = {0};

    state.acc_valid = true;
    state.acc_x_g = 100.0f;
    state.acc_y_g = -100.0f;
    state.gyro_valid = true;
    state.gyro_x_dps = 10000.0f;
    state.gyro_y_dps = -10000.0f;
    state.angle_valid = true;
    state.angle_roll_deg = 1000.0f;
    state.angle_pitch_deg = -1000.0f;

    expect(
        jy901s_imu_telemetry_encode(
            &state,
            &parser_stats,
            &transport,
            payload,
            sizeof payload) == JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH,
        "out-of-range source values should be clamped, not rejected");
    expect(read_le16(payload, 2U) == INT16_MAX
               && read_le16(payload, 4U) == INT16_MIN,
           "Acc fixed-point values must clamp to int16 bounds");
    expect(read_le16(payload, 8U) == INT16_MAX
               && read_le16(payload, 10U) == INT16_MIN,
           "Gyro fixed-point values must clamp to int16 bounds");
    expect(read_le16(payload, 14U) == INT16_MAX
               && read_le16(payload, 16U) == INT16_MIN,
           "Angle fixed-point values must clamp to int16 bounds");

    expect(jy901s_imu_telemetry_encode(
               NULL, &parser_stats, &transport, payload, sizeof payload) == 0U,
           "null state must fail safely");
    expect(jy901s_imu_telemetry_encode(
               &state, NULL, &transport, payload, sizeof payload) == 0U,
           "null parser stats must fail safely");
    expect(jy901s_imu_telemetry_encode(
               &state, &parser_stats, NULL, payload, sizeof payload) == 0U,
           "null transport diagnostics must fail safely");
    expect(jy901s_imu_telemetry_encode(
               &state,
               &parser_stats,
               &transport,
               payload,
               JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH - 1U) == 0U,
           "short output capacity must fail safely");
}

int main(void)
{
    test_fixed_layout_and_diagnostics();
    test_invalid_domains_are_zeroed();
    test_fixed_point_bounds_and_arguments();

    if (failures == 0)
    {
        (void)puts("All JY901S telemetry encoding tests passed");
    }

    return failures == 0 ? 0 : 1;
}
