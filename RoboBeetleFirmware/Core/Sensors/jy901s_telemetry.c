#include "jy901s_telemetry.h"

#include <string.h>

static void write_le16(
    uint8_t *data,
    int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
}

static void write_le32(
    uint8_t *data,
    uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static int16_t encode_fixed_point(
    float value,
    float scale)
{
    float scaled = value * scale;
    int32_t rounded;

    /* NaN is never a useful live sensor value. */
    if (!(scaled == scaled))
    {
        return 0;
    }

    /* Check before the integer conversion so infinities are safe too. */
    if (scaled >= 32767.5f)
    {
        return 32767;
    }

    if (scaled <= -32768.5f)
    {
        return (int16_t)-32768;
    }

    rounded = scaled >= 0.0f
        ? (int32_t)(scaled + 0.5f)
        : (int32_t)(scaled - 0.5f);

    if (rounded > 32767)
    {
        return 32767;
    }

    if (rounded < -32768)
    {
        return (int16_t)-32768;
    }

    return (int16_t)rounded;
}

static void write_domain(
    uint8_t *payload,
    uint8_t offset,
    bool valid,
    float x,
    float y,
    float z,
    float scale)
{
    if (!valid)
    {
        write_le16(&payload[offset], 0);
        write_le16(&payload[(uint8_t)(offset + 2U)], 0);
        write_le16(&payload[(uint8_t)(offset + 4U)], 0);
        return;
    }

    write_le16(
        &payload[offset],
        encode_fixed_point(x, scale));
    write_le16(
        &payload[(uint8_t)(offset + 2U)],
        encode_fixed_point(y, scale));
    write_le16(
        &payload[(uint8_t)(offset + 4U)],
        encode_fixed_point(z, scale));
}

size_t jy901s_imu_telemetry_encode(
    const jy901s_imu_state_t *state,
    const jy901s_parser_stats_t *parser_stats,
    const jy901s_imu_telemetry_diagnostics_t *transport_diagnostics,
    uint8_t *payload,
    size_t payload_capacity)
{
    uint8_t flags = 0U;

    if ((state == NULL) ||
        (parser_stats == NULL) ||
        (transport_diagnostics == NULL) ||
        (payload == NULL) ||
        (payload_capacity < JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    (void)memset(
        payload,
        0,
        JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH);

    if (state->acc_valid)
    {
        flags |= JY901S_IMU_VALID_ACC;
    }

    if (state->gyro_valid)
    {
        flags |= JY901S_IMU_VALID_GYRO;
    }

    if (state->angle_valid)
    {
        flags |= JY901S_IMU_VALID_ANGLE;
    }

    payload[0] = JY901S_IMU_TELEMETRY_SCHEMA_VERSION;
    payload[1] = flags;

    write_domain(
        payload,
        2U,
        state->acc_valid,
        state->acc_x_g,
        state->acc_y_g,
        state->acc_z_g,
        1000.0f);
    write_domain(
        payload,
        8U,
        state->gyro_valid,
        state->gyro_x_dps,
        state->gyro_y_dps,
        state->gyro_z_dps,
        10.0f);
    write_domain(
        payload,
        14U,
        state->angle_valid,
        state->angle_roll_deg,
        state->angle_pitch_deg,
        state->angle_yaw_deg,
        100.0f);

    write_le32(&payload[20], transport_diagnostics->rx_byte_count);
    write_le32(&payload[24], parser_stats->header_count);
    write_le32(&payload[28], parser_stats->valid_frame_count);
    write_le32(&payload[32], parser_stats->checksum_error_count);
    write_le32(
        &payload[36],
        transport_diagnostics->rx_buffer_overflow_count);
    write_le32(
        &payload[40],
        transport_diagnostics->rx_rearm_failure_count);
    write_le32(
        &payload[44],
        transport_diagnostics->uart_error_count);
    write_le32(&payload[48], parser_stats->mag_frame_count);
    write_le32(
        &payload[52],
        parser_stats->unsupported_frame_count);

    return JY901S_IMU_TELEMETRY_PAYLOAD_LENGTH;
}
