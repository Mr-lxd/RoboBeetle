#include "depth_telemetry.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static void write_le16(
    uint8_t *data,
    uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
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

static uint16_t read_le16(
    const uint8_t *data)
{
    return (uint16_t)data[0]
        | (uint16_t)((uint16_t)data[1] << 8U);
}

static uint32_t read_le32(
    const uint8_t *data)
{
    return (uint32_t)data[0]
        | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U)
        | ((uint32_t)data[3] << 24U);
}

static int16_t read_signed_le16(
    const uint8_t *data)
{
    uint16_t raw = read_le16(data);

    if ((raw & 0x8000U) == 0U)
    {
        return (int16_t)raw;
    }

    if (raw == 0x8000U)
    {
        return INT16_MIN;
    }

    return (int16_t)-(int16_t)((uint16_t)(~raw) + 1U);
}

static int32_t read_signed_le32(
    const uint8_t *data)
{
    uint32_t raw = read_le32(data);

    if ((raw & 0x80000000U) == 0U)
    {
        return (int32_t)raw;
    }

    if (raw == 0x80000000U)
    {
        return INT32_MIN;
    }

    return (int32_t)-(int32_t)((uint32_t)(~raw) + 1U);
}

static uint16_t encode_sample_age(
    const depth_telemetry_source_t *source)
{
    if (!source->depth_valid)
    {
        /* sample_age_ms describes the latest valid depth sample only. */
        return DEPTH_TELEMETRY_SAMPLE_AGE_UNKNOWN;
    }

    if (source->sample_age_ms >= UINT16_MAX)
    {
        return DEPTH_TELEMETRY_SAMPLE_AGE_UNKNOWN;
    }

    return (uint16_t)source->sample_age_ms;
}

size_t depth_telemetry_encode(
    const depth_telemetry_source_t *source,
    const depth_telemetry_diagnostics_t *diagnostics,
    uint8_t *payload,
    size_t payload_capacity)
{
    uint8_t flags = 0U;
    uint16_t sample_age;

    if ((source == NULL) ||
        (diagnostics == NULL) ||
        (payload == NULL) ||
        (payload_capacity < DEPTH_TELEMETRY_PAYLOAD_LENGTH))
    {
        return 0U;
    }

    for (size_t index = 0U;
         index < DEPTH_TELEMETRY_PAYLOAD_LENGTH;
         ++index)
    {
        payload[index] = 0U;
    }

    if (source->depth_valid)
    {
        flags |= DEPTH_TELEMETRY_FLAG_DEPTH_VALID;
    }

    if (source->temperature_valid)
    {
        flags |= DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID;
    }

    payload[0] = DEPTH_TELEMETRY_SCHEMA_VERSION;
    payload[1] = flags;

    if (source->depth_valid)
    {
        write_le32(&payload[2], (uint32_t)source->depth_mm);
    }

    if (source->temperature_valid)
    {
        write_le16(
            &payload[6],
            (uint16_t)source->temperature_centi_c);
    }

    sample_age = encode_sample_age(source);
    write_le16(&payload[8], sample_age);

    write_le32(&payload[10], diagnostics->rx_byte_count);
    write_le32(&payload[14], diagnostics->valid_line_count);
    write_le32(&payload[18], diagnostics->parse_error_count);
    write_le32(&payload[22], diagnostics->overlong_line_count);
    write_le32(&payload[26], diagnostics->rx_buffer_overflow_count);
    write_le32(&payload[30], diagnostics->hard_rearm_failure_count);
    write_le32(&payload[34], diagnostics->uart_error_count);

    return DEPTH_TELEMETRY_PAYLOAD_LENGTH;
}

bool depth_telemetry_decode(
    const uint8_t *payload,
    size_t payload_length,
    depth_telemetry_source_t *source,
    depth_telemetry_diagnostics_t *diagnostics)
{
    uint8_t flags;
    uint32_t depth_raw;
    uint16_t temperature_raw;
    uint16_t sample_age_raw;
    depth_telemetry_source_t decoded_source = {0};
    depth_telemetry_diagnostics_t decoded_diagnostics = {0};

    if ((payload == NULL) ||
        (payload_length != DEPTH_TELEMETRY_PAYLOAD_LENGTH) ||
        (source == NULL) ||
        (diagnostics == NULL))
    {
        return false;
    }

    if (payload[0] != DEPTH_TELEMETRY_SCHEMA_VERSION)
    {
        return false;
    }

    flags = payload[1];

    if ((flags & DEPTH_TELEMETRY_RESERVED_FLAGS) != 0U)
    {
        return false;
    }

    depth_raw = read_le32(&payload[2]);
    temperature_raw = read_le16(&payload[6]);
    sample_age_raw = read_le16(&payload[8]);

    if (((flags & DEPTH_TELEMETRY_FLAG_DEPTH_VALID) == 0U) &&
        (depth_raw != 0U))
    {
        return false;
    }

    if (((flags & DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID) == 0U) &&
        (temperature_raw != 0U))
    {
        return false;
    }

    if (((flags & DEPTH_TELEMETRY_FLAG_DEPTH_VALID) == 0U) &&
        (sample_age_raw != DEPTH_TELEMETRY_SAMPLE_AGE_UNKNOWN))
    {
        return false;
    }

    decoded_source.depth_valid =
        (flags & DEPTH_TELEMETRY_FLAG_DEPTH_VALID) != 0U;
    decoded_source.temperature_valid =
        (flags & DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID) != 0U;
    decoded_source.depth_mm = decoded_source.depth_valid
        ? read_signed_le32(&payload[2])
        : 0;
    decoded_source.temperature_centi_c = decoded_source.temperature_valid
        ? read_signed_le16(&payload[6])
        : 0;
    decoded_source.sample_age_ms = sample_age_raw;

    decoded_diagnostics.rx_byte_count = read_le32(&payload[10]);
    decoded_diagnostics.valid_line_count = read_le32(&payload[14]);
    decoded_diagnostics.parse_error_count = read_le32(&payload[18]);
    decoded_diagnostics.overlong_line_count = read_le32(&payload[22]);
    decoded_diagnostics.rx_buffer_overflow_count = read_le32(&payload[26]);
    decoded_diagnostics.hard_rearm_failure_count = read_le32(&payload[30]);
    decoded_diagnostics.uart_error_count = read_le32(&payload[34]);

    *source = decoded_source;
    *diagnostics = decoded_diagnostics;

    return true;
}

void depth_telemetry_policy_init(
    depth_telemetry_policy_t *policy)
{
    if (policy != NULL)
    {
        policy->last_success_ms = 0U;
        policy->has_success = false;
    }
}

bool depth_telemetry_policy_is_due(
    const depth_telemetry_policy_t *policy,
    uint32_t now_ms)
{
    if (policy == NULL)
    {
        return false;
    }

    if (!policy->has_success)
    {
        return true;
    }

    return (uint32_t)(now_ms - policy->last_success_ms) >=
           DEPTH_TELEMETRY_INTERVAL_MS;
}

void depth_telemetry_policy_mark_success(
    depth_telemetry_policy_t *policy,
    uint32_t now_ms)
{
    if (policy != NULL)
    {
        policy->last_success_ms = now_ms;
        policy->has_success = true;
    }
}
