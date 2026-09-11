#ifndef ROBOBEETLE_DEPTH_TELEMETRY_H
#define ROBOBEETLE_DEPTH_TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEPTH_TELEMETRY_MESSAGE_ID 0x22U
#define DEPTH_TELEMETRY_SCHEMA_VERSION 1U
#define DEPTH_TELEMETRY_PAYLOAD_LENGTH 38U

#define DEPTH_TELEMETRY_FLAG_DEPTH_VALID 0x01U
#define DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID 0x02U
#define DEPTH_TELEMETRY_VALID_FLAGS \
    (DEPTH_TELEMETRY_FLAG_DEPTH_VALID \
     | DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID)
#define DEPTH_TELEMETRY_RESERVED_FLAGS 0xFCU

/* 0xFFFF represents an unknown or saturated age on the wire. */
#define DEPTH_TELEMETRY_SAMPLE_AGE_UNKNOWN UINT16_MAX

/*
 * Firmware-side sensor freshness is separate from the one-second telemetry
 * publication interval and from the Qt host packet StaleTimeoutMs.
 * Vendor line cadence is not specified, so this provisional 3000 ms bound
 * allows three current one-second publication intervals before a valid depth
 * sample is no longer used for live telemetry. The timeout is intentionally
 * explicit so it can be revised when the vendor cadence is measured.
 */
#define DEPTH_TELEMETRY_SENSOR_FRESHNESS_TIMEOUT_MS 3000U
#define DEPTH_TELEMETRY_INTERVAL_MS 1000U

/* Names matching the protocol's DepthSnapshot terminology. */
#define DEPTH_SNAPSHOT_MESSAGE_ID DEPTH_TELEMETRY_MESSAGE_ID
#define DEPTH_SNAPSHOT_SCHEMA_VERSION DEPTH_TELEMETRY_SCHEMA_VERSION
#define DEPTH_SNAPSHOT_PAYLOAD_LENGTH DEPTH_TELEMETRY_PAYLOAD_LENGTH
#define DEPTH_SNAPSHOT_FLAG_DEPTH_VALID DEPTH_TELEMETRY_FLAG_DEPTH_VALID
#define DEPTH_SNAPSHOT_FLAG_TEMPERATURE_VALID \
    DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID

typedef struct
{
    bool depth_valid;
    bool temperature_valid;
    int32_t depth_mm;
    int16_t temperature_centi_c;

    /* A wider source value lets the encoder saturate instead of wrapping. */
    uint32_t sample_age_ms;
} depth_telemetry_source_t;

typedef struct
{
    uint32_t rx_byte_count;
    uint32_t valid_line_count;
    uint32_t parse_error_count;
    uint32_t overlong_line_count;
    uint32_t rx_buffer_overflow_count;
    uint32_t hard_rearm_failure_count;
    uint32_t uart_error_count;
} depth_telemetry_diagnostics_t;

typedef depth_telemetry_source_t depth_snapshot_source_t;
typedef depth_telemetry_diagnostics_t depth_snapshot_diagnostics_t;

size_t depth_telemetry_encode(
    const depth_telemetry_source_t *source,
    const depth_telemetry_diagnostics_t *diagnostics,
    uint8_t *payload,
    size_t payload_capacity);

bool depth_telemetry_decode(
    const uint8_t *payload,
    size_t payload_length,
    depth_telemetry_source_t *source,
    depth_telemetry_diagnostics_t *diagnostics);

bool depth_telemetry_sensor_sample_is_current(
    bool parser_sample_valid,
    uint32_t last_valid_sample_ms,
    uint32_t now_ms);

typedef struct
{
    uint32_t last_success_ms;
    bool has_success;
} depth_telemetry_policy_t;

void depth_telemetry_policy_init(
    depth_telemetry_policy_t *policy);

bool depth_telemetry_policy_is_due(
    const depth_telemetry_policy_t *policy,
    uint32_t now_ms);

void depth_telemetry_policy_mark_success(
    depth_telemetry_policy_t *policy,
    uint32_t now_ms);

#endif /* ROBOBEETLE_DEPTH_TELEMETRY_H */
