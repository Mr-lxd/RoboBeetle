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
 * Firmware-side sensor freshness (3000 ms) is a different thing from:
 *  - the telemetry publication interval below,
 *  - the Qt host's control freshness (DepthControlConfig::controlFreshMs,
 *    provisionally 700 ms), which gates automatic ASCEND/DESCEND, and
 *  - the Qt UI display StaleTimeoutMs (3500 ms).
 * Vendor line cadence is not specified, so this provisional 3000 ms bound
 * is kept; it only decides when old values are scrubbed from telemetry.
 */
#define DEPTH_TELEMETRY_SENSOR_FRESHNESS_TIMEOUT_MS 3000U

/*
 * Telemetry rides on accepted Heartbeat ACKs (one every 250 ms, fixed by the
 * RBRP handshake). A fresh sample is published at most once per
 * DEPTH_TELEMETRY_INTERVAL_MS; the value must stay below the heartbeat period
 * so a heartbeat arriving slightly early does not skip a whole beat (which
 * would degrade the rate to 500 ms). Without a new sample a frame is still
 * sent every DEPTH_TELEMETRY_KEEPALIVE_INTERVAL_MS so the host sees
 * sensor-stopped state and diagnostics.
 */
#define DEPTH_TELEMETRY_INTERVAL_MS 200U
#define DEPTH_TELEMETRY_KEEPALIVE_INTERVAL_MS 1000U

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
    uint32_t last_published_valid_line_count;
    bool has_success;
} depth_telemetry_policy_t;

void depth_telemetry_policy_init(
    depth_telemetry_policy_t *policy);

/* Due when a new valid line arrived and INTERVAL_MS has elapsed, or when
 * KEEPALIVE_INTERVAL_MS has elapsed since the last published frame. */
bool depth_telemetry_policy_is_due(
    const depth_telemetry_policy_t *policy,
    uint32_t now_ms,
    uint32_t valid_line_count);

void depth_telemetry_policy_mark_success(
    depth_telemetry_policy_t *policy,
    uint32_t now_ms,
    uint32_t valid_line_count);

#endif /* ROBOBEETLE_DEPTH_TELEMETRY_H */
