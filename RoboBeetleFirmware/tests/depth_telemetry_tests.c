#include "depth_telemetry.h"

#include <stdbool.h>
#include <stddef.h>
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

static void expect_bytes(
    const uint8_t *actual,
    const uint8_t *expected,
    size_t length,
    const char *message)
{
    expect(memcmp(actual, expected, length) == 0, message);
}

static depth_telemetry_source_t make_source(void)
{
    depth_telemetry_source_t source = {0};

    source.depth_valid = true;
    source.temperature_valid = true;
    source.depth_mm = -123456789;
    source.temperature_centi_c = -1234;
    source.sample_age_ms = 1234U;

    return source;
}

static depth_telemetry_diagnostics_t make_diagnostics(void)
{
    depth_telemetry_diagnostics_t diagnostics = {0};

    diagnostics.rx_byte_count = 0x01020304U;
    diagnostics.valid_line_count = 0x11121314U;
    diagnostics.parse_error_count = 0x21222324U;
    diagnostics.overlong_line_count = 0x31323334U;
    diagnostics.rx_buffer_overflow_count = 0x41424344U;
    diagnostics.hard_rearm_failure_count = 0x51525354U;
    diagnostics.uart_error_count = 0x61626364U;

    return diagnostics;
}

static void test_constants_and_golden_payload(void)
{
    static const uint8_t expected_payload[] = {
        0x01U, 0x03U,
        0xEBU, 0x32U, 0xA4U, 0xF8U,
        0x2EU, 0xFBU,
        0xD2U, 0x04U,
        0x04U, 0x03U, 0x02U, 0x01U,
        0x14U, 0x13U, 0x12U, 0x11U,
        0x24U, 0x23U, 0x22U, 0x21U,
        0x34U, 0x33U, 0x32U, 0x31U,
        0x44U, 0x43U, 0x42U, 0x41U,
        0x54U, 0x53U, 0x52U, 0x51U,
        0x64U, 0x63U, 0x62U, 0x61U,
    };
    depth_telemetry_source_t source = make_source();
    depth_telemetry_diagnostics_t diagnostics = make_diagnostics();
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH] = {0};

    expect(DEPTH_TELEMETRY_MESSAGE_ID == 0x22U,
           "DepthSnapshot message ID must be 0x22");
    expect(DEPTH_TELEMETRY_PAYLOAD_LENGTH == 38U,
           "DepthSnapshot payload length must be 38 bytes");
    expect(DEPTH_TELEMETRY_SCHEMA_VERSION == 1U,
           "DepthSnapshot schema must be version 1");
    expect(DEPTH_TELEMETRY_FLAG_DEPTH_VALID == 0x01U,
           "depth-valid flag must be bit zero");
    expect(DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID == 0x02U,
           "temperature-valid flag must be bit one");

    expect(depth_telemetry_encode(
               &source,
               &diagnostics,
               payload,
               sizeof payload) == DEPTH_TELEMETRY_PAYLOAD_LENGTH,
           "valid source must encode to the fixed payload length");
    expect_bytes(payload,
                  expected_payload,
                  sizeof expected_payload,
                  "valid source payload differs from golden vector");
}

static void test_invalid_values_and_age_saturation_are_encoded_safely(void)
{
    depth_telemetry_source_t source = {
        .depth_valid = false,
        .temperature_valid = true,
        .depth_mm = 123456,
        .temperature_centi_c = -321,
        .sample_age_ms = 1234U,
    };
    depth_telemetry_diagnostics_t diagnostics = {0};
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH] = {0};

    expect(depth_telemetry_encode(
               &source,
               &diagnostics,
               payload,
               sizeof payload) == DEPTH_TELEMETRY_PAYLOAD_LENGTH,
           "partially valid source must encode");
    expect(payload[1] == DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID,
           "only the valid temperature flag may be set");
    expect(payload[2] == 0U && payload[3] == 0U
               && payload[4] == 0U && payload[5] == 0U,
           "invalid depth must be encoded as zero");
    expect(payload[6] == 0xBFU && payload[7] == 0xFEU,
           "valid signed temperature must use little endian encoding");
    expect(payload[8] == 0xFFU && payload[9] == 0xFFU,
           "temperature-only data must not create a depth sample age");

    source.depth_valid = true;
    source.depth_mm = 123456;
    source.sample_age_ms = 70000U;
    expect(depth_telemetry_encode(
               &source,
               &diagnostics,
               payload,
               sizeof payload) == DEPTH_TELEMETRY_PAYLOAD_LENGTH,
           "valid depth with an old sample must still encode");
    expect(payload[8] == 0xFFU && payload[9] == 0xFFU,
           "valid depth sample age must saturate at 0xFFFF");

    source.depth_valid = false;
    source.temperature_valid = false;
    source.temperature_centi_c = 456;
    source.sample_age_ms = 0U;
    (void)memset(payload, 0xA5, sizeof payload);

    expect(depth_telemetry_encode(
               &source,
               &diagnostics,
               payload,
               sizeof payload) == DEPTH_TELEMETRY_PAYLOAD_LENGTH,
           "source with no valid values must still encode");
    expect(payload[1] == 0U,
           "source with no valid values must have no validity flags");
    expect(payload[2] == 0U && payload[3] == 0U
               && payload[4] == 0U && payload[5] == 0U
               && payload[6] == 0U && payload[7] == 0U,
           "all invalid numeric fields must be encoded as zero");
    expect(payload[8] == 0xFFU && payload[9] == 0xFFU,
           "source with no valid sample must encode unknown age");
}

static void test_encode_argument_validation(void)
{
    depth_telemetry_source_t source = make_source();
    depth_telemetry_diagnostics_t diagnostics = make_diagnostics();
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH] = {0};

    expect(depth_telemetry_encode(
               NULL, &diagnostics, payload, sizeof payload) == 0U,
           "null source must fail encoding");
    expect(depth_telemetry_encode(
               &source, NULL, payload, sizeof payload) == 0U,
           "null diagnostics must fail encoding");
    expect(depth_telemetry_encode(
               &source, &diagnostics, NULL, sizeof payload) == 0U,
           "null payload must fail encoding");
    expect(depth_telemetry_encode(
               &source,
               &diagnostics,
               payload,
               DEPTH_TELEMETRY_PAYLOAD_LENGTH - 1U) == 0U,
           "short payload capacity must fail encoding");
}

static void test_decode_round_trip_preserves_signed_values_and_diagnostics(void)
{
    depth_telemetry_source_t source = make_source();
    depth_telemetry_diagnostics_t diagnostics = make_diagnostics();
    depth_telemetry_source_t decoded_source = {0};
    depth_telemetry_diagnostics_t decoded_diagnostics = {0};
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH] = {0};

    (void)depth_telemetry_encode(
        &source,
        &diagnostics,
        payload,
        sizeof payload);

    expect(depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "valid payload must decode");
    expect(decoded_source.depth_valid
               && decoded_source.temperature_valid,
           "decode must preserve both validity flags");
    expect(decoded_source.depth_mm == source.depth_mm,
           "decode must preserve signed depth millimetres");
    expect(decoded_source.temperature_centi_c == source.temperature_centi_c,
           "decode must preserve signed centi-degrees");
    expect(decoded_source.sample_age_ms == source.sample_age_ms,
           "decode must preserve sample age");
    expect(memcmp(&decoded_diagnostics,
                  &diagnostics,
                  sizeof diagnostics) == 0,
           "decode must preserve all diagnostics");
}

static void test_decode_rejects_malformed_payloads(void)
{
    depth_telemetry_source_t source = make_source();
    depth_telemetry_diagnostics_t diagnostics = make_diagnostics();
    depth_telemetry_source_t decoded_source = {0};
    depth_telemetry_diagnostics_t decoded_diagnostics = {0};
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH] = {0};

    (void)depth_telemetry_encode(
        &source,
        &diagnostics,
        payload,
        sizeof payload);

    expect(!depth_telemetry_decode(
               payload,
               DEPTH_TELEMETRY_PAYLOAD_LENGTH - 1U,
               &decoded_source,
               &decoded_diagnostics),
           "short payload must be rejected");
    expect(!depth_telemetry_decode(
               payload,
               DEPTH_TELEMETRY_PAYLOAD_LENGTH + 1U,
               &decoded_source,
               &decoded_diagnostics),
           "long payload must be rejected");

    payload[0] = 2U;
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "unknown schema must be rejected");
    payload[0] = DEPTH_TELEMETRY_SCHEMA_VERSION;

    payload[1] = (uint8_t)(DEPTH_TELEMETRY_FLAG_DEPTH_VALID | 0x80U);
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "reserved flags must be rejected");
    payload[1] = DEPTH_TELEMETRY_FLAG_DEPTH_VALID
        | DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID;

    payload[1] = DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID;
    payload[2] = 1U;
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "nonzero invalid depth must be rejected");
    payload[2] = 0U;

    payload[1] = DEPTH_TELEMETRY_FLAG_DEPTH_VALID;
    payload[6] = 1U;
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "nonzero invalid temperature must be rejected");
    payload[6] = 0U;

    payload[1] = 0U;
    payload[8] = 0U;
    payload[9] = 0U;
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "no-valid-sample payload must use unknown age");

    payload[1] = DEPTH_TELEMETRY_FLAG_TEMPERATURE_VALID;
    payload[2] = 0U;
    payload[3] = 0U;
    payload[4] = 0U;
    payload[5] = 0U;
    payload[8] = 0xD2U;
    payload[9] = 0x04U;
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "temperature-only payload must use unknown depth age");

    expect(!depth_telemetry_decode(
               NULL,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "null payload must be rejected");
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               NULL,
               &decoded_diagnostics),
           "null source output must be rejected");
    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               NULL),
           "null diagnostics output must be rejected");
}

static void test_decode_failure_does_not_publish_partial_output(void)
{
    depth_telemetry_source_t source = make_source();
    depth_telemetry_diagnostics_t diagnostics = make_diagnostics();
    depth_telemetry_source_t decoded_source = {
        .depth_valid = true,
        .temperature_valid = false,
        .depth_mm = 777,
        .temperature_centi_c = -888,
        .sample_age_ms = 999U,
    };
    depth_telemetry_diagnostics_t decoded_diagnostics = {
        .rx_byte_count = 0xA1A2A3A4U,
        .valid_line_count = 0xB1B2B3B4U,
        .parse_error_count = 0xC1C2C3C4U,
        .overlong_line_count = 0xD1D2D3D4U,
        .rx_buffer_overflow_count = 0xE1E2E3E4U,
        .hard_rearm_failure_count = 0xF1F2F3F4U,
        .uart_error_count = 0x01010101U,
    };
    depth_telemetry_source_t source_before = decoded_source;
    depth_telemetry_diagnostics_t diagnostics_before = decoded_diagnostics;
    uint8_t payload[DEPTH_TELEMETRY_PAYLOAD_LENGTH] = {0};

    (void)depth_telemetry_encode(
        &source,
        &diagnostics,
        payload,
        sizeof payload);
    payload[1] = 0x80U;

    expect(!depth_telemetry_decode(
               payload,
               sizeof payload,
               &decoded_source,
               &decoded_diagnostics),
           "malformed payload must fail before output assignment");
    expect(memcmp(&decoded_source,
                  &source_before,
                  sizeof decoded_source) == 0,
           "failed decode must not partially update source output");
    expect(memcmp(&decoded_diagnostics,
                  &diagnostics_before,
                  sizeof decoded_diagnostics) == 0,
           "failed decode must not partially update diagnostics output");
}

static void test_policy_is_due_at_first_opportunity_and_interval_boundary(void)
{
    depth_telemetry_policy_t policy = {0};

    depth_telemetry_policy_init(&policy);
    expect(depth_telemetry_policy_is_due(&policy, 0U),
           "depth policy must be due before its first success");

    depth_telemetry_policy_mark_success(&policy, 500U);
    expect(!depth_telemetry_policy_is_due(&policy, 1499U),
           "depth policy must wait for the complete one-second interval");
    expect(depth_telemetry_policy_is_due(&policy, 1500U),
           "depth policy must be due at the one-second boundary");
    expect(depth_telemetry_policy_is_due(&policy, 2500U),
           "depth policy remains due until a success is recorded");

    depth_telemetry_policy_mark_success(&policy, 2500U);
    expect(!depth_telemetry_policy_is_due(&policy, 3499U),
           "successful publication must restart the interval");
    expect(depth_telemetry_policy_is_due(&policy, 3500U),
           "policy must become due after the restarted interval");
}

static void test_policy_elapsed_comparison_is_wrap_safe(void)
{
    depth_telemetry_policy_t policy = {0};

    depth_telemetry_policy_init(&policy);
    depth_telemetry_policy_mark_success(&policy, 0xFFFFFF00U);

    expect(!depth_telemetry_policy_is_due(&policy, 0x000002E7U),
           "wrapped elapsed time of 999 ms must not be due");
    expect(depth_telemetry_policy_is_due(&policy, 0x000002E8U),
           "wrapped elapsed time of 1000 ms must be due");
}

static void test_policy_null_arguments_are_safe(void)
{
    depth_telemetry_policy_t policy = {0};

    depth_telemetry_policy_init(NULL);
    depth_telemetry_policy_mark_success(NULL, 123U);
    expect(!depth_telemetry_policy_is_due(NULL, 123U),
           "null depth policy must not report due");

    depth_telemetry_policy_init(&policy);
    expect(policy.has_success == false,
           "policy init must clear the success marker");
}

static void test_sensor_freshness_policy_expires_and_recovers(void)
{
    const uint32_t sample_ms = 1000U;
    const uint32_t just_before_timeout_ms =
        sample_ms + DEPTH_TELEMETRY_SENSOR_FRESHNESS_TIMEOUT_MS - 1U;
    const uint32_t at_timeout_ms =
        sample_ms + DEPTH_TELEMETRY_SENSOR_FRESHNESS_TIMEOUT_MS;
    const uint32_t fresh_sample_ms = at_timeout_ms + 100U;

    expect(depth_telemetry_sensor_sample_is_current(
               true,
               sample_ms,
               just_before_timeout_ms),
           "valid depth sample must remain current before timeout");
    expect(!depth_telemetry_sensor_sample_is_current(
                true,
                sample_ms,
                at_timeout_ms),
           "valid depth sample must become stale at timeout");
    expect(!depth_telemetry_sensor_sample_is_current(
                true,
                sample_ms,
                fresh_sample_ms),
           "old depth sample must remain stale after timeout");
    expect(depth_telemetry_sensor_sample_is_current(
               true,
               fresh_sample_ms,
               fresh_sample_ms),
           "new valid depth sample must recover freshness");
    expect(!depth_telemetry_sensor_sample_is_current(
                false,
                fresh_sample_ms,
                fresh_sample_ms),
           "invalid parser state must not report a current sample");
}

static void test_sensor_freshness_policy_is_wrap_safe(void)
{
    const uint32_t sample_ms = UINT32_MAX - 500U;

    expect(depth_telemetry_sensor_sample_is_current(
               true,
               sample_ms,
               2498U),
           "wrapped depth sample age below timeout must remain current");
    expect(!depth_telemetry_sensor_sample_is_current(
                true,
                sample_ms,
                2499U),
           "wrapped depth sample age at timeout must be stale");
}

int main(void)
{
    test_constants_and_golden_payload();
    test_invalid_values_and_age_saturation_are_encoded_safely();
    test_encode_argument_validation();
    test_decode_round_trip_preserves_signed_values_and_diagnostics();
    test_decode_rejects_malformed_payloads();
    test_decode_failure_does_not_publish_partial_output();
    test_policy_is_due_at_first_opportunity_and_interval_boundary();
    test_policy_elapsed_comparison_is_wrap_safe();
    test_policy_null_arguments_are_safe();
    test_sensor_freshness_policy_expires_and_recovers();
    test_sensor_freshness_policy_is_wrap_safe();

    if (failures == 0)
    {
        (void)puts("All depth telemetry codec and policy tests passed");
    }

    return failures == 0 ? 0 : 1;
}
