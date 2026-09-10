#include "protocol_dispatcher.h"

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

typedef struct
{
    bool start_result;
    unsigned int write_calls;
    unsigned int start_calls;
    unsigned int stop_calls;
    uint8_t last_servo_id;
    uint16_t last_pulse_us;
} fake_driver_t;

static void fake_write_pulse(
    void *context,
    uint8_t servo_id,
    uint16_t pulse_us)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->write_calls;
    driver->last_servo_id = servo_id;
    driver->last_pulse_us = pulse_us;
}

static bool fake_start(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->start_calls;
    driver->last_servo_id = servo_id;
    return driver->start_result;
}

static void fake_stop(
    void *context,
    uint8_t servo_id)
{
    fake_driver_t *driver = (fake_driver_t *)context;

    ++driver->stop_calls;
    driver->last_servo_id = servo_id;
}

static const servo_service_driver_ops_t fake_ops = {
    .write_pulse_us = fake_write_pulse,
    .start = fake_start,
    .stop = fake_stop,
};

typedef struct
{
    fake_driver_t driver;
    servo_service_t servo_service;
    safety_supervisor_t safety_supervisor;
    protocol_dispatcher_t dispatcher;
} fixture_t;

static void fixture_init(fixture_t *fixture)
{
    (void)memset(fixture, 0, sizeof(*fixture));
    fixture->driver.start_result = true;

    servo_service_init(
        &fixture->servo_service,
        &fake_ops,
        &fixture->driver);
    safety_supervisor_init(&fixture->safety_supervisor);
    protocol_dispatcher_init(
        &fixture->dispatcher,
        &fixture->servo_service,
        &fixture->safety_supervisor);
}

static rbp2_frame_t make_frame(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length)
{
    rbp2_frame_t frame = {0};

    frame.type = type;
    frame.sequence = sequence;
    frame.payload_length = payload_length;

    if ((payload != NULL) && (payload_length > 0U))
    {
        (void)memcpy(
            frame.payload,
            payload,
            payload_length);
    }

    return frame;
}

static void write_le16(
    uint8_t *data,
    uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static protocol_dispatcher_outcome_t handle(
    fixture_t *fixture,
    const rbp2_frame_t *frame,
    uint32_t now_ms)
{
    return protocol_dispatcher_handle(
        &fixture->dispatcher,
        frame,
        now_ms);
}

static void accept_heartbeat(
    fixture_t *fixture,
    uint16_t sequence,
    uint32_t uptime_ms,
    uint32_t now_ms)
{
    uint8_t payload[4] = {
        (uint8_t)(uptime_ms & 0xFFU),
        (uint8_t)((uptime_ms >> 8U) & 0xFFU),
        (uint8_t)((uptime_ms >> 16U) & 0xFFU),
        (uint8_t)((uptime_ms >> 24U) & 0xFFU),
    };
    rbp2_frame_t frame = make_frame(
        RBP2_MSG_HEARTBEAT,
        sequence,
        payload,
        sizeof(payload));
    protocol_dispatcher_outcome_t outcome =
        handle(fixture, &frame, now_ms);

    expect(outcome.result == RBP2_RESULT_OK,
           "valid heartbeat should return OK");
    expect(outcome.heartbeat_accepted,
           "valid heartbeat should be accepted");
}

static void test_valid_heartbeat(void)
{
    fixture_t fixture;
    uint8_t payload[4] = {0x12U, 0x34U, 0x56U, 0x78U};
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    frame = make_frame(
        RBP2_MSG_HEARTBEAT,
        7U,
        payload,
        sizeof(payload));
    outcome = handle(&fixture, &frame, 1234U);

    expect(outcome.result == RBP2_RESULT_OK,
           "valid heartbeat result should be OK");
    expect(outcome.heartbeat_accepted,
           "valid heartbeat outcome should be accepted");
    expect(outcome.heartbeat_uptime_ms == 0x78563412UL,
           "heartbeat uptime should decode little-endian");
    expect(!outcome.count_bad_frame,
           "valid heartbeat must not count as bad frame");
    expect(safety_supervisor_is_host_alive(
               &fixture.safety_supervisor),
           "valid heartbeat should mark host alive");
}

static void test_invalid_heartbeat_does_not_refresh_safety(void)
{
    fixture_t fixture;
    uint8_t invalid_payload[3] = {0x01U, 0x02U, 0x03U};
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 100U, 100U);

    frame = make_frame(
        RBP2_MSG_HEARTBEAT,
        2U,
        invalid_payload,
        sizeof(invalid_payload));
    outcome = handle(&fixture, &frame, 200U);

    expect(outcome.result == RBP2_RESULT_INVALID_PAYLOAD,
           "invalid heartbeat should return InvalidPayload");
    expect(!outcome.heartbeat_accepted,
           "invalid heartbeat must not be accepted");
    expect(outcome.count_bad_frame,
           "invalid heartbeat should mark a bad-frame outcome");
    expect(safety_supervisor_process(
               &fixture.safety_supervisor,
               601U),
           "invalid heartbeat must not refresh Safety timeout");
}

static void test_host_alive_validation_order(void)
{
    fixture_t fixture;
    uint8_t one_byte[1] = {0U};
    uint8_t three_bytes[3] = {1U, 0U, 0U};
    uint8_t valid_mask[2] = {1U, 0U};
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);

    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        1U,
        one_byte,
        sizeof(one_byte));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_INVALID_PAYLOAD,
           "Enable must validate length before HostAlive");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_PWM,
        2U,
        three_bytes,
        sizeof(three_bytes));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_HOST_NOT_ALIVE,
           "Set PWM must check HostAlive before length");

    frame = make_frame(
        RBP2_MSG_NEUTRAL,
        3U,
        one_byte,
        sizeof(one_byte));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_INVALID_PAYLOAD,
           "Neutral must validate length before HostAlive");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_ANGLE,
        4U,
        three_bytes,
        sizeof(three_bytes));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_HOST_NOT_ALIVE,
           "Set Angle must check HostAlive before length");

    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        5U,
        valid_mask,
        sizeof(valid_mask));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_HOST_NOT_ALIVE,
           "valid Enable should be gated by HostAlive");

    frame = make_frame(
        RBP2_MSG_NEUTRAL,
        6U,
        valid_mask,
        sizeof(valid_mask));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_HOST_NOT_ALIVE,
           "valid Neutral should be gated by HostAlive");

    frame = make_frame(
        RBP2_MSG_SERVO_DISABLE,
        7U,
        valid_mask,
        sizeof(valid_mask));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "Disable should remain allowed without HostAlive");
}

static void test_servo_commands(void)
{
    fixture_t fixture;
    uint8_t mask_payload[2];
    uint8_t pwm_payload[4] = {1U, 0U, 0x40U, 0x06U};
    uint8_t angle_payload[4] = {1U, 0U, 0x94U, 0x11U};
    uint8_t negative_angle_payload[4] = {1U, 0U, 0x6CU, 0xEEU};
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);

    write_le16(mask_payload,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);
    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        2U,
        mask_payload,
        sizeof(mask_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "valid Enable should return OK");
    expect(fixture.driver.start_calls == 1U,
           "valid Enable should start Servo1");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_PWM,
        3U,
        pwm_payload,
        sizeof(pwm_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "valid Set PWM should return OK");
    expect(fixture.driver.last_pulse_us == 1600U,
           "Set PWM should decode little-endian pulse");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_ANGLE,
        4U,
        angle_payload,
        sizeof(angle_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "valid Set Angle should return OK");
    expect(fixture.driver.last_pulse_us == 1950U,
           "Set Angle should pass cdeg to Servo service");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_ANGLE,
        41U,
        negative_angle_payload,
        sizeof(negative_angle_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "negative Set Angle should return OK");
    expect(fixture.driver.last_pulse_us == 1050U,
           "Set Angle should decode signed little-endian cdeg");

    frame = make_frame(
        RBP2_MSG_NEUTRAL,
        5U,
        mask_payload,
        sizeof(mask_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "valid Neutral should return OK");
    expect(fixture.driver.last_pulse_us == 1500U,
           "Neutral should call Servo service");

    frame = make_frame(
        RBP2_MSG_SERVO_DISABLE,
        6U,
        mask_payload,
        sizeof(mask_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "valid Disable should return OK");
    expect(fixture.driver.stop_calls == 1U,
           "valid Disable should stop Servo1");
    expect(servo_service_enabled_mask(&fixture.servo_service) == 0U,
           "Disable should clear Servo enabled state");
}

static void test_all_semantic_servo_ids_route(void)
{
    fixture_t fixture;
    uint8_t payload[4];

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);

    for (uint8_t servo_id = SERVO_ID_FRONT_RIGHT;
         servo_id <= SERVO_ID_REAR_LEFT;
         ++servo_id)
    {
        const servo_descriptor_t *descriptor =
            servo_descriptor_for_id(servo_id);
        rbp2_frame_t frame;
        protocol_dispatcher_outcome_t outcome;

        expect(descriptor != NULL,
               "every semantic servo ID must have a descriptor");
        if (descriptor == NULL)
        {
            continue;
        }

        write_le16(payload, descriptor->mask);
        frame = make_frame(
            RBP2_MSG_SERVO_ENABLE,
            (uint16_t)(100U + servo_id),
            payload,
            2U);
        outcome = handle(&fixture, &frame, 0U);
        expect(outcome.result == RBP2_RESULT_OK,
               "every semantic servo ID must accept Enable");

        payload[0] = 1U;
        payload[1] = servo_id;
        write_le16(&payload[2], descriptor->command_min_pulse_us);
        frame = make_frame(
            RBP2_MSG_SET_SERVO_PWM,
            (uint16_t)(200U + servo_id),
            payload,
            sizeof(payload));
        outcome = handle(&fixture, &frame, 0U);
        expect(outcome.result == RBP2_RESULT_OK,
               "every semantic servo ID must route Set PWM");
        expect(fixture.driver.last_servo_id == servo_id
                   && fixture.driver.last_pulse_us
                          == descriptor->command_min_pulse_us,
               "Set PWM must reach the descriptor-selected servo");

        payload[0] = 1U;
        payload[1] = servo_id;
        write_le16(&payload[2], 4500U);
        frame = make_frame(
            RBP2_MSG_SET_SERVO_ANGLE,
            (uint16_t)(300U + servo_id),
            payload,
            sizeof(payload));
        outcome = handle(&fixture, &frame, 0U);
        if (descriptor->angle_supported)
        {
            expect(outcome.result == RBP2_RESULT_OK,
                   "angle-capable semantic servo must route Set Angle");
        }
        else
        {
            expect(outcome.result == RBP2_RESULT_UNSUPPORTED_SERVO,
                   "FrontAxis Set Angle must remain unsupported");
        }
    }

    expect(servo_service_enabled_mask(&fixture.servo_service)
               == SERVO_DESCRIPTOR_SUPPORTED_MASK,
           "all five semantic Enable requests must set the fixed mask");
}

static void test_duplicate_success_is_replayed(void)
{
    fixture_t fixture;
    uint8_t mask_payload[2];
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t first;
    protocol_dispatcher_outcome_t second;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);
    write_le16(mask_payload,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);
    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        10U,
        mask_payload,
        sizeof(mask_payload));

    first = handle(&fixture, &frame, 0U);
    second = handle(&fixture, &frame, 100U);

    expect(first.result == RBP2_RESULT_OK,
           "first successful command should return OK");
    expect(second.result == RBP2_RESULT_OK,
           "duplicate should replay cached result");
    expect(fixture.driver.start_calls == 1U,
           "duplicate must not repeat Servo side effect");
}

static void test_heartbeat_does_not_evict_action_cache(void)
{
    fixture_t fixture;
    uint8_t mask_payload[2];
    rbp2_frame_t enable_frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);
    write_le16(mask_payload,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);
    enable_frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        20U,
        mask_payload,
        sizeof(mask_payload));

    outcome = handle(&fixture, &enable_frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "setup Enable should succeed");

    accept_heartbeat(&fixture, 20U, 100U, 100U);
    accept_heartbeat(&fixture, 20U, 200U, 200U);
    outcome = handle(&fixture, &enable_frame, 200U);

    expect(outcome.result == RBP2_RESULT_OK,
           "actuator retry should replay cached result after Heartbeats");
    expect(fixture.driver.start_calls == 1U,
           "Heartbeats must not evict successful action cache");
}

static void test_failed_command_is_not_cached(void)
{
    fixture_t fixture;
    uint8_t mask_payload[2];
    uint8_t invalid_pwm_payload[3] = {1U, 0U, 0U};
    uint8_t valid_pwm_payload[4] = {1U, 0U, 0x40U, 0x06U};
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);
    write_le16(mask_payload,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);
    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        2U,
        mask_payload,
        sizeof(mask_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "failed-command setup Enable should succeed");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_PWM,
        30U,
        invalid_pwm_payload,
        sizeof(invalid_pwm_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_INVALID_PAYLOAD,
           "first malformed command should fail validation");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_PWM,
        30U,
        valid_pwm_payload,
        sizeof(valid_pwm_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "corrected command should execute after failed command");
    expect(fixture.driver.last_pulse_us == 1600U,
           "corrected command should reach Servo service");
}

static void test_same_sequence_different_type_executes(void)
{
    fixture_t fixture;
    uint8_t mask_payload[2];
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);
    write_le16(mask_payload,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);

    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        40U,
        mask_payload,
        sizeof(mask_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "same-sequence setup Enable should succeed");

    frame = make_frame(
        RBP2_MSG_SERVO_DISABLE,
        40U,
        mask_payload,
        sizeof(mask_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "same sequence with different type should execute");
    expect(fixture.driver.stop_calls == 1U,
           "different message type must not be treated as duplicate");
}

static void test_cache_invalidation_allows_retry(void)
{
    fixture_t fixture;
    uint8_t mask_payload[2];
    uint8_t pwm_payload[4];
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);
    write_le16(mask_payload,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);
    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        49U,
        mask_payload,
        sizeof(mask_payload));

    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "cache invalidation setup Enable should succeed");

    pwm_payload[0] = 1U;
    pwm_payload[1] = SERVO_ID_FRONT_RIGHT;
    write_le16(&pwm_payload[2], 1600U);
    frame = make_frame(
        RBP2_MSG_SET_SERVO_PWM,
        50U,
        pwm_payload,
        sizeof(pwm_payload));
    const unsigned int writes_before = fixture.driver.write_calls;
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "cache invalidation setup PWM should succeed");
    outcome = handle(&fixture, &frame, 0U);
    expect(fixture.driver.write_calls == writes_before + 1U,
           "cached PWM retry must not repeat the Servo side effect");

    protocol_dispatcher_invalidate_action_cache(&fixture.dispatcher);
    outcome = handle(&fixture, &frame, 0U);

    expect(outcome.result == RBP2_RESULT_OK,
           "command after cache invalidation should execute");
    expect(fixture.driver.write_calls == writes_before + 2U,
           "cache invalidation must permit the PWM side effect again");
}

static void test_result_mappings(void)
{
    fixture_t fixture;
    uint8_t unsupported_mask[2] = {0x20U, 0x00U};
    uint8_t valid_pwm_payload[4] = {1U, 0U, 0xF0U, 0x05U};
    uint8_t out_of_range_angle[4] = {1U, 0U, 0x29U, 0x23U};
    uint8_t valid_mask[2];
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);
    accept_heartbeat(&fixture, 1U, 0U, 0U);

    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        2U,
        unsupported_mask,
        sizeof(unsupported_mask));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_UNSUPPORTED_SERVO,
           "unsupported Servo result should map correctly");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_PWM,
        3U,
        valid_pwm_payload,
        sizeof(valid_pwm_payload));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_SERVO_NOT_ENABLED,
           "ServoNotEnabled result should map correctly");

    write_le16(valid_mask,
               servo_descriptor_for_id(SERVO_ID_FRONT_RIGHT)->mask);
    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        4U,
        valid_mask,
        sizeof(valid_mask));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OK,
           "mapping setup Enable should succeed");

    frame = make_frame(
        RBP2_MSG_SET_SERVO_ANGLE,
        5U,
        out_of_range_angle,
        sizeof(out_of_range_angle));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_OUT_OF_RANGE,
           "OutOfRange result should map correctly");

    fixture_init(&fixture);
    fixture.driver.start_result = false;
    accept_heartbeat(&fixture, 1U, 0U, 0U);
    frame = make_frame(
        RBP2_MSG_SERVO_ENABLE,
        6U,
        valid_mask,
        sizeof(valid_mask));
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_HARDWARE_FAILURE,
           "HardwareFailure result should map correctly");
}

static void test_unknown_messages_are_invalid_payload(void)
{
    fixture_t fixture;
    rbp2_frame_t frame;
    protocol_dispatcher_outcome_t outcome;

    fixture_init(&fixture);

    frame = make_frame(0x7FU, 1U, NULL, 0U);
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_INVALID_PAYLOAD,
           "unknown message should map to InvalidPayload");

    frame = make_frame(RBP2_MSG_ERROR, 2U, NULL, 0U);
    outcome = handle(&fixture, &frame, 0U);
    expect(outcome.result == RBP2_RESULT_INVALID_PAYLOAD,
           "reserved Error message must remain InvalidPayload");
}

int main(void)
{
    test_valid_heartbeat();
    test_invalid_heartbeat_does_not_refresh_safety();
    test_host_alive_validation_order();
    test_servo_commands();
    test_all_semantic_servo_ids_route();
    test_duplicate_success_is_replayed();
    test_heartbeat_does_not_evict_action_cache();
    test_failed_command_is_not_cached();
    test_same_sequence_different_type_executes();
    test_cache_invalidation_allows_retry();
    test_result_mappings();
    test_unknown_messages_are_invalid_payload();

    if (failures == 0)
    {
        (void)puts("All firmware Protocol Dispatcher tests passed");
    }

    return failures == 0 ? 0 : 1;
}
