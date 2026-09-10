#include "jy901s_parser.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void expect_close(
    float actual,
    float expected,
    const char *message)
{
    expect(fabsf(actual - expected) <= 0.001f, message);
}

static void expect_event(
    jy901s_parser_event_t actual,
    jy901s_parser_event_t expected,
    const char *message)
{
    expect(actual == expected, message);
}

static void write_raw_le(uint8_t *frame, uint8_t offset, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    frame[offset] = (uint8_t)(raw & 0xFFU);
    frame[(uint8_t)(offset + 1U)] =
        (uint8_t)((raw >> 8U) & 0xFFU);
}

static void make_frame(
    uint8_t type,
    int16_t x,
    int16_t y,
    int16_t z,
    uint8_t frame[JY901S_FRAME_LENGTH])
{
    uint16_t sum = 0U;

    frame[0] = JY901S_FRAME_HEADER;
    frame[1] = type;
    write_raw_le(frame, 2U, x);
    write_raw_le(frame, 4U, y);
    write_raw_le(frame, 6U, z);
    write_raw_le(frame, 8U, 0);

    for (uint8_t index = 0U; index < 10U; ++index)
    {
        sum = (uint16_t)(sum + frame[index]);
    }
    frame[10] = (uint8_t)sum;
}

static jy901s_parser_event_t feed_frame(
    jy901s_parser_t *parser,
    const uint8_t frame[JY901S_FRAME_LENGTH])
{
    jy901s_parser_event_t event = JY901S_PARSER_EVENT_NONE;

    for (uint8_t index = 0U; index < JY901S_FRAME_LENGTH; ++index)
    {
        jy901s_parser_event_t current =
            jy901s_parser_feed_byte(parser, frame[index]);

        if (current != JY901S_PARSER_EVENT_NONE)
        {
            event = current;
        }
    }

    return event;
}

static void expect_acc_state(
    const jy901s_imu_state_t *state,
    float x,
    float y,
    float z)
{
    expect(state->acc_valid, "Acc state should be valid");
    expect_close(state->acc_x_g, x, "Acc X scaling differs");
    expect_close(state->acc_y_g, y, "Acc Y scaling differs");
    expect_close(state->acc_z_g, z, "Acc Z scaling differs");
}

static void expect_gyro_state(
    const jy901s_imu_state_t *state,
    float x,
    float y,
    float z)
{
    expect(state->gyro_valid, "Gyro state should be valid");
    expect_close(state->gyro_x_dps, x, "Gyro X scaling differs");
    expect_close(state->gyro_y_dps, y, "Gyro Y scaling differs");
    expect_close(state->gyro_z_dps, z, "Gyro Z scaling differs");
}

static void expect_angle_state(
    const jy901s_imu_state_t *state,
    float roll,
    float pitch,
    float yaw)
{
    expect(state->angle_valid, "Angle state should be valid");
    expect_close(state->angle_roll_deg, roll, "Angle roll scaling differs");
    expect_close(
        state->angle_pitch_deg,
        pitch,
        "Angle pitch scaling differs");
    expect_close(state->angle_yaw_deg, yaw, "Angle yaw scaling differs");
}

static void test_acc_decode(void)
{
    jy901s_parser_t parser;
    jy901s_imu_state_t state;
    jy901s_parser_stats_t stats;
    uint8_t frame[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_ACC, 16384, -8192, 0, frame);

    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_ACC,
        "Acc frame event differs");
    jy901s_parser_get_state(&parser, &state);
    jy901s_parser_get_stats(&parser, &stats);

    expect_acc_state(&state, 8.0f, -4.0f, 0.0f);
    expect(!state.gyro_valid, "Gyro should remain invalid before Gyro frame");
    expect(!state.angle_valid, "Angle should remain invalid before Angle frame");
    expect(stats.valid_frame_count == 1U, "Acc valid count differs");
    expect(stats.acc_frame_count == 1U, "Acc domain count differs");
    expect(stats.checksum_error_count == 0U, "Acc checksum should be valid");
}

static void test_gyro_and_angle_decode(void)
{
    jy901s_parser_t parser;
    jy901s_imu_state_t state;
    uint8_t frame[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_GYRO, 16384, -8192, 0, frame);
    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_GYRO,
        "Gyro frame event differs");

    make_frame(JY901S_TYPE_ANGLE, 16384, -8192, 0, frame);
    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_ANGLE,
        "Angle frame event differs");

    jy901s_parser_get_state(&parser, &state);
    expect_gyro_state(&state, 1000.0f, -500.0f, 0.0f);
    expect_angle_state(&state, 90.0f, -45.0f, 0.0f);
}

static void test_garbage_split_and_concatenated_frames(void)
{
    jy901s_parser_t parser;
    jy901s_imu_state_t state;
    uint8_t acc[JY901S_FRAME_LENGTH];
    uint8_t gyro[JY901S_FRAME_LENGTH];
    uint8_t angle[JY901S_FRAME_LENGTH];
    const uint8_t garbage[] = {0x00U, 0x12U, 0xAAU};

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_ACC, 16384, 0, 0, acc);
    make_frame(JY901S_TYPE_GYRO, 16384, 0, 0, gyro);
    make_frame(JY901S_TYPE_ANGLE, 16384, 0, 0, angle);

    for (uint8_t index = 0U; index < sizeof garbage; ++index)
    {
        expect(
            jy901s_parser_feed_byte(&parser, garbage[index]) ==
                JY901S_PARSER_EVENT_NONE,
            "leading garbage should not emit an event");
    }

    for (uint8_t index = 0U; index < 5U; ++index)
    {
        expect(
            jy901s_parser_feed_byte(&parser, acc[index]) ==
                JY901S_PARSER_EVENT_NONE,
            "split Acc prefix should not emit an event");
    }

    for (uint8_t index = 5U; index < JY901S_FRAME_LENGTH; ++index)
    {
        (void)jy901s_parser_feed_byte(&parser, acc[index]);
    }
    expect_event(
        feed_frame(&parser, gyro),
        JY901S_PARSER_EVENT_GYRO,
        "concatenated Gyro frame was not accepted");
    expect_event(
        feed_frame(&parser, angle),
        JY901S_PARSER_EVENT_ANGLE,
        "concatenated Angle frame was not accepted");

    jy901s_parser_get_state(&parser, &state);
    expect_acc_state(&state, 8.0f, 0.0f, 0.0f);
    expect_gyro_state(&state, 1000.0f, 0.0f, 0.0f);
    expect_angle_state(&state, 90.0f, 0.0f, 0.0f);
}

static void test_bad_checksum_and_following_frame(void)
{
    jy901s_parser_t parser;
    jy901s_imu_state_t state;
    jy901s_parser_stats_t stats;
    uint8_t acc[JY901S_FRAME_LENGTH];
    uint8_t gyro[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_ACC, 16384, 0, 0, acc);
    make_frame(JY901S_TYPE_GYRO, 8192, 0, 0, gyro);
    acc[10] ^= 0x01U;

    expect_event(
        feed_frame(&parser, acc),
        JY901S_PARSER_EVENT_NONE,
        "bad checksum should not emit a valid event");
    expect_event(
        feed_frame(&parser, gyro),
        JY901S_PARSER_EVENT_GYRO,
        "valid frame after bad checksum was not accepted");

    jy901s_parser_get_state(&parser, &state);
    jy901s_parser_get_stats(&parser, &stats);
    expect(!state.acc_valid, "bad Acc frame must not update Acc state");
    expect_gyro_state(&state, 500.0f, 0.0f, 0.0f);
    expect(stats.checksum_error_count == 1U, "checksum error count differs");
}

static void test_dropped_byte_resynchronizes(void)
{
    jy901s_parser_t parser;
    jy901s_parser_stats_t stats;
    jy901s_imu_state_t state;
    uint8_t acc[JY901S_FRAME_LENGTH];
    uint8_t angle[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_ACC, 16384, 0, 0, acc);
    make_frame(JY901S_TYPE_ANGLE, 8192, 0, 0, angle);

    for (uint8_t index = 0U; index < JY901S_FRAME_LENGTH; ++index)
    {
        if (index != 5U)
        {
            (void)jy901s_parser_feed_byte(&parser, acc[index]);
        }
    }

    expect_event(
        feed_frame(&parser, angle),
        JY901S_PARSER_EVENT_ANGLE,
        "parser did not resynchronize after dropped byte");

    jy901s_parser_get_state(&parser, &state);
    jy901s_parser_get_stats(&parser, &stats);
    expect_angle_state(&state, 45.0f, 0.0f, 0.0f);
    expect(stats.checksum_error_count >= 1U,
           "dropped byte should produce a rejected candidate");
}

static void test_mag_is_known_ignored_and_sync_continues(void)
{
    jy901s_parser_t parser;
    jy901s_imu_state_t before;
    jy901s_imu_state_t after;
    jy901s_parser_stats_t stats;
    uint8_t frame[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_ACC, 8192, 0, 0, frame);
    (void)feed_frame(&parser, frame);
    make_frame(JY901S_TYPE_GYRO, 8192, 0, 0, frame);
    (void)feed_frame(&parser, frame);
    make_frame(JY901S_TYPE_ANGLE, 8192, 0, 0, frame);
    (void)feed_frame(&parser, frame);
    jy901s_parser_get_state(&parser, &before);

    make_frame(JY901S_TYPE_MAG, -1234, 2345, -3456, frame);
    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_MAG_IGNORED,
        "Mag frame should be a known-ignored event");

    jy901s_parser_get_state(&parser, &after);
    jy901s_parser_get_stats(&parser, &stats);
    expect(memcmp(&before, &after, sizeof before) == 0,
           "Mag frame must not change supported state");
    expect(stats.valid_frame_count == 4U,
           "Mag should increment overall valid-frame count");
    expect(stats.mag_frame_count == 1U, "Mag frame count differs");
    expect(stats.unsupported_frame_count == 0U,
           "Mag must not increment unsupported count");

    make_frame(JY901S_TYPE_ACC, 16384, 0, 0, frame);
    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_ACC,
        "parser lost sync after known Mag frame");
}

static void test_unknown_valid_type_is_unsupported_and_sync_continues(void)
{
    jy901s_parser_t parser;
    jy901s_parser_stats_t stats;
    uint8_t frame[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(0x60U, 100, 200, 300, frame);
    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_UNSUPPORTED,
        "unknown valid type event differs");

    jy901s_parser_get_stats(&parser, &stats);
    expect(stats.valid_frame_count == 1U,
           "unknown valid frame should count as valid checksum frame");
    expect(stats.unsupported_frame_count == 1U,
           "unknown valid type should increment unsupported count");
    expect(stats.mag_frame_count == 0U,
           "unknown type must not increment Mag count");

    make_frame(JY901S_TYPE_ANGLE, 8192, 0, 0, frame);
    expect_event(
        feed_frame(&parser, frame),
        JY901S_PARSER_EVENT_ANGLE,
        "parser lost sync after unknown valid type");
}

static void test_bad_checksum_does_not_change_state(void)
{
    jy901s_parser_t parser;
    jy901s_imu_state_t before;
    jy901s_imu_state_t after;
    uint8_t frame[JY901S_FRAME_LENGTH];

    jy901s_parser_init(&parser);
    make_frame(JY901S_TYPE_ACC, 4096, -4096, 1024, frame);
    (void)feed_frame(&parser, frame);
    jy901s_parser_get_state(&parser, &before);

    make_frame(JY901S_TYPE_ACC, 16384, 16384, 16384, frame);
    frame[10] ^= 0x01U;
    (void)feed_frame(&parser, frame);

    jy901s_parser_get_state(&parser, &after);
    expect(memcmp(&before, &after, sizeof before) == 0,
           "bad checksum changed supported state");
}

int main(void)
{
    test_acc_decode();
    test_gyro_and_angle_decode();
    test_garbage_split_and_concatenated_frames();
    test_bad_checksum_and_following_frame();
    test_dropped_byte_resynchronizes();
    test_mag_is_known_ignored_and_sync_continues();
    test_unknown_valid_type_is_unsupported_and_sync_continues();
    test_bad_checksum_does_not_change_state();

    if (failures == 0)
    {
        (void)puts("All JY901S parser tests passed");
    }

    return failures == 0 ? 0 : 1;
}
