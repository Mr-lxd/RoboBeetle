#include "jy901s_parser.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static int16_t decode_signed_le(
    const uint8_t frame[JY901S_FRAME_LENGTH],
    uint8_t offset)
{
    uint16_t raw = (uint16_t)frame[offset] |
                   (uint16_t)((uint16_t)frame[offset + 1U] << 8U);

    return (int16_t)raw;
}

static float scale_raw(int16_t raw, float full_scale)
{
    return ((float)raw / 32768.0f) * full_scale;
}

static void start_candidate(
    jy901s_parser_t *parser,
    uint8_t byte)
{
    parser->candidate[0] = byte;
    parser->candidate_length = 1U;
    ++parser->stats.header_count;
}

static void resync_after_bad_candidate(jy901s_parser_t *parser)
{
    for (uint8_t index = 1U;
         index < parser->candidate_length;
         ++index)
    {
        if (parser->candidate[index] == JY901S_FRAME_HEADER)
        {
            uint8_t suffix_length =
                (uint8_t)(parser->candidate_length - index);

            memmove(
                parser->candidate,
                &parser->candidate[index],
                suffix_length);
            parser->candidate_length = suffix_length;
            ++parser->stats.header_count;
            return;
        }
    }

    parser->candidate_length = 0U;
}

void jy901s_parser_init(jy901s_parser_t *parser)
{
    if (parser != NULL)
    {
        memset(parser, 0, sizeof *parser);
    }
}

jy901s_parser_event_t jy901s_parser_feed_byte(
    jy901s_parser_t *parser,
    uint8_t byte)
{
    uint16_t checksum = 0U;
    uint8_t type = 0U;

    if (parser == NULL)
    {
        return JY901S_PARSER_EVENT_NONE;
    }

    if (parser->candidate_length == 0U)
    {
        if (byte == JY901S_FRAME_HEADER)
        {
            start_candidate(parser, byte);
        }
        return JY901S_PARSER_EVENT_NONE;
    }

    parser->candidate[parser->candidate_length] = byte;
    ++parser->candidate_length;

    if (parser->candidate_length < JY901S_FRAME_LENGTH)
    {
        return JY901S_PARSER_EVENT_NONE;
    }

    for (uint8_t index = 0U; index < 10U; ++index)
    {
        checksum = (uint16_t)(checksum + parser->candidate[index]);
    }

    if ((uint8_t)checksum !=
        parser->candidate[JY901S_FRAME_LENGTH - 1U])
    {
        ++parser->stats.checksum_error_count;
        resync_after_bad_candidate(parser);
        return JY901S_PARSER_EVENT_NONE;
    }

    type = parser->candidate[1];
    ++parser->stats.valid_frame_count;
    parser->candidate_length = 0U;

    switch (type)
    {
        case JY901S_TYPE_ACC:
            parser->state.acc_x_g =
                scale_raw(decode_signed_le(parser->candidate, 2U), 16.0f);
            parser->state.acc_y_g =
                scale_raw(decode_signed_le(parser->candidate, 4U), 16.0f);
            parser->state.acc_z_g =
                scale_raw(decode_signed_le(parser->candidate, 6U), 16.0f);
            parser->state.acc_valid = true;
            ++parser->stats.acc_frame_count;
            return JY901S_PARSER_EVENT_ACC;

        case JY901S_TYPE_GYRO:
            parser->state.gyro_x_dps =
                scale_raw(decode_signed_le(parser->candidate, 2U), 2000.0f);
            parser->state.gyro_y_dps =
                scale_raw(decode_signed_le(parser->candidate, 4U), 2000.0f);
            parser->state.gyro_z_dps =
                scale_raw(decode_signed_le(parser->candidate, 6U), 2000.0f);
            parser->state.gyro_valid = true;
            ++parser->stats.gyro_frame_count;
            return JY901S_PARSER_EVENT_GYRO;

        case JY901S_TYPE_ANGLE:
            parser->state.angle_roll_deg =
                scale_raw(decode_signed_le(parser->candidate, 2U), 180.0f);
            parser->state.angle_pitch_deg =
                scale_raw(decode_signed_le(parser->candidate, 4U), 180.0f);
            parser->state.angle_yaw_deg =
                scale_raw(decode_signed_le(parser->candidate, 6U), 180.0f);
            parser->state.angle_valid = true;
            ++parser->stats.angle_frame_count;
            return JY901S_PARSER_EVENT_ANGLE;

        case JY901S_TYPE_MAG:
            ++parser->stats.mag_frame_count;
            return JY901S_PARSER_EVENT_MAG_IGNORED;

        default:
            ++parser->stats.unsupported_frame_count;
            return JY901S_PARSER_EVENT_UNSUPPORTED;
    }
}

void jy901s_parser_get_state(
    const jy901s_parser_t *parser,
    jy901s_imu_state_t *state)
{
    if ((parser != NULL) && (state != NULL))
    {
        memcpy(state, &parser->state, sizeof *state);
    }
}

void jy901s_parser_get_stats(
    const jy901s_parser_t *parser,
    jy901s_parser_stats_t *stats)
{
    if ((parser != NULL) && (stats != NULL))
    {
        memcpy(stats, &parser->stats, sizeof *stats);
    }
}
