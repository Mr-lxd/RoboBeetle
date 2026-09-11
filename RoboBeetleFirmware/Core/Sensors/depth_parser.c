#include "depth_parser.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static bool is_decimal_digit(uint8_t byte)
{
    return (byte >= (uint8_t)'0') && (byte <= (uint8_t)'9');
}

static bool match_literal(
    const uint8_t *line,
    size_t length,
    size_t *offset,
    const char *literal)
{
    size_t literal_length = strlen(literal);

    if ((*offset > length) ||
        (literal_length > (length - *offset)))
    {
        return false;
    }

    if (memcmp(&line[*offset], literal, literal_length) != 0)
    {
        return false;
    }

    *offset += literal_length;
    return true;
}

static bool parse_fixed_decimal(
    const uint8_t *line,
    size_t length,
    size_t *offset,
    int64_t *value)
{
    bool negative = false;
    uint64_t magnitude = 0U;
    uint8_t fractional_tens;
    uint8_t fractional_ones;
    size_t index = *offset;

    if (index >= length)
    {
        return false;
    }

    if ((line[index] == (uint8_t)'+') ||
        (line[index] == (uint8_t)'-'))
    {
        negative = line[index] == (uint8_t)'-';
        ++index;
    }

    if ((index >= length) || !is_decimal_digit(line[index]))
    {
        return false;
    }

    while ((index < length) && is_decimal_digit(line[index]))
    {
        uint8_t digit = (uint8_t)(line[index] - (uint8_t)'0');

        if (magnitude >
            (((uint64_t)INT64_MAX - (uint64_t)digit) / 10U))
        {
            return false;
        }

        magnitude = (magnitude * 10U) + (uint64_t)digit;
        ++index;
    }

    if ((index >= length) || (line[index] != (uint8_t)'.'))
    {
        return false;
    }
    ++index;

    if ((index + 1U >= length) ||
        !is_decimal_digit(line[index]) ||
        !is_decimal_digit(line[index + 1U]))
    {
        return false;
    }

    fractional_tens = (uint8_t)(line[index] - (uint8_t)'0');
    fractional_ones = (uint8_t)(line[index + 1U] - (uint8_t)'0');
    index += 2U;

    {
        uint64_t fractional_value =
            ((uint64_t)fractional_tens * 10U) +
            (uint64_t)fractional_ones;

        if (magnitude >
            (((uint64_t)INT64_MAX - fractional_value) / 100U))
        {
            return false;
        }

        magnitude = (magnitude * 100U) + fractional_value;
    }

    if (magnitude > (uint64_t)INT64_MAX)
    {
        return false;
    }

    *value = negative ? -(int64_t)magnitude : (int64_t)magnitude;
    *offset = index;
    return true;
}

static bool convert_depth_to_millimetres(
    int64_t metres_centi,
    int32_t *depth_mm)
{
    if ((metres_centi > ((int64_t)INT32_MAX / 10)) ||
        (metres_centi < ((int64_t)INT32_MIN / 10)))
    {
        return false;
    }

    *depth_mm = (int32_t)(metres_centi * 10);
    return true;
}

static bool convert_temperature_to_centi_degrees(
    int64_t temperature_centi,
    int16_t *temperature_centi_c)
{
    if ((temperature_centi > (int64_t)INT16_MAX) ||
        (temperature_centi < (int64_t)INT16_MIN))
    {
        return false;
    }

    *temperature_centi_c = (int16_t)temperature_centi;
    return true;
}

static bool parse_canonical_line(
    const uint8_t *line,
    size_t length,
    int32_t *depth_mm,
    int16_t *temperature_centi_c)
{
    size_t offset = 0U;
    int64_t depth_metres_centi = 0;
    int64_t temperature_centi = 0;

    if (!match_literal(line, length, &offset, "Depth:") ||
        !parse_fixed_decimal(
            line,
            length,
            &offset,
            &depth_metres_centi) ||
        ((!match_literal(line, length, &offset, "m Temp:")) &&
         (!match_literal(line, length, &offset, "m Temp="))) ||
        !parse_fixed_decimal(
            line,
            length,
            &offset,
            &temperature_centi) ||
        !match_literal(line, length, &offset, "C") ||
        (offset != length) ||
        !convert_depth_to_millimetres(depth_metres_centi, depth_mm) ||
        !convert_temperature_to_centi_degrees(
            temperature_centi,
            temperature_centi_c))
    {
        return false;
    }

    return true;
}

static bool parse_line(
    const uint8_t *line,
    size_t length,
    depth_parser_state_t *state,
    uint32_t now_ms)
{
    int32_t depth_mm = 0;
    int16_t temperature_centi_c = 0;
    bool accepted = parse_canonical_line(
        line,
        length,
        &depth_mm,
        &temperature_centi_c);

    if (!accepted)
    {
        return false;
    }

    state->depth_valid = true;
    state->temperature_valid = true;
    state->depth_mm = depth_mm;
    state->temperature_centi_c = temperature_centi_c;
    state->last_valid_sample_ms = now_ms;
    return true;
}

static void reset_line_collector(depth_parser_t *parser)
{
    parser->line_length = 0U;
    parser->saw_carriage_return = false;
    parser->discarding_overlong_line = false;
    parser->discarded_carriage_return = false;
}

static bool finish_line(depth_parser_t *parser, uint32_t now_ms)
{
    bool accepted = parse_line(
        parser->line,
        (size_t)parser->line_length,
        &parser->state,
        now_ms);

    if (accepted)
    {
        ++parser->stats.valid_line_count;
    }
    else
    {
        ++parser->stats.parse_error_count;
    }

    reset_line_collector(parser);
    return accepted;
}

void depth_parser_init(depth_parser_t *parser)
{
    if (parser != NULL)
    {
        memset(parser, 0, sizeof *parser);
    }
}

bool depth_parser_feed_byte(
    depth_parser_t *parser,
    uint8_t byte,
    uint32_t now_ms)
{
    if (parser == NULL)
    {
        return false;
    }

    if (parser->discarding_overlong_line)
    {
        /* Any LF terminates the discarded malformed line, including bare LF. */
        if (byte == (uint8_t)'\n')
        {
            ++parser->stats.overlong_line_count;
            reset_line_collector(parser);
        }
        else
        {
            parser->discarded_carriage_return =
                byte == (uint8_t)'\r';
        }
        return false;
    }

    if (parser->saw_carriage_return)
    {
        if (byte == (uint8_t)'\n')
        {
            return finish_line(parser, now_ms);
        }

        ++parser->stats.parse_error_count;
        reset_line_collector(parser);
    }

    if (byte == (uint8_t)'\r')
    {
        parser->saw_carriage_return = true;
        return false;
    }

    if (byte == (uint8_t)'\n')
    {
        ++parser->stats.parse_error_count;
        reset_line_collector(parser);
        return false;
    }

    if (parser->line_length >= DEPTH_PARSER_MAX_LINE_LENGTH)
    {
        parser->discarding_overlong_line = true;
        parser->discarded_carriage_return = false;
        return false;
    }

    parser->line[parser->line_length] = byte;
    ++parser->line_length;
    return false;
}

void depth_parser_get_state(
    const depth_parser_t *parser,
    depth_parser_state_t *state)
{
    if ((parser != NULL) && (state != NULL))
    {
        *state = parser->state;
    }
}

void depth_parser_get_stats(
    const depth_parser_t *parser,
    depth_parser_stats_t *stats)
{
    if ((parser != NULL) && (stats != NULL))
    {
        *stats = parser->stats;
    }
}
