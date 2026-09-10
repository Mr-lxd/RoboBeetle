#include "depth_parser.h"

#include <stdbool.h>
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

static size_t feed_text(
    depth_parser_t *parser,
    const char *text,
    uint32_t now_ms)
{
    size_t accepted = 0U;
    size_t length = strlen(text);

    for (size_t index = 0U; index < length; ++index)
    {
        if (depth_parser_feed_byte(
                parser,
                (uint8_t)text[index],
                now_ms))
        {
            ++accepted;
        }
    }

    return accepted;
}

static void expect_sample(
    const depth_parser_state_t *state,
    int32_t depth_mm,
    int16_t temperature_centi_c,
    uint32_t last_valid_sample_ms)
{
    expect(state->depth_valid, "depth should be valid");
    expect(state->temperature_valid, "temperature should be valid");
    expect(state->depth_mm == depth_mm, "depth millimetres differ");
    expect(
        state->temperature_centi_c == temperature_centi_c,
        "temperature centi-degrees differ");
    expect(
        state->last_valid_sample_ms == last_valid_sample_ms,
        "last-valid timestamp differs");
}

static void test_canonical_line_decodes_fixed_point_values(void)
{
    depth_parser_t parser;
    depth_parser_state_t state;
    depth_parser_stats_t stats;

    depth_parser_init(&parser);

    expect(
        feed_text(&parser, "Depth:12.34m Temp:-5.67C\r\n", 1234U) == 1U,
        "canonical line should emit one accepted sample");

    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 12340, -567, 1234U);
    expect(stats.valid_line_count == 1U, "canonical valid-line count differs");
    expect(stats.parse_error_count == 0U, "canonical line should not be an error");
    expect(stats.overlong_line_count == 0U, "canonical line should not be overlong");
}

static void test_compact_line_accepts_optional_signs(void)
{
    depth_parser_t parser;
    depth_parser_state_t state;

    depth_parser_init(&parser);

    expect(
        feed_text(&parser, "T=+23.45D=-0.60\r\n", 77U) == 1U,
        "compact line should emit one accepted sample");

    depth_parser_get_state(&parser, &state);
    expect_sample(&state, -600, 2345, 77U);
}

static void test_split_crlf_and_concatenated_lines_are_supported(void)
{
    static const char first[] = "Depth:1.20m Temp:3.40C\r\n";
    static const char second[] = "T=-5.60D=7.80\r\n";
    depth_parser_t parser;
    depth_parser_state_t state;
    depth_parser_stats_t stats;

    depth_parser_init(&parser);

    for (size_t index = 0U; index < sizeof first - 1U; ++index)
    {
        expect(
            !depth_parser_feed_byte(
                &parser,
                (uint8_t)first[index],
                100U) ||
                (index == sizeof first - 2U),
            "split canonical feed emitted at an unexpected byte");
    }

    for (size_t index = 0U; index < sizeof second - 1U; ++index)
    {
        (void)depth_parser_feed_byte(&parser, (uint8_t)second[index], 200U);
    }

    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 7800, -560, 200U);
    expect(stats.valid_line_count == 2U, "concatenated valid-line count differs");
    expect(stats.parse_error_count == 0U, "split lines should not be errors");
}

static void test_exact_grammar_rejects_guesses_and_separators(void)
{
    depth_parser_t parser;
    depth_parser_stats_t stats;

    depth_parser_init(&parser);
    (void)feed_text(&parser, "Depth:1.23m Temp=4.56C\r\n", 1U);
    (void)feed_text(&parser, "Depth:1.23m Temp:4.56C\n", 2U);
    (void)feed_text(&parser, "Depth:1.23m Temp:4.56Cextra\r\n", 3U);
    (void)feed_text(&parser, "xxDepth:1.23m Temp:4.56C\r\n", 4U);
    (void)feed_text(&parser, "T=1.23 D=4.56\r\n", 5U);
    (void)feed_text(&parser, "T=1.2D=4.56\r\n", 6U);
    (void)feed_text(&parser, "T=1.234D=4.56\r\n", 7U);

    depth_parser_get_stats(&parser, &stats);
    expect(stats.valid_line_count == 0U, "guessed grammar must not be valid");
    expect(stats.parse_error_count == 7U, "malformed-line count differs");
    expect(stats.overlong_line_count == 0U, "grammar errors must not be overlong");
}

static void test_fixed_point_overflow_is_rejected_without_state_change(void)
{
    depth_parser_t parser;
    depth_parser_state_t state;
    depth_parser_stats_t stats;

    depth_parser_init(&parser);
    (void)feed_text(&parser, "Depth:2.00m Temp:3.00C\r\n", 10U);
    (void)feed_text(&parser, "Depth:2147483.65m Temp:3.00C\r\n", 20U);
    (void)feed_text(&parser, "Depth:2.00m Temp:327.68C\r\n", 30U);
    (void)feed_text(&parser, "Depth:-2.00m Temp:-327.69C\r\n", 40U);
    (void)feed_text(&parser, "T=184467440737095517.00D=0.00\r\n", 50U);

    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 2000, 300, 10U);
    expect(stats.valid_line_count == 1U, "overflow lines must not be valid");
    expect(stats.parse_error_count == 4U, "overflow error count differs");
}

static void test_overlong_line_is_discarded_and_next_line_recovers(void)
{
    uint8_t overlong[DEPTH_PARSER_MAX_LINE_LENGTH + 3U];
    depth_parser_t parser;
    depth_parser_state_t state;
    depth_parser_stats_t stats;

    for (size_t index = 0U; index < DEPTH_PARSER_MAX_LINE_LENGTH + 1U; ++index)
    {
        overlong[index] = (uint8_t)'X';
    }
    overlong[DEPTH_PARSER_MAX_LINE_LENGTH + 1U] = (uint8_t)'\r';
    overlong[DEPTH_PARSER_MAX_LINE_LENGTH + 2U] = (uint8_t)'\n';

    depth_parser_init(&parser);
    for (size_t index = 0U; index < sizeof overlong; ++index)
    {
        (void)depth_parser_feed_byte(&parser, overlong[index], 1U);
    }
    (void)feed_text(&parser, "T=9.87D=6.50\r\n", 2U);

    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 6500, 987, 2U);
    expect(stats.valid_line_count == 1U, "post-overlong line should be valid");
    expect(stats.parse_error_count == 0U, "overlong line must not be a parse error");
    expect(stats.overlong_line_count == 1U, "overlong-line count differs");
}

static void test_overlong_bare_lf_recovers_next_line(void)
{
    uint8_t overlong[DEPTH_PARSER_MAX_LINE_LENGTH + 2U];
    depth_parser_t parser;
    depth_parser_state_t state;
    depth_parser_stats_t stats;

    for (size_t index = 0U; index < DEPTH_PARSER_MAX_LINE_LENGTH + 1U; ++index)
    {
        overlong[index] = (uint8_t)'X';
    }
    overlong[DEPTH_PARSER_MAX_LINE_LENGTH + 1U] = (uint8_t)'\n';

    depth_parser_init(&parser);
    for (size_t index = 0U; index < sizeof overlong; ++index)
    {
        (void)depth_parser_feed_byte(&parser, overlong[index], 1U);
    }
    (void)feed_text(&parser, "Depth:4.32m Temp:1.23C\r\n", 2U);

    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 4320, 123, 2U);
    expect(stats.valid_line_count == 1U,
           "bare-LF recovery must preserve the next valid line");
    expect(stats.parse_error_count == 0U,
           "overlong bare-LF recovery must not add a parse error");
    expect(stats.overlong_line_count == 1U,
           "overlong bare-LF count differs");
}

static void test_invalid_line_does_not_replace_latest_sample(void)
{
    depth_parser_t parser;
    depth_parser_state_t state;
    depth_parser_stats_t stats;

    depth_parser_init(&parser);
    (void)feed_text(&parser, "T=1.11D=2.22\r\n", 100U);
    (void)feed_text(&parser, "Depth:9.99m Temp:8.88X\r\n", 200U);

    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 2220, 111, 100U);
    expect(stats.valid_line_count == 1U, "invalid line replaced valid-line count");
    expect(stats.parse_error_count == 1U, "invalid line error count differs");

    (void)feed_text(&parser, "Depth:3.33m Temp:-4.44C\r\n", 300U);
    depth_parser_get_state(&parser, &state);
    depth_parser_get_stats(&parser, &stats);
    expect_sample(&state, 3330, -444, 300U);
    expect(stats.valid_line_count == 2U, "parser did not recover after an invalid line");
    expect(stats.parse_error_count == 1U, "recovery changed parse-error count");
}

int main(void)
{
    test_canonical_line_decodes_fixed_point_values();
    test_compact_line_accepts_optional_signs();
    test_split_crlf_and_concatenated_lines_are_supported();
    test_exact_grammar_rejects_guesses_and_separators();
    test_fixed_point_overflow_is_rejected_without_state_change();
    test_overlong_line_is_discarded_and_next_line_recovers();
    test_overlong_bare_lf_recovers_next_line();
    test_invalid_line_does_not_replace_latest_sample();

    if (failures == 0)
    {
        (void)puts("All depth parser tests passed");
    }

    return failures == 0 ? 0 : 1;
}
