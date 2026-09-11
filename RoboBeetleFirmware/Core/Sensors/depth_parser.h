#ifndef ROBOBEETLE_DEPTH_PARSER_H
#define ROBOBEETLE_DEPTH_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#define DEPTH_PARSER_MAX_LINE_LENGTH 64U

typedef struct
{
    bool depth_valid;
    bool temperature_valid;
    int32_t depth_mm;
    int16_t temperature_centi_c;
    uint32_t last_valid_sample_ms;
} depth_parser_state_t;

typedef struct
{
    uint32_t valid_line_count;
    uint32_t parse_error_count;
    uint32_t overlong_line_count;
} depth_parser_stats_t;

typedef struct
{
    uint8_t line[DEPTH_PARSER_MAX_LINE_LENGTH];
    uint16_t line_length;
    bool saw_carriage_return;
    bool discarding_overlong_line;
    bool discarded_carriage_return;
    depth_parser_state_t state;
    depth_parser_stats_t stats;
} depth_parser_t;

void depth_parser_init(depth_parser_t *parser);
bool depth_parser_feed_byte(
    depth_parser_t *parser,
    uint8_t byte,
    uint32_t now_ms);
void depth_parser_get_state(
    const depth_parser_t *parser,
    depth_parser_state_t *state);
void depth_parser_get_stats(
    const depth_parser_t *parser,
    depth_parser_stats_t *stats);

#endif /* ROBOBEETLE_DEPTH_PARSER_H */
