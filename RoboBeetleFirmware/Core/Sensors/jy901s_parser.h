#ifndef ROBOBEETLE_JY901S_PARSER_H
#define ROBOBEETLE_JY901S_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#define JY901S_FRAME_LENGTH 11U
#define JY901S_FRAME_HEADER 0x55U
#define JY901S_TYPE_ACC 0x51U
#define JY901S_TYPE_GYRO 0x52U
#define JY901S_TYPE_ANGLE 0x53U
#define JY901S_TYPE_MAG 0x54U

typedef struct
{
    bool acc_valid;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    bool gyro_valid;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    bool angle_valid;
    float angle_roll_deg;
    float angle_pitch_deg;
    float angle_yaw_deg;
} jy901s_imu_state_t;

typedef struct
{
    uint32_t header_count;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t mag_frame_count;
    uint32_t unsupported_frame_count;
    uint32_t acc_frame_count;
    uint32_t gyro_frame_count;
    uint32_t angle_frame_count;
} jy901s_parser_stats_t;

typedef enum
{
    JY901S_PARSER_EVENT_NONE = 0,
    JY901S_PARSER_EVENT_ACC,
    JY901S_PARSER_EVENT_GYRO,
    JY901S_PARSER_EVENT_ANGLE,
    JY901S_PARSER_EVENT_MAG_IGNORED,
    JY901S_PARSER_EVENT_UNSUPPORTED
} jy901s_parser_event_t;

typedef struct
{
    uint8_t candidate[JY901S_FRAME_LENGTH];
    uint8_t candidate_length;
    jy901s_imu_state_t state;
    jy901s_parser_stats_t stats;
} jy901s_parser_t;

void jy901s_parser_init(jy901s_parser_t *parser);
jy901s_parser_event_t jy901s_parser_feed_byte(
    jy901s_parser_t *parser,
    uint8_t byte);
void jy901s_parser_get_state(
    const jy901s_parser_t *parser,
    jy901s_imu_state_t *state);
void jy901s_parser_get_stats(
    const jy901s_parser_t *parser,
    jy901s_parser_stats_t *stats);

#endif /* ROBOBEETLE_JY901S_PARSER_H */
