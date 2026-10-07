#ifndef RB_MOTION_STATE_CODEC_H
#define RB_MOTION_STATE_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RB_MOTION_STATE_HEADER_SIZE 16U
#define RB_MOTION_STATE_SAMPLE_SIZE 22U
#define RB_MOTION_STATE_MAX_SAMPLES 2U
#define RB_MOTION_STATE_MAX_PAYLOAD 60U
#define RB_MOTION_STATE_SCHEMA 1U

/* Fixed-point wire units: gyro 0.1 deg/s, roll/pitch 0.01 deg.
 * phase_u16 represents radians * 65536 / (2*pi). Ages: 65535 unknown. */
typedef struct {
    uint32_t mcu_ms;
    int16_t gyro_tenth_dps[3];
    int16_t roll_centidegrees, pitch_centidegrees;
    uint16_t phase_u16;
    uint8_t active_mode, target_mode, coordination;
    uint8_t backend, state;
    bool transition, phase_valid, gyro_valid, angle_valid;
    uint16_t angle_age_ms, phase_age_ms;
} rb_motion_state_sample_t;

typedef struct {
    uint8_t sample_count;
    uint16_t batch_seq;
    uint8_t fragment_index, fragment_count;
    uint32_t mcu_tx_ms, sampler_drop_total;
    rb_motion_state_sample_t samples[RB_MOTION_STATE_MAX_SAMPLES];
} rb_motion_state_batch_t;

/* Payload only. Offset 6 is reserved LE u16 zero. No state/freshness policy. */
size_t rb_motion_state_encode(const rb_motion_state_batch_t *batch, uint8_t *payload,
                              size_t capacity);
bool rb_motion_state_decode(const uint8_t *payload, size_t length, rb_motion_state_batch_t *batch);
#endif
