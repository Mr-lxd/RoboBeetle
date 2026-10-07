#ifndef ROBOBEETLE_MOTION_STATE_SAMPLER_H
#define ROBOBEETLE_MOTION_STATE_SAMPLER_H

#include "motion_state_codec.h"

#define MOTION_STATE_RING_CAPACITY 64U
#define MOTION_STATE_BATCH_CAPACITY 32U

typedef struct
{
    rb_motion_state_sample_t samples[MOTION_STATE_RING_CAPACITY];
    uint8_t head, count, batch_remaining, fragment_index, fragment_count;
    uint16_t next_batch_seq, batch_seq;
    uint32_t drop_total, batch_drop_total;
    bool batch_active;
} motion_state_sampler_t;

void motion_state_sampler_init(motion_state_sampler_t *sampler);
void motion_state_sampler_push(motion_state_sampler_t *sampler,
                               const rb_motion_state_sample_t *sample);
void motion_state_sampler_begin_batch(motion_state_sampler_t *sampler);
bool motion_state_sampler_fragment(const motion_state_sampler_t *sampler,
                                  uint32_t tx_ms, rb_motion_state_batch_t *batch);
void motion_state_sampler_accept_fragment(motion_state_sampler_t *sampler);
#endif
