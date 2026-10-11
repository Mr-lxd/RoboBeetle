#include "motion_state_sampler.h"
#include <string.h>

void motion_state_sampler_init(motion_state_sampler_t *sampler)
{
    memset(sampler, 0, sizeof *sampler);
}

void motion_state_sampler_push(motion_state_sampler_t *sampler,
                               const rb_motion_state_sample_t *sample)
{
    if (sampler->count == MOTION_STATE_RING_CAPACITY)
    {
        ++sampler->drop_total;
        return;
    }
    sampler->samples[(sampler->head + sampler->count) %
                     MOTION_STATE_RING_CAPACITY] = *sample;
    ++sampler->count;
}

void motion_state_sampler_begin_batch(motion_state_sampler_t *sampler)
{
    if (sampler->batch_active)
    {
        return;
    }
    sampler->batch_active = true;
    sampler->batch_seq = sampler->next_batch_seq++;
    sampler->batch_remaining = sampler->count > MOTION_STATE_BATCH_CAPACITY
        ? MOTION_STATE_BATCH_CAPACITY : sampler->count;
    sampler->fragment_index = 0U;
    sampler->fragment_count = sampler->batch_remaining == 0U ? 1U : sampler->batch_remaining;
    sampler->batch_drop_total = sampler->drop_total;
}

bool motion_state_sampler_fragment(const motion_state_sampler_t *sampler,
                                  uint32_t tx_ms, rb_motion_state_batch_t *batch)
{
    if (!sampler->batch_active)
    {
        return false;
    }
    memset(batch, 0, sizeof *batch);
    batch->schema = 2;
    batch->sample_count = sampler->batch_remaining > 1U ? 1U : sampler->batch_remaining;
    batch->batch_seq = sampler->batch_seq;
    batch->fragment_index = sampler->fragment_index;
    batch->fragment_count = sampler->fragment_count;
    batch->mcu_tx_ms = tx_ms;
    batch->sampler_drop_total = sampler->batch_drop_total;
    for (uint8_t i = 0; i < batch->sample_count; ++i)
    {
        batch->samples[i] = sampler->samples[(sampler->head + i) %
                                            MOTION_STATE_RING_CAPACITY];
    }
    return true;
}

void motion_state_sampler_accept_fragment(motion_state_sampler_t *sampler)
{
    const uint8_t count = sampler->batch_remaining > 1U ? 1U : sampler->batch_remaining;
    sampler->head = (sampler->head + count) % MOTION_STATE_RING_CAPACITY;
    sampler->count -= count;
    sampler->batch_remaining -= count;
    ++sampler->fragment_index;
    sampler->batch_active = sampler->fragment_index < sampler->fragment_count;
}
