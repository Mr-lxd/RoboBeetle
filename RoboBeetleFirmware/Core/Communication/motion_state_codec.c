#include "motion_state_codec.h"
#include <string.h>

static void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static void put32(uint8_t *p, uint32_t v) {
    put16(p, (uint16_t)v);
    put16(p + 2, (uint16_t)(v >> 16));
}
static uint16_t get16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16);
}
static int16_t geti16(const uint8_t *p) {
    uint16_t v = get16(p);
    return (int16_t)(v < 0x8000U ? (int32_t)v : (int32_t)v - 65536);
}

size_t rb_motion_state_encode(const rb_motion_state_batch_t *b, uint8_t *p, size_t capacity) {
    size_t length, i, axis;
    if (!b || !p || b->sample_count > 2 || b->fragment_count == 0 || b->fragment_count > 16 ||
        b->fragment_index >= b->fragment_count)
        return 0;
    length = 16U + 22U * b->sample_count;
    if (capacity < length)
        return 0;
    memset(p, 0, length);
    p[0] = 1;
    p[1] = b->sample_count;
    put16(p + 2, b->batch_seq);
    p[4] = b->fragment_index;
    p[5] = b->fragment_count;
    put32(p + 8, b->mcu_tx_ms);
    put32(p + 12, b->sampler_drop_total);
    for (i = 0; i < b->sample_count; ++i) {
        const rb_motion_state_sample_t *s = &b->samples[i];
        uint8_t *q = p + 16 + 22 * i;
        put32(q, s->mcu_ms);
        for (axis = 0; axis < 3; ++axis)
            put16(q + 4 + axis * 2, (uint16_t)s->gyro_tenth_dps[axis]);
        put16(q + 10, (uint16_t)s->roll_centidegrees);
        put16(q + 12, (uint16_t)s->pitch_centidegrees);
        put16(q + 14, s->phase_u16);
        q[16] = (uint8_t)((s->active_mode & 7U) | ((s->target_mode & 7U) << 3) |
                          ((s->coordination & 1U) << 6));
        q[17] = (uint8_t)((s->backend & 3U) | ((s->state & 3U) << 2) | (s->transition << 4) |
                          (s->phase_valid << 5) | (s->gyro_valid << 6) | (s->angle_valid << 7));
        put16(q + 18, s->angle_age_ms);
        put16(q + 20, s->phase_age_ms);
    }
    return length;
}

bool rb_motion_state_decode(const uint8_t *p, size_t length, rb_motion_state_batch_t *b) {
    size_t i, axis;
    if (!p || !b || length < 16 || p[0] != 1 || p[1] > 2 || length != 16U + 22U * p[1] ||
        get16(p + 6) != 0 || p[5] == 0 || p[5] > 16 || p[4] >= p[5])
        return false;
    memset(b, 0, sizeof *b);
    b->sample_count = p[1];
    b->batch_seq = get16(p + 2);
    b->fragment_index = p[4];
    b->fragment_count = p[5];
    b->mcu_tx_ms = get32(p + 8);
    b->sampler_drop_total = get32(p + 12);
    for (i = 0; i < b->sample_count; ++i) {
        rb_motion_state_sample_t *s = &b->samples[i];
        const uint8_t *q = p + 16 + 22 * i;
        if (q[16] & 0x80U)
            return false;
        s->mcu_ms = get32(q);
        for (axis = 0; axis < 3; ++axis)
            s->gyro_tenth_dps[axis] = geti16(q + 4 + axis * 2);
        s->roll_centidegrees = geti16(q + 10);
        s->pitch_centidegrees = geti16(q + 12);
        s->phase_u16 = get16(q + 14);
        s->active_mode = q[16] & 7U;
        s->target_mode = (q[16] >> 3) & 7U;
        s->coordination = (q[16] >> 6) & 1U;
        s->backend = q[17] & 3U;
        s->state = (q[17] >> 2) & 3U;
        s->transition = (q[17] & 0x10U) != 0;
        s->phase_valid = (q[17] & 0x20U) != 0;
        s->gyro_valid = (q[17] & 0x40U) != 0;
        s->angle_valid = (q[17] & 0x80U) != 0;
        s->angle_age_ms = get16(q + 18);
        s->phase_age_ms = get16(q + 20);
    }
    return true;
}
