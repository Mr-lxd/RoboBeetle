#ifndef ROBOBEETLE_EXPERIMENTAL_FLEX_GAIT_GENERATOR_H
#define ROBOBEETLE_EXPERIMENTAL_FLEX_GAIT_GENERATOR_H

#include "gait_generator.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t phase_ms;
} experimental_flex_gait_generator_t;

void experimental_flex_gait_generator_init(
    experimental_flex_gait_generator_t *generator);

void experimental_flex_gait_generator_reset(
    experimental_flex_gait_generator_t *generator);

gait_generator_t experimental_flex_gait_generator_interface(
    experimental_flex_gait_generator_t *generator);

void experimental_flex_gait_generator_advance(
    experimental_flex_gait_generator_t *generator,
    uint32_t dt_ms);

bool experimental_flex_gait_generator_sample(
    experimental_flex_gait_generator_t *generator,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets);

bool experimental_flex_gait_generator_is_mode_valid(
    motion_mode_t mode);

#endif /* ROBOBEETLE_EXPERIMENTAL_FLEX_GAIT_GENERATOR_H */
