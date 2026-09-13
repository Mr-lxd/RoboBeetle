#ifndef ROBOBEETLE_SIMPLE_GAIT_GENERATOR_H
#define ROBOBEETLE_SIMPLE_GAIT_GENERATOR_H

#include "gait_generator.h"

#include <stdint.h>

typedef struct
{
    float phase_rad;
} simple_gait_generator_t;

void simple_gait_generator_init(
    simple_gait_generator_t *generator);

gait_generator_t simple_gait_generator_interface(
    simple_gait_generator_t *generator);

void simple_gait_generator_advance(
    simple_gait_generator_t *generator,
    uint32_t dt_ms);

bool simple_gait_generator_sample(
    simple_gait_generator_t *generator,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets);

bool simple_gait_generator_is_mode_valid(
    motion_mode_t mode);

float simple_gait_generator_phase(
    const simple_gait_generator_t *generator);

#endif /* ROBOBEETLE_SIMPLE_GAIT_GENERATOR_H */
