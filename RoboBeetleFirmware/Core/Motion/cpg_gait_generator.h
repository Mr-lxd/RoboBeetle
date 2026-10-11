#ifndef ROBOBEETLE_CPG_GAIT_GENERATOR_H
#define ROBOBEETLE_CPG_GAIT_GENERATOR_H

#include "cpg_core.h"
#include "cpg_parameters.h"
#include "gait_generator.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    double front_amplitude_deg;
    double rear_amplitude_deg;
    double nominal_period_s;
    double beta, front_rear_phase_deg, left_right_phase_deg, coupling_strength;
    uint8_t coupling_mask;
    double front_axis_bias_cdeg[MOTION_COUNT];
} cpg_gait_profile_t;

typedef struct
{
    cpg_core_t core;
    cpg_gait_profile_t profile;
} cpg_gait_generator_t;

void cpg_gait_profile_production_default(
    cpg_gait_profile_t *profile);

void cpg_gait_generator_init(
    cpg_gait_generator_t *generator);

void cpg_gait_generator_init_with_profile(
    cpg_gait_generator_t *generator,
    const cpg_gait_profile_t *profile);

void cpg_gait_generator_get_parameters(const cpg_gait_generator_t *generator, cpg_parameters_t *parameters);
void cpg_gait_generator_apply_parameters(cpg_gait_generator_t *generator, const cpg_parameters_t *parameters);

void cpg_gait_generator_reset(
    cpg_gait_generator_t *generator);

gait_generator_t cpg_gait_generator_interface(
    cpg_gait_generator_t *generator);

void cpg_gait_generator_advance(
    cpg_gait_generator_t *generator,
    uint32_t dt_ms);

bool cpg_gait_generator_sample(
    cpg_gait_generator_t *generator,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets);

bool cpg_gait_generator_is_mode_valid(
    motion_mode_t mode);

uint32_t cpg_gait_generator_diagnostic_count(
    const cpg_gait_generator_t *generator);

#endif /* ROBOBEETLE_CPG_GAIT_GENERATOR_H */
