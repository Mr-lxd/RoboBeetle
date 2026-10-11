#include "cpg_gait_generator.h"

#include "motion_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static int32_t rounded_cdeg(double value)
{
    return (int32_t)lround(value);
}

static void cpg_profile_target_amplitudes(
    const cpg_gait_profile_t *profile,
    double target_amplitude[CPG_CORE_NODE_COUNT])
{
    target_amplitude[0] = -profile->front_amplitude_deg;
    target_amplitude[1] = profile->rear_amplitude_deg;
    target_amplitude[2] = profile->rear_amplitude_deg;
    target_amplitude[3] = -profile->front_amplitude_deg;

}

static void cpg_profile_periods(
    const cpg_gait_profile_t *profile,
    double period_s[CPG_CORE_NODE_COUNT])
{
    size_t i;

    for (i = 0U; i < CPG_CORE_NODE_COUNT; ++i)
    {
        period_s[i] = profile->nominal_period_s;
    }
}

void cpg_gait_profile_production_default(
    cpg_gait_profile_t *profile)
{
    if (profile == NULL)
    {
        return;
    }

    (void)memset(profile, 0, sizeof(*profile));
    profile->front_amplitude_deg = 10.0;
    profile->rear_amplitude_deg = 10.0;
    /* Actual ~2.000 s: D:\RoboBeetle-results\task12-cpg-period-2026-10-07; beta/amplitude/coupling/timestep changes require recalibration. */
    profile->nominal_period_s = 2.5162;
    profile->beta = .75;
    profile->coupling_strength = 2.0;
    profile->coupling_mask = 0x3c;
    profile->front_axis_bias_cdeg[MOTION_ASCEND] =
        (double)MOTION_PROFILE_ASCEND_FRONT_AXIS_BIAS_CDEG;
    profile->front_axis_bias_cdeg[MOTION_DESCEND] =
        (double)MOTION_PROFILE_DESCEND_FRONT_AXIS_BIAS_CDEG;
}

void cpg_gait_generator_init_with_profile(
    cpg_gait_generator_t *generator,
    const cpg_gait_profile_t *profile)
{
    cpg_gait_profile_t selected_profile;
    cpg_model_params_t params;
    double target_amplitude[CPG_CORE_NODE_COUNT];
    double period_s[CPG_CORE_NODE_COUNT];

    if (generator == NULL)
    {
        return;
    }

    if (profile == NULL)
    {
        cpg_gait_profile_production_default(&selected_profile);
        profile = &selected_profile;
    }

    generator->profile = *profile;
    cpg_legacy_source_compatible_default_params(&params);
    cpg_profile_target_amplitudes(
        profile,
        target_amplitude);
    cpg_profile_periods(profile, period_s);
    (void)memcpy(
        params.target_amplitude,
        target_amplitude,
        sizeof(target_amplitude));
    (void)memcpy(
        params.period_s,
        period_s,
        sizeof(period_s));
    /* Keep the original directed edge order and zero signs at defaults. */
    static const uint8_t from[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    static const uint8_t to[12] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};
    static const uint8_t bit[12] = {3, 5, 0, 3, 1, 4, 5, 1, 2, 0, 4, 2};
    const double rad = 3.14159265358979323846 / 180.0;
    const double theta[4] = {profile->left_right_phase_deg * rad,
                             (profile->front_rear_phase_deg + profile->left_right_phase_deg) * rad,
                             profile->front_rear_phase_deg * rad, 0.0};
    for (unsigned i = 0; i < 4; ++i)
        params.beta[i] = profile->beta;
    for (unsigned i = 0; i < 12; ++i)
    {
        const bool enabled = (profile->coupling_mask & (1U << bit[i])) != 0;
        params.coupling_weight[i] = enabled ? profile->coupling_strength : 0.0;
        params.phase_target_gain[i] = enabled ? 20.0 : 0.0;
        params.desired_phase[i] =
            (profile->front_rear_phase_deg == 0.0 && profile->left_right_phase_deg == 0.0)
                ? 0.0
                : theta[to[i]] - theta[from[i]];
    }
    cpg_core_init(&generator->core, &params);
    memcpy(generator->core.phase, theta, sizeof theta);
    memcpy(generator->core.phase_target, params.desired_phase, sizeof params.desired_phase);
}

void cpg_gait_generator_init(
    cpg_gait_generator_t *generator)
{
    cpg_gait_profile_t profile;

    if (generator == NULL)
    {
        return;
    }

    cpg_gait_profile_production_default(&profile);
    cpg_gait_generator_init_with_profile(generator, &profile);
}

void cpg_gait_generator_reset(
    cpg_gait_generator_t *generator)
{
    cpg_gait_profile_t profile;

    if (generator == NULL)
    {
        return;
    }

    profile = generator->profile;
    cpg_gait_generator_init_with_profile(generator, &profile);
}

void cpg_gait_generator_advance(
    cpg_gait_generator_t *generator,
    uint32_t dt_ms)
{
    if (generator == NULL)
    {
        return;
    }

    (void)cpg_core_advance_elapsed_ms(&generator->core, dt_ms);
}

bool cpg_gait_generator_sample(
    cpg_gait_generator_t *generator,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    double target_amplitude[CPG_CORE_NODE_COUNT];
    const double amplitude = (double)amplitude_scale;
    const double bias = (double)bias_scale;
    const cpg_core_t *core;

    if ((generator == NULL) ||
        (targets == NULL) ||
        !cpg_gait_generator_is_mode_valid(mode))
    {
        return false;
    }

    if (mode == MOTION_STOP)
    {
        *targets = (joint_targets_t){0};
        return true;
    }

    cpg_profile_target_amplitudes(
        &generator->profile,
        target_amplitude);
    cpg_core_set_target_amplitudes(
        &generator->core,
        target_amplitude);

    core = &generator->core;
    targets->front_right_cdeg = rounded_cdeg(
        100.0 * core->raw_output[0] * amplitude);
    targets->front_left_cdeg = rounded_cdeg(
        100.0 * core->raw_output[3] * amplitude);
    targets->rear_right_cdeg = rounded_cdeg(
        100.0 * core->raw_output[1] * amplitude);
    targets->rear_left_cdeg = rounded_cdeg(
        100.0 * core->raw_output[2] * amplitude);
    targets->front_axis_cdeg = rounded_cdeg(
        generator->profile.front_axis_bias_cdeg[mode] * bias);
    return true;
}

bool cpg_gait_generator_is_mode_valid(
    motion_mode_t mode)
{
    return motion_mode_is_valid(mode) && (mode != MOTION_BACKWARD);
}

uint32_t cpg_gait_generator_diagnostic_count(
    const cpg_gait_generator_t *generator)
{
    return generator == NULL ?
        0U :
        cpg_core_discarded_catch_up_count(&generator->core);
}

static void cpg_gait_generator_advance_interface(
    void *context,
    uint32_t dt_ms)
{
    cpg_gait_generator_advance(
        (cpg_gait_generator_t *)context,
        dt_ms);
}

static void cpg_gait_generator_reset_interface(
    void *context)
{
    cpg_gait_generator_reset((cpg_gait_generator_t *)context);
}

static bool cpg_gait_generator_sample_interface(
    void *context,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    return cpg_gait_generator_sample(
        (cpg_gait_generator_t *)context,
        mode,
        amplitude_scale,
        bias_scale,
        targets);
}

static bool cpg_gait_generator_valid_interface(
    void *context,
    motion_mode_t mode)
{
    (void)context;
    return cpg_gait_generator_is_mode_valid(mode);
}

static uint32_t cpg_gait_generator_diagnostic_interface(
    void *context)
{
    return cpg_gait_generator_diagnostic_count(
        (const cpg_gait_generator_t *)context);
}

static const gait_generator_ops_t cpg_gait_generator_ops = {
    .advance = cpg_gait_generator_advance_interface,
    .reset = cpg_gait_generator_reset_interface,
    .sample = cpg_gait_generator_sample_interface,
    .is_mode_valid = cpg_gait_generator_valid_interface,
    .diagnostic_count = cpg_gait_generator_diagnostic_interface,
};

gait_generator_t cpg_gait_generator_interface(
    cpg_gait_generator_t *generator)
{
    gait_generator_t interface = {
        .ops = &cpg_gait_generator_ops,
        .context = generator,
    };
    return interface;
}

void cpg_gait_generator_get_parameters(const cpg_gait_generator_t *g, cpg_parameters_t *p)
{
    const cpg_gait_profile_t *f = &g->profile;
    *p = (cpg_parameters_t){
        f->front_amplitude_deg,  f->rear_amplitude_deg,   f->nominal_period_s,  f->beta,
        f->front_rear_phase_deg, f->left_right_phase_deg, f->coupling_strength, f->coupling_mask};
}
void cpg_gait_generator_apply_parameters(cpg_gait_generator_t *g, const cpg_parameters_t *p)
{
    cpg_gait_profile_t f = g->profile;
    f.front_amplitude_deg = p->front_amp;
    f.rear_amplitude_deg = p->rear_amp;
    f.nominal_period_s = p->period;
    f.beta = p->beta;
    f.front_rear_phase_deg = p->F;
    f.left_right_phase_deg = p->L;
    f.coupling_strength = p->w;
    f.coupling_mask = p->mask;
    cpg_gait_generator_init_with_profile(g, &f);
}
