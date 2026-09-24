#include "simple_gait_generator.h"

#include "motion_config.h"

#include <math.h>
#include <stddef.h>

typedef struct
{
    float paddle_amplitude_cdeg;
    float front_axis_bias_cdeg;
    float rear_phase_offset_rad;
} simple_profile_t;

static const simple_profile_t profiles[MOTION_COUNT] = {
    [MOTION_STOP] = {
        .paddle_amplitude_cdeg = 0.0F,
        .front_axis_bias_cdeg = 0.0F,
        .rear_phase_offset_rad = 0.0F,
    },
    [MOTION_FORWARD] = {
        .paddle_amplitude_cdeg = MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
        .front_axis_bias_cdeg = 0.0F,
        .rear_phase_offset_rad = MOTION_PI_F,
    },
    [MOTION_TURN_LEFT] = {
        .paddle_amplitude_cdeg = MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
        .front_axis_bias_cdeg = 0.0F,
        .rear_phase_offset_rad = MOTION_PI_F,
    },
    [MOTION_TURN_RIGHT] = {
        .paddle_amplitude_cdeg = MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
        .front_axis_bias_cdeg = 0.0F,
        .rear_phase_offset_rad = MOTION_PI_F,
    },
    [MOTION_ASCEND] = {
        .paddle_amplitude_cdeg = MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
        .front_axis_bias_cdeg = MOTION_PROFILE_ASCEND_FRONT_AXIS_BIAS_CDEG,
        .rear_phase_offset_rad = MOTION_PI_F,
    },
    [MOTION_DESCEND] = {
        .paddle_amplitude_cdeg = MOTION_PROFILE_PADDLE_AMPLITUDE_CDEG,
        .front_axis_bias_cdeg = MOTION_PROFILE_DESCEND_FRONT_AXIS_BIAS_CDEG,
        .rear_phase_offset_rad = MOTION_PI_F,
    },
};

static int32_t rounded_cdeg(float value)
{
    return (int32_t)lroundf(value);
}

static int32_t paddle_target(
    float phase_rad,
    float phase_offset_rad,
    const simple_profile_t *profile,
    float amplitude_scale)
{
    const float wave = sinf(phase_rad + phase_offset_rad);
    return rounded_cdeg(
        profile->paddle_amplitude_cdeg *
        amplitude_scale * wave);
}

static void simple_gait_generator_advance_interface(
    void *context,
    uint32_t dt_ms)
{
    simple_gait_generator_advance(
        (simple_gait_generator_t *)context,
        dt_ms);
}

static void simple_gait_generator_reset_interface(
    void *context)
{
    simple_gait_generator_reset((simple_gait_generator_t *)context);
}

static bool simple_gait_generator_sample_interface(
    void *context,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    return simple_gait_generator_sample(
        (simple_gait_generator_t *)context,
        mode,
        amplitude_scale,
        bias_scale,
        targets);
}

static bool simple_gait_generator_valid_interface(
    void *context,
    motion_mode_t mode)
{
    (void)context;
    return simple_gait_generator_is_mode_valid(mode);
}

static const gait_generator_ops_t simple_gait_generator_ops = {
    .advance = simple_gait_generator_advance_interface,
    .reset = simple_gait_generator_reset_interface,
    .sample = simple_gait_generator_sample_interface,
    .is_mode_valid = simple_gait_generator_valid_interface,
    .diagnostic_count = NULL,
};

void simple_gait_generator_init(
    simple_gait_generator_t *generator)
{
    if (generator == NULL)
    {
        return;
    }

    generator->phase_rad = 0.0F;
}

void simple_gait_generator_reset(
    simple_gait_generator_t *generator)
{
    simple_gait_generator_init(generator);
}

gait_generator_t simple_gait_generator_interface(
    simple_gait_generator_t *generator)
{
    gait_generator_t interface = {
        .ops = &simple_gait_generator_ops,
        .context = generator,
    };
    return interface;
}

void simple_gait_generator_advance(
    simple_gait_generator_t *generator,
    uint32_t dt_ms)
{
    if (generator == NULL)
    {
        return;
    }

    generator->phase_rad +=
        2.0F * MOTION_PI_F * MOTION_PROFILE_FREQUENCY_HZ *
        ((float)dt_ms / 1000.0F);
    generator->phase_rad = fmodf(
        generator->phase_rad,
        2.0F * MOTION_PI_F);
}

bool simple_gait_generator_sample(
    simple_gait_generator_t *generator,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    const simple_profile_t *profile;

    if ((generator == NULL) ||
        (targets == NULL) ||
        !simple_gait_generator_is_mode_valid(mode))
    {
        return false;
    }

    if (mode == MOTION_STOP)
    {
        *targets = (joint_targets_t){0};
        return true;
    }

    profile = &profiles[mode];
    targets->front_right_cdeg = paddle_target(
        generator->phase_rad,
        0.0F,
        profile,
        amplitude_scale);
    targets->front_left_cdeg = paddle_target(
        generator->phase_rad,
        0.0F,
        profile,
        amplitude_scale);
    targets->front_axis_cdeg = rounded_cdeg(
        profile->front_axis_bias_cdeg * bias_scale);
    targets->rear_right_cdeg = paddle_target(
        generator->phase_rad,
        profile->rear_phase_offset_rad,
        profile,
        amplitude_scale);
    targets->rear_left_cdeg = paddle_target(
        generator->phase_rad,
        profile->rear_phase_offset_rad,
        profile,
        amplitude_scale);

    return true;
}

bool simple_gait_generator_is_mode_valid(
    motion_mode_t mode)
{
    return motion_mode_is_valid(mode) && (mode != MOTION_BACKWARD);
}

float simple_gait_generator_phase(
    const simple_gait_generator_t *generator)
{
    return generator == NULL ? 0.0F : generator->phase_rad;
}
