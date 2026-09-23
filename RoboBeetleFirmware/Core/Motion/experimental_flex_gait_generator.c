#include "experimental_flex_gait_generator.h"

#include "motion_config.h"

#include <math.h>
#include <stddef.h>

#define EXPERIMENTAL_FLEX_CYCLE_MS 2000U
#define EXPERIMENTAL_FLEX_POWER_MS 1300U
#define EXPERIMENTAL_FLEX_RECOVERY_MS 700U
#define EXPERIMENTAL_FLEX_RESET_PHASE_MS 650U
#define EXPERIMENTAL_FLEX_AMPLITUDE_CDEG 1000.0F

static int32_t rounded_cdeg(float value)
{
    return (int32_t)lroundf(value);
}

static float flex_position(const experimental_flex_gait_generator_t *generator)
{
    if (generator->phase_ms < EXPERIMENTAL_FLEX_POWER_MS)
    {
        const float progress =
            (float)generator->phase_ms / (float)EXPERIMENTAL_FLEX_POWER_MS;
        return -EXPERIMENTAL_FLEX_AMPLITUDE_CDEG *
               cosf(MOTION_PI_F * progress);
    }

    const float recovery_ms =
        (float)(generator->phase_ms - EXPERIMENTAL_FLEX_POWER_MS);
    const float progress = recovery_ms / (float)EXPERIMENTAL_FLEX_RECOVERY_MS;
    return EXPERIMENTAL_FLEX_AMPLITUDE_CDEG *
           cosf(MOTION_PI_F * progress);
}

static void advance_interface(void *context, uint32_t dt_ms)
{
    experimental_flex_gait_generator_advance(
        (experimental_flex_gait_generator_t *)context,
        dt_ms);
}

static void reset_interface(void *context)
{
    experimental_flex_gait_generator_reset(
        (experimental_flex_gait_generator_t *)context);
}

static bool sample_interface(
    void *context,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    return experimental_flex_gait_generator_sample(
        (experimental_flex_gait_generator_t *)context,
        mode,
        amplitude_scale,
        bias_scale,
        targets);
}

static bool valid_interface(void *context, motion_mode_t mode)
{
    (void)context;
    return experimental_flex_gait_generator_is_mode_valid(mode);
}

static const gait_generator_ops_t operations = {
    .advance = advance_interface,
    .reset = reset_interface,
    .sample = sample_interface,
    .is_mode_valid = valid_interface,
    .diagnostic_count = NULL,
};

void experimental_flex_gait_generator_init(
    experimental_flex_gait_generator_t *generator)
{
    if (generator == NULL)
    {
        return;
    }

    generator->phase_ms = EXPERIMENTAL_FLEX_RESET_PHASE_MS;
}

void experimental_flex_gait_generator_reset(
    experimental_flex_gait_generator_t *generator)
{
    experimental_flex_gait_generator_init(generator);
}

gait_generator_t experimental_flex_gait_generator_interface(
    experimental_flex_gait_generator_t *generator)
{
    const gait_generator_t interface = {
        .ops = &operations,
        .context = generator,
    };
    return interface;
}

void experimental_flex_gait_generator_advance(
    experimental_flex_gait_generator_t *generator,
    uint32_t dt_ms)
{
    if (generator == NULL)
    {
        return;
    }

    const uint32_t advance_ms = dt_ms % EXPERIMENTAL_FLEX_CYCLE_MS;
    generator->phase_ms = (uint16_t)(
        ((uint32_t)generator->phase_ms + advance_ms) %
        EXPERIMENTAL_FLEX_CYCLE_MS);
}

bool experimental_flex_gait_generator_sample(
    experimental_flex_gait_generator_t *generator,
    motion_mode_t mode,
    float amplitude_scale,
    float bias_scale,
    joint_targets_t *targets)
{
    if ((generator == NULL) || (targets == NULL) ||
        !experimental_flex_gait_generator_is_mode_valid(mode))
    {
        return false;
    }

    if (mode == MOTION_STOP)
    {
        *targets = (joint_targets_t){0};
        return true;
    }

    const int32_t q = rounded_cdeg(
        flex_position(generator) * amplitude_scale);
    int32_t front_axis = 0;
    if (mode == MOTION_ASCEND)
    {
        front_axis = rounded_cdeg(
            MOTION_PROFILE_ASCEND_FRONT_AXIS_BIAS_CDEG * bias_scale);
    }
    else if (mode == MOTION_DESCEND)
    {
        front_axis = rounded_cdeg(
            MOTION_PROFILE_DESCEND_FRONT_AXIS_BIAS_CDEG * bias_scale);
    }

    targets->front_right_cdeg = q;
    targets->front_left_cdeg = q;
    targets->front_axis_cdeg = front_axis;
    targets->rear_right_cdeg = -q;
    targets->rear_left_cdeg = -q;
    return true;
}

bool experimental_flex_gait_generator_is_mode_valid(motion_mode_t mode)
{
    return motion_mode_is_valid(mode) && (mode != MOTION_BACKWARD);
}
